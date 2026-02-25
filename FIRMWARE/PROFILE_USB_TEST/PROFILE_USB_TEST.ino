// =============================================================================
// PROFILE_USB_TEST.ino
//
// Standalone integration test — USB host + EEPROM + profile CSV + encoder.
// No thermocouple, relay, ESC, or learn mode.
//
// Hardware (same pins as furnace controller):
//   TFT ILI9486 480×320   SPI0: MISO=0, MOSI=3, SCLK=2, CS=20, DC=22, RST=21
//   Encoder A/B            PIO0: D9, D10
//   Encoder button         D8  (INPUT_PULLUP, interrupt RISING)
//   USB-A host             PIO1: D+=D27, D-=D28 (22Ω series), VBUS via polyfuse
//
// Menu (rotate to scroll, press to run):
//   Write test CSV     — write 5-step hard-coded profile to profile.csv on USB
//   Read  profile.csv  — read profile.csv from USB into memory
//   Clear loaded data  — discard in-memory steps
//   Save EEPROM slot 1 — write in-memory steps to EEPROM slot 0
//   Load EEPROM slot 1 — read EEPROM slot 0 into memory
//   Save EEPROM slot 2 — write in-memory steps to EEPROM slot 1
//   Load EEPROM slot 2 — read EEPROM slot 1 into memory
//
// EEPROM.commit() is NEVER called directly.  commitEEPROMSafe() first asks
// Core 1 to park in a spin loop (interrupts enabled, not inside TinyUSB),
// then calls EEPROM.commit(), then releases Core 1.  This prevents the
// multicore-lockout deadlock that occurs when Core 1 is inside a USB
// critical section.
//
// Serial (115200) logs every encoder tick, button press, and file operation.
// =============================================================================

#include <SPI.h>
#include <TFT_eSPI.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "SdFat_Adafruit_Fork.h"
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "quadrature.pio.h"
#include <EEPROM.h>

TFT_eSPI tft = TFT_eSPI();

// ── Encoder ──────────────────────────────────────────────────────────────────
#define ENC_A  9
#define ENC_B  10
#define ENC_SW 8

PIO  enc_pio     = pio0;
uint enc_offset  = 0;
uint enc_sm      = 0;
int32_t enc_raw_last = 0;
volatile bool btnPressed = false;

void encISR() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last > 50) { btnPressed = true; }
  last = now;
}

// ── USB host ─────────────────────────────────────────────────────────────────
#define PIN_USB_DP 27

Adafruit_USBH_Host            USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume                     fatfs;

volatile bool usbJustMounted   = false;
volatile bool usbJustUnmounted = false;
volatile bool usbMounted       = false;   // set unconditionally in mount callback
bool          usbFsReady       = false;   // FAT volume open and usable

extern "C" void tuh_msc_mount_cb(uint8_t dev_addr) {
  usbMounted = true;
  if (msc_block_dev.begin(dev_addr)) { usbJustMounted = true; }
}

extern "C" void tuh_msc_umount_cb(uint8_t dev_addr) {
  (void)dev_addr;
  usbMounted       = false;
  usbJustUnmounted = true;
}

// ── Core 1 pause / safe EEPROM commit ────────────────────────────────────────
// EEPROM.commit() calls multicore_lockout_start_blocking() which deadlocks if
// Core 1 is inside a TinyUSB critical section with interrupts disabled.
// commitEEPROMSafe() waits for Core 1 to park in a spin loop first.
volatile bool core1Ready        = false;
volatile bool core1PauseRequest = false;
volatile bool core1PauseAck     = false;
bool          pendingEEPROMCommit = false;  // set from setup(); never from loop()

void commitEEPROMSafe() {
  // Block until loop1() is running (setup1 / USB init is complete)
  while (!core1Ready) { tight_loop_contents(); }
  // Ask Core 1 to stop calling USBHost.task() and spin with interrupts enabled
  core1PauseRequest = true;
  while (!core1PauseAck) { tight_loop_contents(); }
  // Core 1 is now idle — multicore lockout in EEPROM.commit() works cleanly
  EEPROM.commit();
  // Release Core 1
  core1PauseRequest = false;
  while (core1PauseAck) { tight_loop_contents(); }
  Serial.println("[EE] committed");
}

// ── EEPROM layout ─────────────────────────────────────────────────────────────
// Total: 2 + 2×(1 + 10×8) = 164 bytes  →  EEPROM.begin(256)
#define EE_MAGIC  0xBE03
#define EE_SLOTS  2
#define EE_STEPS  10

struct __attribute__((packed)) EEStep {
  float    temp;     // 4 bytes
  uint32_t dwellMs;  // 4 bytes
};

struct __attribute__((packed)) EESlot {
  uint8_t numSteps;          // 0 = empty; >EE_STEPS = treat as corrupt
  EEStep  steps[EE_STEPS];
};

struct __attribute__((packed)) EEData {
  uint16_t magic;
  EESlot   slots[EE_SLOTS];
};

EEData eedata;

void initEEPROM() {
  EEPROM.begin(256);
  EEPROM.get(0, eedata);
  if (eedata.magic != EE_MAGIC) {
    Serial.println("[EE] uninitialised — writing defaults");
    eedata.magic = EE_MAGIC;
    for (int s = 0; s < EE_SLOTS; s++) eedata.slots[s].numSteps = 0;
    EEPROM.put(0, eedata);
    pendingEEPROMCommit = true;  // deferred: Core 1 not running yet
  } else {
    Serial.println("[EE] loaded from flash:");
    for (int s = 0; s < EE_SLOTS; s++) {
      uint8_t n = eedata.slots[s].numSteps;
      if (n == 0 || n > EE_STEPS)
        Serial.printf("  slot %d: %s\n", s + 1, n == 0 ? "empty" : "corrupt");
      else
        Serial.printf("  slot %d: %d steps\n", s + 1, n);
    }
  }
}

// ── In-memory profile ────────────────────────────────────────────────────────
#define MAX_STEPS EE_STEPS

struct Step { float temp; uint32_t dwellMs; };

Step loadedSteps[MAX_STEPS];
int  loadedCount = 0;

// Hard-coded test profile for "Write test CSV"
static const float    TEST_TEMPS[]  = { 100, 200, 350, 500, 200 };
static const uint32_t TEST_DWELLS[] = {  30,  60,   0, 120,   0 };  // seconds
#define N_TEST_STEPS 5

// ── Menu ─────────────────────────────────────────────────────────────────────
#define NUM_ITEMS 7
const char* MENU[NUM_ITEMS] = {
  "Write test CSV",
  "Read  profile.csv",
  "Clear loaded data",
  "Save EEPROM slot 1",
  "Load EEPROM slot 1",
  "Save EEPROM slot 2",
  "Load EEPROM slot 2",
};

int  menuSel          = 0;
int  lastMenuSel      = -1;
bool fullRedrawPending = true;

char statusLine[80]     = "Ready";
char lastStatusLine[80] = "";

// ── Display layout constants ──────────────────────────────────────────────────
// Title:  y=3
// Menu:   y=26..207  (7 rows × 26 px each, textSize 2 = 16 px tall)
// Sep:    y=210
// Status: y=214  (textSize 1)
// EE:     y=226 + 236  (two lines, one per slot)
// Loaded: y=250  (one summary line)
// Hint:   y=308
#define MENU_Y0  26
#define MENU_H   26
#define SEP_Y   210
#define STAT_Y  214
#define EE1_Y   226
#define EE2_Y   236
#define LOAD_Y  250
#define HINT_Y  308

void drawEESlotLine(int slot, int y) {
  tft.fillRect(0, y, 480, 9, TFT_BLACK);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, y);
  uint8_t n = eedata.slots[slot].numSteps;
  if (n == 0) {
    tft.printf("EEPROM slot %d: empty", slot + 1);
  } else if (n > EE_STEPS) {
    tft.printf("EEPROM slot %d: CORRUPT (n=%d)", slot + 1, n);
  } else {
    tft.printf("EEPROM slot %d: %d steps  (", slot + 1, n);
    for (int i = 0; i < n; i++) {
      tft.printf("%.0f", eedata.slots[slot].steps[i].temp);
      if (i < n - 1) tft.print(" ");
    }
    tft.print("C)");
  }
}

void drawLoadedLine() {
  tft.fillRect(0, LOAD_Y, 480, 9, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, LOAD_Y);
  if (loadedCount == 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.print("In-memory: empty");
  } else {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("In-memory: %d steps  (", loadedCount);
    for (int i = 0; i < loadedCount; i++) {
      tft.printf("%.0f", loadedSteps[i].temp);
      if (loadedSteps[i].dwellMs) tft.print("d");
      if (i < loadedCount - 1) tft.print(" ");
    }
    tft.print("C)");
  }
}

void drawAll() {
  tft.fillScreen(TFT_BLACK);

  // Title
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 3);
  tft.print("PROFILE + EEPROM TEST");

  // USB chip (top right)
  uint16_t usbCol = usbFsReady ? TFT_GREEN : (usbMounted ? TFT_ORANGE : TFT_DARKGREY);
  tft.fillRect(352, 2, 120, 18, usbCol);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK, usbCol);
  tft.setCursor(356, 7);
  tft.print(usbFsReady ? "USB: ready  " : (usbMounted ? "USB: mounted" : "USB: absent "));

  // Menu
  tft.setTextSize(2);
  for (int i = 0; i < NUM_ITEMS; i++) {
    uint16_t bg = (i == menuSel) ? TFT_BLUE : TFT_BLACK;
    tft.fillRect(8, MENU_Y0 + i * MENU_H, 464, MENU_H - 2, bg);
    tft.setTextColor(TFT_WHITE, bg);
    tft.setCursor(12, MENU_Y0 + i * MENU_H);
    tft.print(MENU[i]);
  }

  tft.drawFastHLine(0, SEP_Y, 480, TFT_DARKGREY);

  // Status
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, STAT_Y);
  tft.print(statusLine);

  // EEPROM slot summaries
  tft.setTextSize(1);
  drawEESlotLine(0, EE1_Y);
  drawEESlotLine(1, EE2_Y);

  // In-memory summary
  drawLoadedLine();

  // Hint
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, HINT_Y);
  tft.print("Rotate: navigate   Press: run action");

  // Sync tracking
  lastMenuSel = menuSel;
  strcpy(lastStatusLine, statusLine);
  fullRedrawPending = false;
}

void drawMenuOnly() {
  tft.setTextSize(2);
  for (int i = 0; i < NUM_ITEMS; i++) {
    uint16_t bg = (i == menuSel) ? TFT_BLUE : TFT_BLACK;
    tft.fillRect(8, MENU_Y0 + i * MENU_H, 464, MENU_H - 2, bg);
    tft.setTextColor(TFT_WHITE, bg);
    tft.setCursor(12, MENU_Y0 + i * MENU_H);
    tft.print(MENU[i]);
  }
  lastMenuSel = menuSel;
}

void drawStatus() {
  tft.setTextSize(1);
  tft.fillRect(0, STAT_Y, 480, 10, TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, STAT_Y);
  tft.print(statusLine);
  strcpy(lastStatusLine, statusLine);
}

// ── Actions ──────────────────────────────────────────────────────────────────
void doWriteCSV() {
  if (!usbFsReady) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: no USB flash");
    fullRedrawPending = true; Serial.println(statusLine); return;
  }
  File32 f;
  if (!f.open(&fatfs, "profile.csv", O_WRITE | O_CREAT | O_TRUNC)) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: cannot open profile.csv for write");
    fullRedrawPending = true; Serial.println(statusLine); return;
  }
  f.println("temp,dwell_seconds");
  for (int i = 0; i < N_TEST_STEPS; i++) {
    f.print(TEST_TEMPS[i]); f.print(","); f.println(TEST_DWELLS[i]);
  }
  f.close();
  snprintf(statusLine, sizeof(statusLine), "OK: wrote %d steps to profile.csv", N_TEST_STEPS);
  fullRedrawPending = true; Serial.println(statusLine);
}

void doReadCSV() {
  if (!usbFsReady) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: no USB flash");
    fullRedrawPending = true; Serial.println(statusLine); return;
  }
  File32 f;
  if (!f.open(&fatfs, "profile.csv", O_RDONLY)) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: profile.csv not found");
    fullRedrawPending = true; Serial.println(statusLine); return;
  }
  while (f.available() && f.read() != '\n');  // skip header line
  loadedCount = 0;
  char buf[64];
  while (f.available() && loadedCount < MAX_STEPS) {
    size_t idx = 0;
    while (f.available() && idx < sizeof(buf) - 1) {
      char ch = (char)f.read();
      if (ch == '\n' || ch == '\r') break;
      buf[idx++] = ch;
    }
    buf[idx] = '\0';
    if (idx == 0) continue;
    float temp = atof(buf);
    if (temp <= 0.0f || isnan(temp)) continue;
    loadedSteps[loadedCount].temp = temp;
    char* comma = strchr(buf, ',');
    loadedSteps[loadedCount].dwellMs = comma ? strtoul(comma + 1, NULL, 10) * 1000UL : 0;
    Serial.printf("  step %d: %.0fC  dwell %lums\n",
                  loadedCount + 1, loadedSteps[loadedCount].temp, loadedSteps[loadedCount].dwellMs);
    loadedCount++;
  }
  f.close();
  snprintf(statusLine, sizeof(statusLine), "OK: loaded %d steps from profile.csv", loadedCount);
  fullRedrawPending = true; Serial.println(statusLine);
}

void doClear() {
  loadedCount = 0;
  snprintf(statusLine, sizeof(statusLine), "Cleared");
  fullRedrawPending = true; Serial.println("[MEM] cleared");
}

void doSaveEEPROM(int slot) {
  if (loadedCount == 0) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: nothing in memory to save");
    fullRedrawPending = true; return;
  }
  EESlot& es = eedata.slots[slot];
  es.numSteps = (uint8_t)(loadedCount <= EE_STEPS ? loadedCount : EE_STEPS);
  for (int i = 0; i < es.numSteps; i++) {
    es.steps[i].temp    = loadedSteps[i].temp;
    es.steps[i].dwellMs = loadedSteps[i].dwellMs;
  }
  EEPROM.put(0, eedata);
  commitEEPROMSafe();  // safe to call from loop() — Core 1 is running
  snprintf(statusLine, sizeof(statusLine),
           "OK: saved %d steps to EEPROM slot %d", es.numSteps, slot + 1);
  fullRedrawPending = true; Serial.println(statusLine);
}

void doLoadEEPROM(int slot) {
  EESlot& es = eedata.slots[slot];
  if (es.numSteps == 0 || es.numSteps > EE_STEPS) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: slot %d is %s",
             slot + 1, es.numSteps == 0 ? "empty" : "corrupt");
    fullRedrawPending = true; Serial.println(statusLine); return;
  }
  loadedCount = es.numSteps;
  for (int i = 0; i < loadedCount; i++) {
    loadedSteps[i].temp    = es.steps[i].temp;
    loadedSteps[i].dwellMs = es.steps[i].dwellMs;
    Serial.printf("  step %d: %.0fC  dwell %lums\n",
                  i + 1, loadedSteps[i].temp, loadedSteps[i].dwellMs);
  }
  snprintf(statusLine, sizeof(statusLine),
           "OK: loaded %d steps from EEPROM slot %d", loadedCount, slot + 1);
  fullRedrawPending = true; Serial.println(statusLine);
}

// ── Core 0 setup / loop ──────────────────────────────────────────────────────
void setup() {
  set_sys_clock_khz(120000, true);
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("PROFILE+EEPROM TEST");
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, 36);
  tft.print("Starting...");

  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_SW), encISR, RISING);

  enc_offset = pio_add_program(enc_pio, &quadrature_program);
  enc_sm     = pio_claim_unused_sm(enc_pio, true);
  quadrature_program_init(enc_pio, enc_sm, enc_offset, ENC_A, ENC_B);

  initEEPROM();  // loads from flash; sets pendingEEPROMCommit if uninitialised

  Serial.println("Ready");
}

void loop() {
  // Deferred EEPROM commit (first boot only — Core 1 was not running in setup())
  if (pendingEEPROMCommit) {
    pendingEEPROMCommit = false;
    commitEEPROMSafe();
  }

  // USB events
  if (usbJustMounted) {
    usbJustMounted = false;
    Serial.println("[USB] opening FAT...");
    if (fatfs.begin(&msc_block_dev)) {
      usbFsReady = true;
      snprintf(statusLine, sizeof(statusLine), "USB flash mounted (FAT32 OK)");
      Serial.println("[USB] FAT32 OK");
    } else {
      snprintf(statusLine, sizeof(statusLine), "FAIL: FAT mount failed (not FAT32?)");
      Serial.println("[USB] FAT mount FAILED");
    }
    fullRedrawPending = true;
  }
  if (usbJustUnmounted) {
    usbJustUnmounted = false;
    usbFsReady = false;
    snprintf(statusLine, sizeof(statusLine), "USB flash removed");
    Serial.println("[USB] removed");
    fullRedrawPending = true;
  }

  // Encoder rotation
  pio_sm_exec_wait_blocking(enc_pio, enc_sm, pio_encode_in(pio_x, 32));
  int32_t raw   = (int32_t)pio_sm_get_blocking(enc_pio, enc_sm);
  int32_t delta = raw - enc_raw_last;
  enc_raw_last  = raw;
  if (delta != 0) {
    menuSel = ((menuSel + delta) % NUM_ITEMS + NUM_ITEMS) % NUM_ITEMS;
    Serial.printf("[ENC] delta=%d  sel=%d (%s)\n", delta, menuSel, MENU[menuSel]);
  }

  // Button press
  if (btnPressed) {
    btnPressed = false;
    Serial.printf("[BTN] sel=%d: %s\n", menuSel, MENU[menuSel]);
    switch (menuSel) {
      case 0: doWriteCSV();    break;
      case 1: doReadCSV();     break;
      case 2: doClear();       break;
      case 3: doSaveEEPROM(0); break;
      case 4: doLoadEEPROM(0); break;
      case 5: doSaveEEPROM(1); break;
      case 6: doLoadEEPROM(1); break;
    }
  }

  // Display
  if (fullRedrawPending) {
    drawAll();
  } else if (menuSel != lastMenuSel) {
    drawMenuOnly();
  } else if (strcmp(statusLine, lastStatusLine) != 0) {
    drawStatus();
  }

  delay(5);
}

// ── Core 1 — PIO USB host ────────────────────────────────────────────────────
void setup1() {
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp     = PIN_USB_DP;
  pio_cfg.pio_rx_num = 1;   // pio1 — pio0 used by encoder
  pio_cfg.pio_tx_num = 1;
  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1);
}

void loop1() {
  core1Ready = true;   // signal: setup1() is done, safe to pause this core
  if (core1PauseRequest) {
    core1PauseAck = true;
    while (core1PauseRequest) { tight_loop_contents(); }
    core1PauseAck = false;
    return;            // skip USBHost.task() for this iteration
  }
  USBHost.task();
}

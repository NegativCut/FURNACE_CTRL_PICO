// =============================================================================
// PROFILE_USB_TEST.ino
//
// Standalone integration test — USB host + profile CSV read/write + encoder.
// Proves these subsystems work together before wiring them into the main sketch.
// No thermocouple, no relay, no ESC, no EEPROM, no learn mode.
//
// Hardware (same pins as furnace controller):
//   TFT ILI9486 480×320   SPI0: MISO=0, MOSI=3, SCLK=2, CS=20, DC=22, RST=21
//   Encoder A/B            PIO0: D9, D10
//   Encoder button         D8  (INPUT_PULLUP, interrupt RISING)
//   USB-A host             PIO1: D+=D27, D-=D28 (22Ω series), VBUS via polyfuse
//
// Encoder: rotate to move menu cursor, press to run selected action.
//
// Menu actions:
//   Write CSV   — write a hard-coded 5-step test profile to profile.csv
//   Read CSV    — read profile.csv and show loaded steps on-screen
//   Clear       — discard loaded steps, reset display
//
// Serial output (115200) shows every operation and result.
// =============================================================================

#include <SPI.h>
#include <TFT_eSPI.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "SdFat_Adafruit_Fork.h"
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "quadrature.pio.h"

TFT_eSPI tft = TFT_eSPI();

// ── Encoder ──────────────────────────────────────────────────────────────────
#define ENC_A  9
#define ENC_B  10
#define ENC_SW 8

PIO  enc_pio    = pio0;
uint enc_offset = 0;
uint enc_sm     = 0;
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
volatile bool usbMounted       = false;  // display flag — set unconditionally in callback
bool          usbFsReady       = false;  // FAT volume is mounted and usable

extern "C" void tuh_msc_mount_cb(uint8_t dev_addr) {
  usbMounted = true;
  if (msc_block_dev.begin(dev_addr)) { usbJustMounted = true; }
}

extern "C" void tuh_msc_umount_cb(uint8_t dev_addr) {
  (void)dev_addr;
  usbMounted       = false;
  usbJustUnmounted = true;
}

// ── Profile data (in-memory) ─────────────────────────────────────────────────
#define MAX_STEPS 10

struct Step {
  float    temp;
  uint32_t dwellMs;
};

Step loadedSteps[MAX_STEPS];
int  loadedCount = 0;

// Hard-coded test profile written by "Write CSV"
static const float    TEST_TEMPS[]  = { 100, 200, 350, 500, 200 };
static const uint32_t TEST_DWELLS[] = { 30, 60, 0, 120, 0 };  // seconds
#define N_TEST_STEPS 5

// ── Display state ─────────────────────────────────────────────────────────────
#define NUM_ITEMS 3
const char* MENU[NUM_ITEMS] = {
  "Write test CSV",
  "Read  profile.csv",
  "Clear loaded data",
};

int  menuSel     = 0;
int  lastMenuSel = -1;
bool lastUsbReady       = false;
bool lastUsbMounted     = false;
int  lastLoadedCount    = -1;
bool fullRedrawPending  = true;

char statusLine[80] = "Insert USB flash drive";
char lastStatusLine[80] = "";

// ── Draw helpers ─────────────────────────────────────────────────────────────
void drawAll() {
  tft.fillScreen(TFT_BLACK);

  // Title bar
  tft.setTextSize(2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setCursor(10, 6);
  tft.print("PROFILE USB TEST");

  // USB status chip
  uint16_t usbCol = usbFsReady ? TFT_GREEN : (usbMounted ? TFT_ORANGE : TFT_DARKGREY);
  tft.fillRect(340, 4, 130, 20, usbCol);
  tft.setTextSize(1);
  tft.setTextColor(TFT_BLACK, usbCol);
  tft.setCursor(346, 9);
  tft.print(usbFsReady ? "USB: ready  " : (usbMounted ? "USB: mounted" : "USB: absent "));

  // Menu items
  tft.setTextSize(2);
  for (int i = 0; i < NUM_ITEMS; i++) {
    uint16_t bg = (i == menuSel) ? TFT_BLUE : TFT_BLACK;
    tft.fillRect(8, 36 + i * 32, 464, 24, bg);
    tft.setTextColor(TFT_WHITE, bg);
    tft.setCursor(12, 38 + i * 32);
    tft.print(MENU[i]);
  }

  // Separator
  tft.drawFastHLine(0, 136, 480, TFT_DARKGREY);

  // Status line
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 142);
  tft.print(statusLine);

  // Loaded profile steps
  tft.drawFastHLine(0, 155, 480, TFT_DARKGREY);
  if (loadedCount == 0) {
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 163);
    tft.print("No profile loaded");
  } else {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(10, 159);
    tft.printf("%d steps loaded:", loadedCount);
    for (int i = 0; i < loadedCount && i < 9; i++) {
      tft.setTextColor(i < loadedCount ? TFT_WHITE : TFT_DARKGREY, TFT_BLACK);
      tft.setCursor(10, 170 + i * 15);
      if (loadedSteps[i].dwellMs > 0) {
        tft.printf("  Step %d: %.0f C   dwell %lu s", i + 1,
                   loadedSteps[i].temp, loadedSteps[i].dwellMs / 1000UL);
      } else {
        tft.printf("  Step %d: %.0f C", i + 1, loadedSteps[i].temp);
      }
    }
    if (loadedCount > 9) {
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      tft.setCursor(10, 170 + 9 * 15);
      tft.printf("  ... (%d more)", loadedCount - 9);
    }
  }

  // Hint bar
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setCursor(10, 308);
  tft.print("Rotate: move cursor   Press: run action");

  // Sync tracking
  lastMenuSel     = menuSel;
  lastUsbReady    = usbFsReady;
  lastUsbMounted  = usbMounted;
  lastLoadedCount = loadedCount;
  strcpy(lastStatusLine, statusLine);
  fullRedrawPending = false;
}

void drawMenuOnly() {
  tft.setTextSize(2);
  for (int i = 0; i < NUM_ITEMS; i++) {
    uint16_t bg = (i == menuSel) ? TFT_BLUE : TFT_BLACK;
    tft.fillRect(8, 36 + i * 32, 464, 24, bg);
    tft.setTextColor(TFT_WHITE, bg);
    tft.setCursor(12, 38 + i * 32);
    tft.print(MENU[i]);
  }
  lastMenuSel = menuSel;
}

void drawStatus() {
  tft.setTextSize(1);
  tft.fillRect(0, 138, 480, 16, TFT_BLACK);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 142);
  tft.print(statusLine);
  strcpy(lastStatusLine, statusLine);
}

// ── Actions ──────────────────────────────────────────────────────────────────
void doWriteCSV() {
  if (!usbFsReady) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: no USB flash ready");
    fullRedrawPending = true;
    Serial.println(statusLine);
    return;
  }

  Serial.println("Writing profile.csv ...");
  File32 f;
  if (!f.open(&fatfs, "profile.csv", O_WRITE | O_CREAT | O_TRUNC)) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: could not open profile.csv for write");
    fullRedrawPending = true;
    Serial.println(statusLine);
    return;
  }

  f.println("temp,dwell_seconds");
  for (int i = 0; i < N_TEST_STEPS; i++) {
    f.print(TEST_TEMPS[i]);
    f.print(",");
    f.println(TEST_DWELLS[i]);
  }
  f.close();

  snprintf(statusLine, sizeof(statusLine),
           "OK: wrote %d steps to profile.csv", N_TEST_STEPS);
  fullRedrawPending = true;
  Serial.println(statusLine);
}

void doReadCSV() {
  if (!usbFsReady) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: no USB flash ready");
    fullRedrawPending = true;
    Serial.println(statusLine);
    return;
  }

  Serial.println("Reading profile.csv ...");
  File32 f;
  if (!f.open(&fatfs, "profile.csv", O_RDONLY)) {
    snprintf(statusLine, sizeof(statusLine), "FAIL: profile.csv not found");
    fullRedrawPending = true;
    Serial.println(statusLine);
    return;
  }

  // Skip header line
  while (f.available() && f.read() != '\n');

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
    loadedSteps[loadedCount].dwellMs =
        comma ? (uint32_t)strtoul(comma + 1, NULL, 10) * 1000UL : 0;

    Serial.printf("  step %d: %.0f C, dwell %lu ms\n",
                  loadedCount + 1,
                  loadedSteps[loadedCount].temp,
                  loadedSteps[loadedCount].dwellMs);
    loadedCount++;
  }
  f.close();

  snprintf(statusLine, sizeof(statusLine),
           "OK: loaded %d steps from profile.csv", loadedCount);
  fullRedrawPending = true;
  Serial.println(statusLine);
}

void doClear() {
  loadedCount = 0;
  snprintf(statusLine, sizeof(statusLine), "Cleared. Insert USB flash and read.");
  fullRedrawPending = true;
  Serial.println("Profile cleared");
}

// ── Core 0 ───────────────────────────────────────────────────────────────────
void setup() {
  set_sys_clock_khz(120000, true);  // PIO USB requires 120 or 240 MHz
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("PROFILE_USB_TEST");
  tft.setCursor(10, 35);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.print("Starting...");

  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_SW), encISR, RISING);

  enc_offset = pio_add_program(enc_pio, &quadrature_program);
  enc_sm     = pio_claim_unused_sm(enc_pio, true);
  quadrature_program_init(enc_pio, enc_sm, enc_offset, ENC_A, ENC_B);

  Serial.println("PROFILE_USB_TEST ready — insert flash drive");
}

void loop() {
  // ── USB events ──────────────────────────────────────────────────────────
  if (usbJustMounted) {
    usbJustMounted = false;
    Serial.println("[USB] device mounted, opening FAT ...");
    if (fatfs.begin(&msc_block_dev)) {
      usbFsReady = true;
      snprintf(statusLine, sizeof(statusLine), "USB flash mounted (FAT32 OK)");
      Serial.println("[USB] FAT32 mounted OK");
    } else {
      snprintf(statusLine, sizeof(statusLine),
               "FAIL: FAT mount failed (not FAT32?)");
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

  // ── Encoder rotation ────────────────────────────────────────────────────
  pio_sm_exec_wait_blocking(enc_pio, enc_sm, pio_encode_in(pio_x, 32));
  int32_t raw   = (int32_t)pio_sm_get_blocking(enc_pio, enc_sm);
  int32_t delta = raw - enc_raw_last;
  enc_raw_last  = raw;

  if (delta != 0) {
    menuSel = ((menuSel + delta) % NUM_ITEMS + NUM_ITEMS) % NUM_ITEMS;
    Serial.printf("[ENC] delta=%d  menuSel=%d (%s)\n", delta, menuSel, MENU[menuSel]);
  }

  // ── Button press ─────────────────────────────────────────────────────────
  if (btnPressed) {
    btnPressed = false;
    Serial.printf("[BTN] pressed: action %d (%s)\n", menuSel, MENU[menuSel]);
    switch (menuSel) {
      case 0: doWriteCSV(); break;
      case 1: doReadCSV();  break;
      case 2: doClear();    break;
    }
  }

  // ── Display update ───────────────────────────────────────────────────────
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
  USBHost.task();
}

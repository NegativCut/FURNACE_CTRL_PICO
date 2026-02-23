// ============================================================
// USB HOST MSC TEST — Furnace Controller project
// ============================================================
// Purpose: prove PIO USB host works on D27/D28 before touching
//          any furnace firmware or SD card code.
//
// What it does:
//   - Mounts a FAT32 USB flash drive on the USB-A socket
//   - Lists the root directory
//   - Reads profile.csv if present (same format as furnace uses)
//   - Writes usb_test.txt to confirm write access
//   - Detects PC connection via GPIO24 (micro-USB VBUS)
//   - Reports everything over Serial
//
// Hardware wiring (USB-A socket):
//   D27  pin 32 ── 22Ω ── USB-A pin 3  (D+)
//   D28  pin 34 ── 22Ω ── USB-A pin 2  (D−)   D− is always DP+1
//   5V supply ── 500mA polyfuse ── USB-A pin 1  (VBUS)
//   GND ────────────────────────── USB-A pin 4  (GND)
//
// Flash drive must be FAT32 formatted.
// exFAT will not mount. For large drives: Disk Management →
// format as FAT32 (or use Rufus / diskpart).
//
// Arduino IDE settings:
//   Board      : Raspberry Pi Pico
//   USB Stack  : Adafruit TinyUSB      ← must select this
//   CPU Speed  : 120 MHz               ← must be 120 or 240
//
// Libraries (all via Library Manager):
//   Adafruit TinyUSB Library
//   Pico-PIO-USB  by sekigon-gonnoc
//   SdFat - Adafruit Fork  (include as SdFat_Adafruit_Fork.h, not SdFat.h)
//
// Include order matters: SdFat_Adafruit_Fork.h must come before
// Adafruit_TinyUSB.h so that the __has_include() guard inside
// Adafruit_USBH_MSC.h resolves correctly.
// ============================================================

#include "hardware/clocks.h"
#include "SdFat_Adafruit_Fork.h"   // MUST be before Adafruit_TinyUSB.h
#include "pio_usb.h"               // MUST be before Adafruit_TinyUSB.h
#include "Adafruit_TinyUSB.h"

// ── Pin assignments ──────────────────────────────────────────
#define PIN_USB_HOST_DP   27    // D+; D− is automatically DP+1 = 28
#define PIN_VBUS_DETECT   24    // Internal Pico VBUS detect — no header pin
                                // HIGH = PC connected to micro-USB

// ── USB host objects ─────────────────────────────────────────
Adafruit_USBH_Host            USBHost;
Adafruit_USBH_MSC_BlockDevice msc_block_dev;
FatVolume                     fatfs;

// ── Inter-core signalling (Core 1 sets, Core 0 reads) ────────
volatile bool driveJustMounted   = false;
volatile bool driveJustUnmounted = false;
volatile bool driveMounted       = false;

// ============================================================
// CORE 1 — PIO USB host task
// ============================================================

void setup1() {
  // Configure PIO USB. For test Pico (no encoder) pio0 defaults are fine.
  // For main furnace firmware, add pio_rx_num=1 / pio_tx_num=1 to use pio1
  // (pio0 is reserved for the quadrature encoder there).
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;

  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1);
}

void loop1() {
  USBHost.task();   // keeps the host stack alive; runs callbacks
}

// ── TinyUSB MSC host callbacks (called from Core 1) ──────────

extern "C" void tuh_msc_mount_cb(uint8_t dev_addr) {
  if (msc_block_dev.begin(dev_addr)) {
    driveMounted       = true;
    driveJustMounted   = true;
  } else {
    Serial.println("[Core1] msc_block_dev.begin() failed");
  }
}

extern "C" void tuh_msc_umount_cb(uint8_t dev_addr) {
  (void) dev_addr;
  driveMounted         = false;
  driveJustUnmounted   = true;
}

// ============================================================
// CORE 0 — Application
// ============================================================

void setup() {
  // Clock change MUST be first — PIO USB requires 120 or 240 MHz.
  // Default is 125 MHz which will not work.
  set_sys_clock_khz(120000, true);

  Serial.begin(115200);
  while (!Serial);   // wait until Serial Monitor is opened

  Serial.println("\n=============================================");
  Serial.println("  USB Host MSC Test — Furnace project");
  Serial.println("=============================================");
  Serial.printf("CPU clock : %lu MHz\n", clock_get_hz(clk_sys) / 1000000UL);
  Serial.printf("USB host  : D%d (D+)  D%d (D-)\n",
                PIN_USB_HOST_DP, PIN_USB_HOST_DP + 1);

  pinMode(PIN_VBUS_DETECT, INPUT);
  Serial.printf("PC (µUSB) : %s\n",
                digitalRead(PIN_VBUS_DETECT) ? "connected" : "not connected");

  Serial.println("\nInsert FAT32 flash drive into USB-A socket...");
}

void loop() {
  // ── PC detection ──────────────────────────────────────────
  static bool lastPc = false;
  bool pc = digitalRead(PIN_VBUS_DETECT);
  if (pc != lastPc) {
    Serial.printf("\nPC (µUSB): %s\n", pc ? "connected" : "disconnected");
    lastPc = pc;
  }

  // ── Drive mounted ─────────────────────────────────────────
  if (driveJustMounted) {
    driveJustMounted = false;
    Serial.println("\n>>> Flash drive connected");
    runTests();
  }

  // ── Drive removed ─────────────────────────────────────────
  if (driveJustUnmounted) {
    driveJustUnmounted = false;
    Serial.println("\n>>> Flash drive removed");
    Serial.println("Insert drive again to re-run, or open Serial Monitor.");
  }

  delay(10);
}

// ============================================================
// Test sequence — called once per mount
// ============================================================

void runTests() {
  // ── 1. Mount FAT filesystem ───────────────────────────────
  Serial.println("\n--- Mounting FAT volume ---");
  if (!fatfs.begin(&msc_block_dev)) {
    Serial.println("FAIL: FAT mount failed.");
    Serial.println("      Is the drive FAT32? exFAT will not work.");
    return;
  }
  Serial.println("PASS: FAT32 mounted");

  // ── 2. List root directory ────────────────────────────────
  Serial.println("\n--- Root directory ---");
  FatFile root, entry;
  char    name[64];
  int     fileCount = 0;

  if (!root.openRoot(&fatfs)) {
    Serial.println("FAIL: could not open root");
  } else {
    while (entry.openNext(&root, O_RDONLY)) {
      if (entry.isHidden()) { entry.close(); continue; }
      entry.getName(name, sizeof(name));
      if (entry.isDir()) {
        Serial.printf("  [DIR]  %s\n", name);
      } else {
        Serial.printf("  %8lu  %s\n", (unsigned long)entry.fileSize(), name);
        fileCount++;
      }
      entry.close();
    }
    root.close();
    Serial.printf("  %d file(s) found\n", fileCount);
  }

  // ── 3. Read profile.csv if present ────────────────────────
  Serial.println("\n--- Reading profile.csv ---");
  File32 f;
  if (!f.open(&fatfs, "profile.csv", O_RDONLY)) {
    Serial.println("  profile.csv not found (not an error for this test)");
  } else {
    Serial.printf("  Size: %lu bytes\n", (unsigned long)f.fileSize());
    Serial.println("  Contents:");
    int lineNum = 0;
    char buf[80];
    while (f.available() && lineNum < 30) {       // cap at 30 lines
      size_t n = f.fgets(buf, sizeof(buf));
      if (n == 0) break;
      // strip trailing newline for display
      if (buf[n - 1] == '\n') buf[n - 1] = '\0';
      Serial.printf("  %2d: %s\n", ++lineNum, buf);
    }
    if (f.available()) Serial.println("  ... (truncated)");
    f.close();
    Serial.println("PASS: profile.csv read OK");
  }

  // ── 4. Write test file ────────────────────────────────────
  Serial.println("\n--- Writing usb_test.txt ---");
  if (!f.open(&fatfs, "usb_test.txt", O_WRITE | O_CREAT | O_TRUNC)) {
    Serial.println("FAIL: could not create usb_test.txt");
  } else {
    f.println("USB host test OK");
    f.printf("CPU: %lu MHz\n", clock_get_hz(clk_sys) / 1000000UL);
    f.printf("D+: GPIO%d   D-: GPIO%d\n",
             PIN_USB_HOST_DP, PIN_USB_HOST_DP + 1);
    f.println("FAT32 read/write confirmed.");
    f.close();
    Serial.println("PASS: usb_test.txt written");
    Serial.println("      Remove drive and check file on a PC to confirm.");
  }

  // ── 5. Re-read the file we just wrote ─────────────────────
  Serial.println("\n--- Verifying usb_test.txt ---");
  if (!f.open(&fatfs, "usb_test.txt", O_RDONLY)) {
    Serial.println("FAIL: could not re-open usb_test.txt");
  } else {
    char buf[80];
    while (f.available()) {
      size_t n = f.fgets(buf, sizeof(buf));
      if (n == 0) break;
      Serial.print("  ");
      Serial.print(buf);
    }
    f.close();
    Serial.println("PASS: verify read-back OK");
  }

  // ── Summary ───────────────────────────────────────────────
  Serial.println("\n=============================================");
  Serial.println("  Test complete. Safe to remove drive.");
  Serial.println("  Re-insert to run again.");
  Serial.println("=============================================");
}

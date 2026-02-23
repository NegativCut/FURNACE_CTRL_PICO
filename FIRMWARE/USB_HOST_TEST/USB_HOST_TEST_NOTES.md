# USB Host Test — Investigation & Results

Last updated: 2026-02-24

---

## Hardware Wiring (USB-A socket)

| USB-A Pin | Signal | Pico Connection |
|---|---|---|
| Pin 1 | VBUS (5V) | Pico VBUS pin 40 → 500mA polyfuse → USB-A pin 1 |
| Pin 2 | D− | Pico D28 (pin 34) → 22Ω → USB-A pin 2 |
| Pin 3 | D+ | Pico D27 (pin 32) → 22Ω → USB-A pin 3 |
| Pin 4 | GND | Pico GND → USB-A pin 4 |

D− is always D+ + 1 (D28 = D27 + 1). This is a hardware requirement of PIO USB.

For bench testing, 5V comes from Pico VBUS pin 40 (powered by PC via micro-USB).

---

## Arduino IDE Settings

| Setting | Value |
|---|---|
| Board | Raspberry Pi Pico |
| USB Stack | Adafruit TinyUSB ← critical |
| CPU Speed | 120 MHz ← critical (PIO USB requires 120 or 240 MHz) |

---

## Libraries Required

| Library | Source | Notes |
|---|---|---|
| Adafruit TinyUSB Library | Library Manager | v3.7.4 used |
| Pico-PIO-USB | Library Manager | by sekigon-gonnoc |
| SdFat - Adafruit Fork | Library Manager | must include as `SdFat_Adafruit_Fork.h` not `SdFat.h` |

---

## Compilation Issues Encountered & Fixes

### Issue 1 — Wrong SdFat library picked up
**Symptom:** `Adafruit_USBH_MSC_BlockDevice does not name a type`
**Cause:** Plain `SdFat` library was being used instead of `SdFat - Adafruit Fork`.
Arduino picked the wrong one because both provide `SdFat.h`.
**Fix:** Use `#include "SdFat_Adafruit_Fork.h"` — the Adafruit fork's specific header.

### Issue 2 — `Adafruit_USBH_MSC.h` explicit include failed
**Symptom:** `fatal error: Adafruit_USBH_MSC.h: No such file or directory`
**Cause:** The file lives at `src/arduino/msc/Adafruit_USBH_MSC.h` inside the library.
A direct `#include "Adafruit_USBH_MSC.h"` cannot reach it — the subdirectory isn't on
the include path. The file is only reachable via `Adafruit_TinyUSB.h`'s include chain.
**Fix:** Remove the explicit include. `Adafruit_TinyUSB.h` pulls it in automatically
when `CFG_TUH_MSC` is defined (which it is for RP2040 in the rp2040 tusb config).

### Issue 3 — `Adafruit_USBH_MSC_BlockDevice` class not defined despite correct library
**Symptom:** Class still not found even with Adafruit fork installed.
**Cause:** Inside `Adafruit_USBH_MSC.h`, the class is gated:
```cpp
#if __has_include("SdFat_Adafruit_Fork.h")
  class Adafruit_USBH_MSC_BlockDevice ...
#endif
```
The `__has_include` check fails if SdFat's include path hasn't been added to the
compiler before TinyUSB headers are processed.
**Fix:** `SdFat_Adafruit_Fork.h` must be included BEFORE `Adafruit_TinyUSB.h` in the
sketch, so Arduino adds the SdFat library path first.

### Issue 4 — `pio_usb_configuration_t` not declared
**Symptom:** `pio_usb_configuration_t was not declared in this scope`
**Cause:** `pio_usb.h` was included AFTER `Adafruit_TinyUSB.h`. Must come before.
**Fix:** Include `pio_usb.h` before `Adafruit_TinyUSB.h`.

### Issue 5 — Wrong setup1() API calls
**Symptom:** Would have caused runtime failure.
**Cause:** Sketch used `tuh_init(1)` which is the raw TinyUSB C API.
The Arduino wrapper uses `USBHost.begin(1)` instead.
Also: `pio_rx_num` and `pio_tx_num` were set unnecessarily.
For standard Pico (not Pico W), the `PIO_USB_DEFAULT_CONFIG` defaults are correct.
Only Pico W needs these overridden.
**Fix:** Replace `tuh_init(1)` with `USBHost.begin(1)`. Remove pio_rx_num/pio_tx_num.

### Issue 6 — `FatFile` has no println/printf
**Symptom:** `class FatFile has no member named println`
**Cause:** `FatFile` is a bare file class without Print methods.
**Fix:** Use `File32` for any file object that needs print/println/printf.
`FatFile` is still correct for directory listing (root, entry) — those don't need Print.

---

## Correct Include Order (critical)

```cpp
#include "hardware/clocks.h"
#include "SdFat_Adafruit_Fork.h"   // MUST be before Adafruit_TinyUSB.h
#include "pio_usb.h"               // MUST be before Adafruit_TinyUSB.h
#include "Adafruit_TinyUSB.h"
```

---

## Correct setup1() / loop1() Structure for RP2040

```cpp
void setup1() {
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_USB_HOST_DP;
  // Do NOT set pio_rx_num / pio_tx_num on standard Pico (only Pico W needs this)
  // For furnace firmware (pio0 used by encoder): add pio_rx_num=1, pio_tx_num=1
  USBHost.configure_pio_usb(1, &pio_cfg);
  USBHost.begin(1);   // NOT tuh_init(1)
}

void loop1() {
  USBHost.task();
}
```

---

## Reference Example

`Examples → Adafruit TinyUSB → DualRole → MassStorage → msc_data_logger`

The example splits the include structure into two files:
- `.ino`: includes `SdFat_Adafruit_Fork.h` first, then `usbh_helper.h`
- `usbh_helper.h`: includes `pio_usb.h` then `Adafruit_TinyUSB.h`

The setup1() → `rp2040_configure_pio_usb()` helper is defined in `usbh_helper.h`.

---

## Runtime Results

| Test | Result |
|---|---|
| Compiled cleanly | ✓ 2026-02-22 |
| CPU clock 120 MHz | ✓ confirmed in Serial output |
| D27/D28 reported correctly | ✓ |
| GPIO24 PC detection | ✓ (always HIGH during bench test — expected) |
| Flash drive mount | ✓ 2026-02-24 |
| FAT32 directory list | ✓ |
| profile.csv read | ✓ (not present on drive — correctly reported as non-error) |
| usb_test.txt write+verify | ✓ — content verified on read-back |

**Phase 1 COMPLETE — USB host proven on D27/D28 with FAT32 drive.**

### Hardware note
USB 3.0 sticks failed to enumerate — known PIO USB limitation.
Test passed using a USB 2.0 device (SD card in USB card reader).
For production use, specify USB 2.0 flash drives or verify each USB 3.0 stick individually.

---

## Notes for Furnace Firmware Integration (Phase 2)

- When integrating into furnace firmware, pio0 is used by the quadrature encoder.
  Must set `pio_cfg.pio_rx_num = 1` and `pio_cfg.pio_tx_num = 1` in setup1().
- `set_sys_clock_khz(120000, true)` must remain the first call in setup().
- Use `File32` (not `FatFile`) for any file object requiring print/write methods.
- `FatFile` is correct for directory iteration (openRoot, openNext).
- Do not change SD card code until USB host is proven end-to-end on test hardware.

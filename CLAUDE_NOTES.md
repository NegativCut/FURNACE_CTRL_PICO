# furnace_ctrl_pico — Claude Session Notes

Last updated: 2026-02-25

---

## Hardware Summary

### Current — Phase 3 complete (SD removed, USB flash is sole storage)

| Component | Arduino D / GPIO | Pico Pin |
|---|---|---|
| TFT MISO (SPI0) | D0 | 1 |
| TFT SCK (SPI0) | D2 | 4 |
| TFT MOSI (SPI0) | D3 | 5 |
| ESC / Blower (Servo PWM) | D4 | 6 |
| MAX31855 DO | D5 | 7 |
| MAX31855 CS | D6 | 9 |
| MAX31855 CLK | D7 | 10 |
| Encoder Switch (INT RISING) | D8 | 11 |
| Encoder A (PIO0) | D9 | 12 |
| Encoder B (PIO0) | D10 | 14 |
| Relay | D11 | 15 |
| TFT CS | D20 | 26 |
| TFT RST | D21 | 27 |
| TFT DC | D22 | 29 |
| Onboard LED | D25 | — |
| Pot / spare ADC | A0/D26 | 31 |
| USB host D+ | D27 | 32 |
| USB host D− | D28 | 34 |
| PC VBUS detect (internal, no wiring) | GPIO24 | — |

**Power:** 3V3 out = pin 36 · VSYS/VIN = pin 39 · VBUS = pin 40

SD card socket (D12–D15, D19) removed. SPI1 dropped from firmware.

---

## Architecture

### Current
- Firmware: `FIRMWARE/FURNACE_CTRL_PICO/FURNACE_CTRL_PICO.ino`
- TFT config: `FIRMWARE/FURNACE_CTRL_PICO/User_Setup_pi_furnace.h`
- Temperature pipeline: raw → +tcOffset → medianFilter (circular, 1–128) → EMA → `currentTempC`
- 4 main menu options: Logging, Profile run, Learn mode, Settings
- Settings + 5 profile slots stored in on-chip EEPROM (EEPROM.begin(512), 477 bytes used)
- USB flash used for: logging (`log_N.csv`), importing settings/profiles, saving learned data
- On boot: EEPROM always wins for settings; USB mount prompts for settings/profile import
- Main menu shows staircase profile graph (x=162..476, y=152..310); past=grey, current=green/cyan, future=light grey; dotted orange = current temp
- SD.h and SPI1 removed (Phase 3 complete)
- PC (micro-USB, device mode) and flash drive (USB-A on D27/D28, host mode) are **never concurrent**
- GPIO24 detects PC presence on micro-USB (internal VBUS divider, no wiring needed)

---

## Tunable Parameters in Code (all at top of .ino)

| Define | Default | Purpose |
|---|---|---|
| `DRAW_INTERVAL` | 250 ms | Min time between display redraws |
| `LOG_INTERVAL_MS` | 10000 ms | Flash log append interval |
| `LEARN_TIMEOUT_MS` | 1 800 000 | Max time for heating step to reach target (30 min) |
| `LEARN_OVERSHOOT_TIMEOUT_MS` | 300 000 | Max time in OVERSHOOT phase (5 min) |
| `LEARN_SETTLING_TIMEOUT_MS` | 900 000 | Max time in SETTLING phase (15 min) |
| `LEARN_CUTOFF_BLEND` | 0.3 | New-run weight in earlyCutoff average (0=never update, 1=no memory) |
| `DWELL_HYSTERESIS_C` | 2.0 °C | Relay re-fires when temp drops this far below target during dwell |

Settings adjustable live via encoder (no recompile): smoothing α, TC offset, median window.

---

## Topic: Learn Mode & Profile Mode

### Learn Mode Phases (per heating step)
```
HEATING   relay+ESC on until currentTempC >= cutoffTemp
           cutoffTemp = target - earlyCutoff  (if prior data), else target
           records: learnRateAtCutoff, learnTargetCrossTime, learnPeakTemp

OVERSHOOT  relay off, ESC on, tracks peak
           exits when currentTempC < learnPeakTemp - 0.5°C
           timeout: LEARN_OVERSHOOT_TIMEOUT_MS

SETTLING   relay off, ESC on
           exits when currentTempC <= target + 1.0°C
           timeout: LEARN_SETTLING_TIMEOUT_MS
           records: timeToReach, overshoot, settleTime, rateOfRise
                    earlyCutoff = overshoot × 0.8, blended 30/70 with previous

COOLING    stopAllOutputs(), waits for currentTempC <= target + 1.0°C
```

### learned.csv fields
```
targetTemp, prevSettledTemp, timeToReach, overshoot, settleTime, rateOfRise, earlyCutoff
```
Only `earlyCutoff` is read back by profile mode. The rest are stored for future use.

### Profile Mode Dwell (post-fix)
- Dwell evaluated before heating/cooling direction check
- `dwellIsHeating` flag set when dwell begins
- Heating dwell: ESC on, relay bang-bang (on if temp < target − DWELL_HYSTERESIS_C, off at target)
- Cooling dwell: relay off, ESC off — just wait

---

## Bugs Fixed

| Bug | Fix |
|---|---|
| OVERSHOOT/SETTLING had no timeout — could hang forever | Added `learnPhaseStartTime`, per-phase timeouts, records and advances on timeout |
| earlyCutoff fully overwritten each run — oscillated | 30/70 exponential blend with previous value (`LEARN_CUTOFF_BLEND`) |
| No temp maintenance during dwell | Restructured ramp block; heating dwell uses relay hysteresis |
| loadSettings() left median buffer uninitialised if window increased | Resets `tcBufferIdx`/`tcBufferCount` after loading new window size |

## Remaining Opportunities
- [ ] Use `rateOfRise` at cutoff to improve earlyCutoff prediction (stored but unused)
- [ ] Proportional blower speed during coast (currently full-on or off)
- [ ] Named profile slots (currently named "1"–"5"; name editing UI not yet implemented)

---

## Topic: USB Host — Full Storage Migration

### Architecture Decision
- **USB flash drive replaces SD card entirely** — all files move to flash
- **PC and flash drive are never connected at the same time** — no concurrent use, no priority logic
- SD card hardware (SPI1, D12–D15, D19) physically removed; SD.h dropped from firmware
- This eliminates the SdFat library conflict entirely (only one FAT library needed)

> **⚠ SEQUENCING: SD card code and hardware stay completely untouched until USB host
> is proven working on a separate test sketch. No migration until USB host is confirmed.**

### Three Operating States

| GPIO24 | PIO USB | State | Display |
|---|---|---|---|
| HIGH | idle | PC connected — programming/serial | `PC: connected` |
| LOW | drive mounted | Flash ready — operational | `Flash: ready` |
| LOW | idle | Nothing connected | `USB: ---` |

GPIO24 = internal Pico VBUS detect (micro-USB). No header pin — `digitalRead(24)` only.
PIO USB mount/unmount callbacks handle flash state.

### Hardware — Two Sockets, Fixed Roles

```
PC ──── micro-USB ────► native USB (device, CDC serial/programming)  always device mode
                         GPIO24 detects 5V here

Flash ── USB-A ────────► D27/D28 (PIO USB host, Core 1)              always host mode
           │
    5V rail via 500mA polyfuse → VBUS
    D27 via 22Ω → D+
    D28 via 22Ω → D−
```

- FAT32 only — exFAT (common on sticks >32 GB) will not mount
- Some drives fail PIO USB enumeration — try a different stick if so
- Flash draw up to 500 mA — budget on 5V rail

### Libraries (simplified — no SD conflict)
- `Adafruit TinyUSB` → `Tools → USB Stack → Adafruit TinyUSB`
- `Pico-PIO-USB` by sekigon-gonnoc
- `Adafruit SdFat fork` → `Adafruit_USBH_MSC_BlockDevice` + `FatVolume`
- `SD.h` **removed**, `SPI1` **removed**

**Reference:** `Examples → Adafruit TinyUSB → DualRole → MassStorage → msc_data_logger`

### Clock Change
PIO USB requires 120 or 240 MHz — default is 125 MHz.
`set_sys_clock_khz(120000, true)` must be the **first call in setup()**.

Impact check:
| Peripheral | Detail | Safe? |
|---|---|---|
| Servo/ESC | PWM recalculated by library | ✓ |
| SPI0 TFT at 20 MHz | 120 ÷ 20 = 6 (clean divider) | ✓ |
| Encoder PIO | Edge detection only, not frequency-sensitive | ✓ |
| UART Serial | Divider auto-recalculated | ✓ |
| SPI1 | Removed entirely | — |

### Code Changes Required

**Remove entirely:**
- `SD.h` include
- `SPI1.begin()` / `SD.begin()` / `SD.end()` / `SPI1.end()`
- `checkSDCard()` function → replaced by USB mount/unmount callbacks
- `sdCardPresent`, `sdInitialized` globals
- All `SD.open()` / `logFile` / `SD_CS` / `SD_CD` references

**Rewrite for FatVolume (file logic unchanged, just open/read/write calls):**
- `readProfile()` → `FatFile` instead of `File`
- `loadLearnedData()` / `saveLearnedData()`
- `loadSettings()` / `saveSettings()`
- `logData()` / log file init

**Add:**
- `set_sys_clock_khz(120000, true)` — first line of setup()
- `setup1()` / `loop1()` — PIO USB host task on pio1
- Mount callback: set `usbFlashMounted = true`, load settings, init log file
- Unmount callback: set `usbFlashMounted = false`, stop logging, stop profile if not mid-run
- `pinMode(24, INPUT)` + poll → `bool pcConnected`
- USB status display (replaces SD status line)

### Action Plan

**Phase 1 — Prove USB host on test hardware (standalone sketch, no furnace code touched)**
- Wire USB-A socket to a spare Pico: D27/D28 via 22Ω, VBUS via 500mA polyfuse
- Run msc_data_logger example to confirm mount, file read/write on target flash drive
- Confirm FAT32 drive enumerates cleanly
- Confirm 120 MHz clock change doesn't break anything

**Phase 2 — Prove on furnace Pico alongside existing SD code (additive only)**
- Add USB host libraries and scaffold (setup1/loop1, mount callback) to furnace sketch
- SD card code left completely intact
- Confirm USB host works alongside SPI0 (TFT), PIO0 (encoder), Servo — no interference
- Do not touch any SD or file logic

**Phase 3 — Storage refactor (only after Phase 2 confirmed)**
- Remove SD.h / SPI1 code
- Rewrite file functions for FatVolume/FatFile
- Remove SD card hardware

**Phase 4 — Clock + scaffold**
- `set_sys_clock_khz(120000, true)` as first line of setup()
- Empty setup1() / loop1() stubs
- Verify servo, TFT, encoder still work at 120 MHz before proceeding

**Phase 3 — USB host plumbing**
- Install libraries, configure USB stack
- Implement setup1() / loop1() PIO host on pio1
- Implement mount/unmount callbacks with usbFlashMounted flag
- GPIO24 poll for pcConnected

**Phase 4 — Storage refactor**
- Remove all SD.h / SPI1 code
- Rewrite readProfile(), loadLearnedData(), saveLearnedData(),
  loadSettings(), saveSettings(), logData() for FatVolume/FatFile
- On mount: auto-load settings, init log file (same auto-increment logic)
- On unmount: close log file cleanly

**Phase 5 — UI**
- Replace SD status line with USB status (PC connected / Flash ready / USB: ---)
- Remove SD-specific menu behaviour

---

## Lesson: USB mount callback pattern

The correct mount/unmount pattern was established by the working test sketch
(`FIRMWARE/USB_HOST_TEST/USB_HOST_TEST.ino`) and must not be deviated from
without a concrete, observable reason.

```cpp
// Callback (Core 1 — called from USBHost.task())
extern "C" void tuh_msc_mount_cb(uint8_t dev_addr) {
  usbFlashMounted = true;               // always — drives display
  if (msc_block_dev.begin(dev_addr)) {  // as in test sketch
    usbFlashJustMounted = true;
  }
}

// loop() (Core 0)
if (usbFlashJustMounted) {
  usbFlashJustMounted = false;
  if (fatfs.begin(&msc_block_dev)) { ... }  // filesystem init here
}
```

Rule: **read the working test sketch first; treat it as ground truth; match it.**
Do not theorise about Core 0/Core 1 deadlocks or move `begin()` around without
an observable failure (not a theory) as justification.

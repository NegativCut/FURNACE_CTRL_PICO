# furnace_ctrl_pico — Session Notes

Last updated: 2026-02-25 — Phase 3 fully tested and working

---

## Session 2026-02-25 — Phase 3: USB Storage Migration (complete, fully tested)

Tested and confirmed working: hot-plug, unplug, re-plug display flags; logging;
profile run; learn mode. All file I/O (settings, profile, learned, log) confirmed
on USB flash drive.

### What was done
Removed the SD card entirely and migrated all file I/O to a USB flash drive via
SdFat `FatVolume` / `File32`. All three phases now complete.

### Files changed
- `FIRMWARE/FURNACE_CTRL_PICO/FURNACE_CTRL_PICO.ino`
- `CLAUDE_NOTES.md`

### Changes summary
| Area | Detail |
|---|---|
| Includes | Removed `SD.h`; added `SdFat_Adafruit_Fork.h` (before `pio_usb.h`) |
| Removed globals | `File logFile`, `sdCardPresent`, `sdInitialized`, `lastSdPresent` |
| Removed defines | `SD_CS`, `SD_CD` |
| Removed function | `checkSDCard()` (~60 lines) |
| New globals | `Adafruit_USBH_MSC_BlockDevice msc_block_dev`, `FatVolume fatfs`, `usbFsReady` |
| File functions | `loadSettings`, `saveSettings`, `readProfile`, `loadLearnedData`, `saveLearnedData`, `logData` all rewritten for `File32` / `fatfs` |
| Display | SD flag block removed; USB/PC flags remain |
| Button guards | `sdCardPresent` → `usbFsReady` throughout |
| Mount handler | Loads settings, creates log file, restarts profile if active |
| Unmount handler | Clears `usbFsReady`, stops all outputs, frees profile/learned |

### SD → SdFat API mapping
| SD.h | SdFat |
|---|---|
| `SD.open(name, FILE_READ)` | `f.open(&fatfs, name, O_RDONLY)` |
| `SD.open(name, FILE_WRITE)` | `f.open(&fatfs, name, O_WRITE\|O_CREAT\|O_TRUNC)` |
| `SD.remove(name)` | `fatfs.remove(name)` |
| `SD.exists(name)` | `fatfs.exists(name)` |
| `f.seek(pos)` | `f.seekSet(pos)` |
| `f.position()` | `f.curPosition()` |
| `File` | `File32` |

---

## Correct USB mount/unmount pattern (tested, working)

Established by `FIRMWARE/USB_HOST_TEST/USB_HOST_TEST.ino`. Match it exactly.

```cpp
// Callback — Core 1, called from within USBHost.task()
extern "C" void tuh_msc_mount_cb(uint8_t dev_addr) {
  usbFlashMounted = true;               // unconditional — drives the display
  if (msc_block_dev.begin(dev_addr)) {  // begin() belongs here, in task() context
    usbFlashJustMounted = true;
  }
}

extern "C" void tuh_msc_umount_cb(uint8_t dev_addr) {
  (void) dev_addr;
  usbFlashMounted       = false;
  usbFlashJustUnmounted = true;
}

// loop() — Core 0
if (usbFlashJustMounted) {
  usbFlashJustMounted = false;
  if (fatfs.begin(&msc_block_dev)) {   // fatfs.begin() belongs here, in Core 0
    usbFsReady = true;
    // loadSettings, create log file, etc.
  }
}
```

### Why this pattern
- `msc_block_dev.begin()` issues a SCSI READ CAPACITY command. It must run inside
  the TinyUSB task() context (Core 1) where USB I/O can complete. The test sketch
  does this and hot-plug works reliably.
- `fatfs.begin()` reads FAT sectors via the already-initialised block device. Core 0
  calling this while Core 1 runs `task()` independently works correctly.
- `usbFlashMounted = true` is set unconditionally so the display updates even if
  `begin()` fails (drive physically present but unreadable).

### Lesson learned
Read the working test sketch **first**. Treat it as ground truth. Match it.
Do not move `begin()` based on theory — only on an observable, reproducible failure.
Three incorrect iterations were made before returning to the test sketch pattern.

---

## Session 2026-02-20 — Learn mode and profile bugs fixed

### Bugs fixed
1. **OVERSHOOT/SETTLING hang** — no timeout existed; steps could hang forever.
   Fix: added `learnPhaseStartTime`, per-phase timeouts (OVERSHOOT 5 min, SETTLING 15 min).
   On timeout: records current data, advances to next step.

2. **earlyCutoff oscillation** — value was fully overwritten each run, causing
   overshoot to swing between runs.
   Fix: 30/70 exponential blend with previous value (`LEARN_CUTOFF_BLEND = 0.3f`).

3. **No temp maintenance during dwell** — relay and ESC were not controlled during
   dwell periods; temperature drifted.
   Fix: `dwellIsHeating` flag. Heating dwell: ESC on, relay bang-bang with
   `DWELL_HYSTERESIS_C = 2.0f` hysteresis. Cooling dwell: both off, just wait.

4. **`loadSettings()` median buffer uninitialised** — if `tcMedianWindow` increased,
   old buffer entries from a smaller window could corrupt the filter.
   Fix: reset `tcBufferIdx` and `tcBufferCount` to 0 after loading new window size.

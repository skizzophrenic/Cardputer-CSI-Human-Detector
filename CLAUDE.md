# CLAUDE.md — WiFi-CSI Radar

> Context handoff for continuing this project in Claude Code / VS Code.
> Standalone project — a handheld WiFi-CSI people sensor with a dual-screen UI.

## Goal
A **handheld device that senses people through WiFi** using Channel State
Information (CSI). Realistic target: on-device **presence + motion**.

## Architecture
- **Sensor:** ESP32-**S3** DevKit (prototype), runs `tools/sensor_s3/sensor_s3.ino`.
  Hosts a WiFi AP (`SQUACHNET`), captures CSI from the Cardputer's own WiFi
  frames, computes amplitude variance as a motion proxy, emits `R,...` frames
  over UDP broadcast at ~15 Hz. **Future goal:** migrate to ESP32-C5 (MonsterC5)
  which has a better CSI PHY.
- **Console:** **Cardputer ADV** (ESP32-**S3**) connects to `SQUACHNET`, receives
  UDP frames on port 4210, runs the UI, drives two screens.
- **Displays (different content per screen, NOT a clone):**
  - Top = external **2.8" ILI9341** (320x240 @ rotation 7): PPI **radar scope**
    (rotating sweep + phosphor trail; motion>threshold paints stable contact
    blips — radius from RSSI, size/heat from motion).
  - Bottom = built-in 1.14" (240x135): WiFi IP, status pill, PRESENCE/CLEAR
    banner, scrolling motion graph, threshold + key hints.
- **No router needed:** S3 is the AP; Cardputer connects directly to it.
  The Cardputer's own WiFi traffic generates the CSI frames the S3 measures.

## Wire protocol (UDP port 4210, newline-terminated ASCII, ~15 Hz)
- S3 → Cardputer: `R,<seq>,<presence 0|1>,<motion 0.000-1.000>,<rssi>,<mode>`
  - e.g. `R,1042,1,0.37,-54,RUN`  (mode = CAL | RUN | IDLE)
- Cardputer → S3: `CAL` | `CALSTOP` | `THR <float>` | `RATE <hz>` | `PING`→`PONG`

## Build / run
- `pio run -e cardputer-radar -t upload` — **fake-data mode** (`RADAR_FAKE=1`),
  no sensor needed.
- `pio run -e cardputer-radar-wifi -t upload` — live WiFi UDP mode; S3 must be
  running and `SQUACHNET` must be up before the Cardputer boots.
- `cd tools/sensor_s3 && pio run -t upload` — flash the S3 sensor (COM44).
- `tools/fake_c5/fake_c5.ino` — spare-S3 UART emitter for wired link testing.

## Hard constraints (don't regress these)
- Build sets `ARDUINO_USB_CDC_ON_BOOT=1` → **`Serial` is USB-C**. Any sensor
  link MUST use a HardwareSerial (`Serial1`), never `Serial`.
- **No PSRAM** (StampS3). Don't allocate a full 320x240 sprite. Pattern: render
  at 240x180 and scale-to-fit (`pushRotateZoom`). Two 240x180x16bpp canvases
  (~84 KB each) is the budget.
- External panel = `include/ext_panel.h`: ILI9341, rotation 7,
  CS5/RST3/DC6/MOSI14/SCK40, SPI2_HOST **shared with SD** (`bus_shared=true`),
  `rgb_order=false`, 27 MHz.
- Display libs: **M5Unified + M5Cardputer (M5GFX/LovyanGFX)**, Arduino framework,
  platform espressif32@6.12.0, board m5stack-stamps3.

## Files
- `include/radar_link.h` — protocol parser + state + 240-sample motion history +
  command senders. No display deps. Has `injectLine()` for fake frames.
- `src/main.cpp` — UI: `drawBottom()` (status), `drawTop()` (radar scope),
  `drawRaycaster()` (vaporwave 3D mode), `serviceKeys()`, fake generator.
  Bottom @30 fps, top @~8 fps.
- `include/ext_panel.h` — external ILI9341 panel class.
- `tools/sensor_s3/` — S3 CSI sensor firmware (AP mode, UDP broadcast).
- `tools/fake_c5/fake_c5.ino` — spare-S3 UART protocol emitter.

## Status
✅ Dual-screen UI, parser, link layer, history, radar scope — working on real
CSI data over WiFi UDP. S3 runs as AP (`SQUACHNET`), Cardputer connects and
receives frames. Stable contact blips with EMA smoothing.
⚠️ `kScale=25.0f` in sensor_s3 needs tuning with real motion data.

## Next steps (priority order)
1. **Tune `kScale`** in `sensor_s3.ino`: the variance normalization constant
   (currently 25.0f) was guessed. Real motion data may need a different value
   to spread the motion range across 0.0–1.0 meaningfully.
2. **Port sensor to ESP32-C5** (MonsterC5): better CSI PHY (Wi-Fi 6), but
   `wifi_csi_acquire_config_t` is different — missing fields like `lltf_en`,
   `sig_mode`, `secondary_channel`, `ant`. Prototype on S3 first, then port.
3. **Calibration wizard:** `c` triggers empty-room training; scope shows a
   distinct CALIBRATING state; bottom shows progress.
4. **Real bearing** via a 2nd antenna (RSSI/phase delta) → blip angle becomes
   meaningful, turning the scope into an actual direction finder.
5. **Presence event log** on the bottom screen with optional SD persistence.

## Hardware inventory (chip cheat-sheet)
- C5 (best CSI): MonsterC5, AWOK Dual Touch (2x each), Marauder v8.
- C3: BLEShark Nanos (many; mesh-capable).
- S3: Cardputer ADV (console, COM18); S3 DevKit (sensor, COM44).
- Classic ESP32: Cheap Yellow Displays, HaleHound (CYD-based).
- Not ESP32: Cardputer Zero (Raspberry Pi CM Zero), Hackberry Pi, HackRF/PortaPack.

# Cardputer Human Detector

A handheld WiFi-CSI people sensor built on the M5Stack Cardputer ADV. Uses Channel State Information (CSI) from the device's own WiFi traffic to detect presence and motion — no camera, no PIR, no dedicated RF hardware required.

---

## How it works

WiFi signals are perturbed by moving objects (including people). By reading the Channel State Information from received WiFi frames, the device detects changes in the RF environment caused by motion. The Cardputer measures CSI from its own WiFi association traffic and computes amplitude + phase variance as a real-time motion proxy.

Two operating modes:

- **On-device CSI** (default): Cardputer connects to your home WiFi and measures CSI locally. No external sensor needed.
- **External sensor**: A separate ESP32-S3 DevKit acts as an access point (`SQUACHNET`), captures CSI, and broadcasts processed frames over UDP to the Cardputer.

---

## Hardware

**Minimum (on-device CSI mode):**

| Board | Role |
|-------|------|
| M5Stack Cardputer ADV (ESP32-S3 / StampS3) | Console UI + CSI sensor |

**Optional (external sensor mode):**

| Board | Role | Port |
|-------|------|------|
| M5Stack Cardputer ADV | Console UI | COM18 |
| ESP32-S3 DevKit | CSI sensor + WiFi AP | COM44 |

**External display wiring (ILI9341 2.8" 320×240, SPI2):**

| Signal | Pin |
|--------|-----|
| CS | 5 |
| RST | 3 |
| DC | 6 |
| MOSI | 14 |
| SCK | 40 |

Shared SPI bus with SD card (`bus_shared = true`), 27 MHz.

---

## Displays

Two screens show different content simultaneously:

- **Top — External 2.8" ILI9341 (320×240):** PPI-style radar scope. Rotating sweep with phosphor trail. Motion above threshold paints contact blips; radius from RSSI, heat/size from motion intensity.
- **Bottom — Built-in 1.14" (240×135):** WiFi IP, status pill, PRESENCE / CLEAR banner, scrolling motion graph, threshold value, key hints.

---

## Build & Flash

Requires [PlatformIO](https://platformio.org/).

### Credentials setup

Copy the example credentials file and fill in your WiFi details:

```sh
cp credentials.ini.example credentials.ini
```

Edit `credentials.ini`:
```ini
[credentials]
home_ssid = YourNetworkName
home_pass = YourPassword
```

### On-device CSI mode (default, no external sensor needed)

```sh
pio run -e cardputer-radar-csi -t upload
```

### External S3 sensor over UDP WiFi

Flash the sensor first, then the Cardputer. The Cardputer connects to `SQUACHNET` automatically on boot.

```sh
# 1. Flash the S3 sensor
cd tools/sensor_s3 && pio run -t upload

# 2. Flash the Cardputer
pio run -e cardputer-radar-wifi -t upload
```

### Fake-data mode (no sensor, no WiFi needed)

```sh
pio run -e cardputer-radar -t upload
```

Synthesizes presence episodes at ~15 Hz so the full UI animates with no hardware.

---

## Keyboard controls

| Key | Action |
|-----|--------|
| `c` | Trigger calibration (empty-room baseline) |
| `,` | Decrease detection threshold |
| `/` | Increase detection threshold |
| `` ` `` | Open settings menu |

---

## Wire protocol (external sensor mode)

UDP port 4210, newline-terminated ASCII, ~15 Hz.

**S3 → Cardputer:**
```
R,<seq>,<presence 0|1>,<motion 0.000-1.000>,<rssi>,<mode>
```
Example: `R,1042,1,0.37,-54,RUN`  — modes: `CAL` | `RUN` | `IDLE`

**Cardputer → S3:**
```
CAL | CALSTOP | THR <float> | RATE <hz> | PING → PONG
```

---

## Project structure

```
├── platformio.ini              Build environments (csi / wifi / fake)
├── partitions.csv              8MB dual-OTA partition table
├── credentials.ini.example     WiFi credentials template (copy to credentials.ini)
├── include/
│   ├── radar_link.h            Protocol parser, state, 240-sample motion history
│   └── ext_panel.h             External ILI9341 panel driver
├── src/main.cpp                Dual-screen radar UI, CSI callbacks, key handling
└── tools/
    ├── sensor_s3/              S3 DevKit firmware (AP mode, CSI capture, UDP broadcast)
    └── fake_c5/                Spare-S3 UART emitter for wired link testing
```

---

## Constraints & notes

- **No PSRAM** (StampS3). Sprites render at 240×180 and scale-to-fit via `pushRotateZoom`. Budget: two 240×180×16bpp canvases (~84 KB each).
- `ARDUINO_USB_CDC_ON_BOOT=1` — `Serial` is the USB-C port. The sensor UART always uses `Serial1` on the Grove pins.
- Platform: `espressif32@6.12.0`, board `m5stack-stamps3`, Arduino framework.

---

## Roadmap

1. Tune `kScale` in `sensor_s3.ino` (variance normalization, currently `25.0f`) with real motion data
2. Port sensor firmware to ESP32-C5 (MonsterC5) for better CSI PHY (Wi-Fi 6)
3. Calibration wizard with on-screen progress indicator
4. Dual-antenna bearing estimation → directional blips on the radar scope
5. Presence event log with optional SD card persistence

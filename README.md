# Cardputer Human Detector

A handheld WiFi-CSI people sensor built on the M5Stack Cardputer ADV. Uses Channel State Information (CSI) from the device's own WiFi traffic to detect presence and motion — no camera, no PIR, no dedicated RF hardware required.

---

## How it works

WiFi signals are perturbed by moving objects (including people). By reading the Channel State Information from received WiFi frames, the device detects changes in the RF environment caused by motion. The Cardputer measures CSI from its own WiFi association traffic and computes amplitude + phase variance as a real-time motion proxy.

---

## Hardware

| Board | Role |
|-------|------|
| M5Stack Cardputer ADV (ESP32-S3 / StampS3) | Console UI + CSI sensor |

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

### Flash

```sh
pio run -e cardputer-radar-csi -t upload
```

---

## Keyboard controls

| Key | Action |
|-----|--------|
| `c` | Trigger calibration (empty-room baseline) |
| `,` | Decrease detection threshold |
| `/` | Increase detection threshold |
| `` ` `` | Open settings menu |

---

## Project structure

```
├── platformio.ini              Build config
├── partitions.csv              8MB dual-OTA partition table
├── credentials.ini.example     WiFi credentials template (copy to credentials.ini)
├── include/
│   ├── radar_link.h            Protocol parser, state, 240-sample motion history
│   └── ext_panel.h             External ILI9341 panel driver
└── src/main.cpp                Dual-screen radar UI, CSI callbacks, key handling
```

---

## Constraints & notes

- **No PSRAM** (StampS3). Sprites render at 240×180 and scale-to-fit via `pushRotateZoom`. Budget: two 240×180×16bpp canvases (~84 KB each).
- `ARDUINO_USB_CDC_ON_BOOT=1` — `Serial` is the USB-C port.
- Platform: `espressif32@6.12.0`, board `m5stack-stamps3`, Arduino framework.

---

## Roadmap

1. Calibration wizard with on-screen progress indicator
2. Dual-antenna bearing estimation → directional blips on the radar scope
3. Presence event log with optional SD card persistence

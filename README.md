# Cardputer Human Detector

This tiny cyber deck is detecting people using Wi-Fi. No camera. No IR sensor. No thermal camera. Just Wi-Fi and this deeply cursed little radar screen.

It uses Channel State Information (CSI) — the same Wi-Fi your router is already blasting everywhere — to detect when people are moving around. The human body is mostly water, and water messes with radio waves in very measurable ways. So we measure them.

The whole build is around $50. Cardputer ADV is $30, grab [two of these TFT screens for $20](https://a.co/d/0bZbfTqO), find somebody with a 3D printer, and you've got yourself a handheld human radar that looks absolutely unhinged. I'm really, really happy with how this came out.

---

## How it works

Wi-Fi signals bounce off everything in a room — walls, furniture, doors, your cat, and that weird little flesh bag you call your body. CSI data captures the *fingerprint* of those reflections. When something moves, that fingerprint changes. This firmware tracks those changes and uses them to detect presence and motion in real time.

It runs entirely on the Cardputer ADV. No external sensor, no second device. The ESP32-S3 is dual-core, so one core handles the sensing while the other drives the screens. Everything is self-contained.

---

## Hardware

| Board | Role |
|-------|------|
| M5Stack Cardputer ADV (ESP32-S3 / StampS3) | Console UI + CSI sensor |

**External display: [ILI9341 2.8" TFT (320×240)](https://a.co/d/0bZbfTqO)**

Wire it up to the Cardputer ADV GPIO like this (SPI2, shared with SD):

| Signal | Pin |
|--------|-----|
| CS | 5 |
| RST | 3 |
| DC | 6 |
| MOSI | 14 |
| SCK | 40 |

---

## Displays

Both screens are running at the same time showing different things — not a clone, actual different content:

- **Top — External 2.8" ILI9341 (320×240):** PPI-style radar scope. Rotating sweep with a phosphor trail. When motion crosses the threshold it paints contact blips — radius from RSSI, heat and size from motion intensity.
- **Bottom — Built-in 1.14" (240×135):** WiFi IP, status pill, PRESENCE / CLEAR banner, scrolling motion graph, threshold value, and key hints.

---

## Flash the pre-built binary (easiest)

Don't want to deal with PlatformIO? Grab the latest `.bin` from the [Releases page](../../releases) and flash it in your browser using [ESP Web Flasher](https://esptool.spacehuhn.com/):

1. Plug in your Cardputer ADV via USB-C
2. Go to [esptool.spacehuhn.com](https://esptool.spacehuhn.com/)
3. Click **Connect** and select your device
4. Upload the `.bin` from the latest release
5. Done

On first boot it'll scan for networks, let you pick one, and ask for the password. Credentials get saved to the device so it won't ask again.

---

## Build from source

Requires [PlatformIO](https://platformio.org/).

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

## Dev notes

- **No PSRAM** (StampS3). Sprites render at 240×180 and scale-to-fit via `pushRotateZoom`. Budget: two 240×180×16bpp canvases (~84 KB each).
- `ARDUINO_USB_CDC_ON_BOOT=1` — `Serial` is the USB-C port.
- Platform: `espressif32@6.12.0`, board `m5stack-stamps3`, Arduino framework.

---

## Roadmap

1. Calibration wizard with on-screen progress indicator
2. Dual-antenna bearing estimation → directional blips on the radar scope
3. Presence event log with optional SD card persistence

---

## The terrifying part

There's almost nothing you can do to protect yourself from this. Wi-Fi goes through walls, floors, and pretty much anything else — because that's the whole point of Wi-Fi. This technology holds a ton of promise for things like elderly fall detection and hospital monitoring, but the privacy implications are real.

I built this to show that you don't need a $60 billion government contract. You just need $50 and a little bit of tinkering time to build something that does what seemingly should be impossible.

Are we okay with this? Drop a comment on the video and let me know.

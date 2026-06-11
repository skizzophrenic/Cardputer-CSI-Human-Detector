# WiFi-CSI Radar

A handheld WiFi people sensor. The ESP32-S3 sensor reads Channel State
Information (CSI) to detect presence and motion, broadcasting results wirelessly
to a **Cardputer ADV** console that renders a dual-screen radar UI.

---

## Hardware

| Board | Role | Port |
|---|---|---|
| ESP32-S3 DevKit | CSI sensor + WiFi AP (`SQUACHNET`) | COM44 |
| M5Stack Cardputer ADV | Radar console UI | COM18 |

---

## Quick start

**Flash the sensor (S3):**
```bash
cd tools/sensor_s3
pio run -t upload
```

**Flash the console (Cardputer) — WiFi UDP mode:**
```bash
pio run -e cardputer-radar-wifi -t upload
```

Boot order: flash the S3 first. Once it's up and `SQUACHNET` is visible, boot
the Cardputer — it connects automatically and the title bar shows
`[ UDP 192.168.4.x ]` when linked.

**No hardware? Fake-data mode:**
```bash
pio run -e cardputer-radar -t upload
```
Synthesizes presence episodes at ~15 Hz so the full UI animates with no sensor.

---

## Wire protocol

UDP port 4210, newline-terminated ASCII, ~15 Hz.

```
S3 → Cardputer:   R,<seq>,<presence>,<motion>,<rssi>,RUN
Cardputer → S3:   CAL | CALSTOP | THR <float> | RATE <hz> | PING
```

---

## Files

```
├── platformio.ini              cardputer-radar (fake) / -wifi / -live envs
├── partitions.csv              8MB dual-OTA table
├── include/radar_link.h        protocol parser + state + motion history
├── include/ext_panel.h         external ILI9341 panel driver
├── src/main.cpp                dual-screen radar UI
└── tools/
    ├── sensor_s3/              S3 CSI sensor firmware (AP mode)
    └── fake_c5/                spare-S3 UART emitter for wired link testing
```

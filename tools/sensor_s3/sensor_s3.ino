// sensor_s3.ino  —  WiFi-CSI presence/motion sensor, ESP32-S3 prototype.
//
// Dual-mode WiFi:
//   DIRECT — tries SQUACHNET (WIFI_SSID) first; Cardputer hosts that AP.
//            Static IP 192.168.4.2, sends UDP to 192.168.4.1 (Cardputer).
//            Cardputer floods 100Hz probes back → high CSI sample rate.
//   HOME   — falls back to HOME_SSID if SQUACHNET not found.
//            Dynamic IP (DHCP), sends UDP to subnet broadcast.
//            Rich traffic from all home devices → excellent CSI quality.
//
// Wiring to Cardputer (optional UART fallback):
//   GPIO17 (TX) → Cardputer Grove G1 (RX)
//   GPIO18 (RX) ← Cardputer Grove G2 (TX)
//   GND         → GND

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <FastLED.h>
#include <math.h>
#include "esp_wifi.h"

#define RADAR_UDP_PORT 4210

#ifndef HOME_SSID
#define HOME_SSID "your_home_wifi"
#endif
#ifndef HOME_PASS
#define HOME_PASS "your_home_pass"
#endif

// ── Hardware ──────────────────────────────────────────────────────────────────
#define LED_PIN   48
#define NUM_LEDS  1
#define LINK_TX   17
#define LINK_RX   18

CRGB           leds[NUM_LEDS];
HardwareSerial Link(1);
WiFiUDP        udpOut;

static IPAddress gUdpTarget;

// ── CSI signal processing ─────────────────────────────────────────────────────
static const int kWindow = 50;
static float  gAmpBuf[kWindow] = {};
static int    gAmpIdx    = 0;
static int    gAmpFilled = 0;
static volatile float    gMotion   = 0.0f;
static volatile bool     gPresent  = false;
static volatile int      gLastRSSI = -80;
static volatile uint32_t gCsiCount = 0;

static const float kMotionThresh = 0.15f;
static const int   kPresenceHold = 10;
static int         gHoldCount   = 0;

// Running max: highest variance ever seen (decays very slowly so the scale
// can rescale if the environment gets quieter over time).
static float gVarMax = 0.001f;

static void IRAM_ATTR csiCallback(void* ctx, wifi_csi_info_t* info) {
    if (!info || !info->buf || info->len < 4) return;
    gCsiCount++;

    int8_t* b      = info->buf;
    int     nPairs = info->len / 2;
    float   sum    = 0.0f;
    for (int i = 0; i < nPairs; i++) {
        float r  = (float)b[2 * i];
        float im = (float)b[2 * i + 1];
        sum += sqrtf(r * r + im * im);
    }
    float meanAmp = sum / (float)nPairs;

    gAmpBuf[gAmpIdx] = meanAmp;
    gAmpIdx          = (gAmpIdx + 1) % kWindow;
    if (gAmpFilled < kWindow) gAmpFilled++;

    int   n    = gAmpFilled;
    float vsum = 0.0f;
    for (int i = 0; i < n; i++) vsum += gAmpBuf[i];
    float mean = vsum / (float)n;
    float var  = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = gAmpBuf[i] - mean;
        var += d * d;
    }
    var /= (float)n;

    // Max-normalise: fraction of the highest variance seen.
    // Faster decay (0.005) keeps gVarMax tracking reality on busy home networks.
    if (var > gVarMax) gVarMax = var;
    else               gVarMax += (var - gVarMax) * 0.005f;

    float motion = var / gVarMax;
    if (motion > 1.0f) motion = 1.0f;

    gMotion   = motion;
    gLastRSSI = info->rx_ctrl.rssi;

    if (motion > 0.15f) {
        gHoldCount = kPresenceHold;
        gPresent   = true;
    } else if (gHoldCount > 0) {
        gHoldCount--;
    } else {
        gPresent = false;
    }
}

// ── Promiscuous rx (discards data, ensures CSI fires on all frame types) ──────
static void promiscuousRx(void* buf, wifi_promiscuous_pkt_type_t type) {
    (void)buf; (void)type;
}

// ── NeoPixel ──────────────────────────────────────────────────────────────────
static uint8_t gHue = 0;

static void updateLED(bool present, uint32_t now) {
    if (!present) {
        leds[0] = CHSV(gHue++, 220, 28);
    } else {
        float b = 0.5f + 0.5f * sinf((float)now / 280.0f);
        leds[0] = CHSV(0, 255, (uint8_t)(18 + 70 * b));
    }
    FastLED.show();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Link.begin(115200, SERIAL_8N1, LINK_RX, LINK_TX);

    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(255);
    leds[0] = CRGB::Black;
    FastLED.show();

    // Connect to home WiFi.
    WiFi.mode(WIFI_STA);
    Serial.printf("# Connecting to %s...\n", HOME_SSID);
    WiFi.begin(HOME_SSID, HOME_PASS);
    {
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
            leds[0] = CHSV(gHue, 200, 20); gHue += 5;
            FastLED.show();
            delay(200);
        }
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("# WiFi failed — rebooting in 3s...");
        delay(3000);
        ESP.restart();
    }
    gUdpTarget = WiFi.broadcastIP();
    Serial.printf("# WiFi: %s  IP:%s  UDP→%s\n",
                  HOME_SSID,
                  WiFi.localIP().toString().c_str(),
                  gUdpTarget.toString().c_str());

    udpOut.begin(4211);
    Serial.printf("# UDP → %s:%d\n", gUdpTarget.toString().c_str(), RADAR_UDP_PORT);

    // Promiscuous mode — capture all frame types for maximum CSI coverage.
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promiscuousRx);

    // CSI config — enable all LTF types.
    wifi_csi_config_t cfg = {};
    cfg.lltf_en           = true;
    cfg.htltf_en          = true;
    cfg.stbc_htltf2_en    = true;
    cfg.ltf_merge_en      = true;
    cfg.channel_filter_en = true;
    cfg.manu_scale        = false;
    cfg.shift             = 0;

    esp_err_t e;
    e = esp_wifi_set_csi_config(&cfg);
    Serial.printf("# csi_config: %s\n", esp_err_to_name(e));
    e = esp_wifi_set_csi_rx_cb(csiCallback, nullptr);
    Serial.printf("# csi_rx_cb:  %s\n", esp_err_to_name(e));
    e = esp_wifi_set_csi(true);
    Serial.printf("# csi_enable: %s\n", esp_err_to_name(e));

    Serial.println("# CSI active — radar protocol on USB + UART + UDP");
    leds[0] = CHSV(160, 255, 30);  // blue = home WiFi connected
    FastLED.show();
    delay(500);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
static uint32_t seq       = 0;
static uint32_t lastFrame = 0;
static uint32_t lastDbg   = 0;

void loop() {
    uint32_t now = millis();

    // If WiFi drops (e.g. Cardputer switched NODE→HOME and killed its softAP),
    // reboot after 5 s so setup() re-runs the SQUACHNET→home fallback logic.
    static uint32_t wifiLostAt = 0;
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiLostAt == 0) wifiLostAt = now;
        else if (now - wifiLostAt > 5000) { delay(500); ESP.restart(); }
    } else {
        wifiLostAt = 0;
    }

    updateLED(gPresent, now);

    // Debug line every 5 s.
    if (now - lastDbg >= 5000) {
        lastDbg = now;
        Serial.printf("# csi_frames=%lu  motion=%.4f  present=%d\n",
                      (unsigned long)gCsiCount, (float)gMotion, (int)gPresent);
    }

    if (now - lastFrame < 66) return;
    lastFrame = now;

    float mot  = gMotion;
    int   pres = gPresent ? 1 : 0;
    int   rssi = gLastRSSI;

    Serial.printf("R,%lu,%d,%.3f,%d,RUN\n", (unsigned long)(++seq), pres, mot, rssi);
    Link.printf( "R,%lu,%d,%.3f,%d,RUN\n", (unsigned long)(seq),   pres, mot, rssi);

    udpOut.beginPacket(gUdpTarget, RADAR_UDP_PORT);
    udpOut.printf("R,%lu,%d,%.3f,%d,RUN\n", (unsigned long)(seq), pres, mot, rssi);
    udpOut.endPacket();
}

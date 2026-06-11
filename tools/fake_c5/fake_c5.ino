#include <Arduino.h>
#include <FastLED.h>
#include <math.h>

// ── NeoPixel ────────────────────────────────────────────────────────────────
#define LED_PIN  48
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// ── UART to Cardputer (mirrors USB serial) ───────────────────────────────────
// S3 GPIO17(TX) → Cardputer Grove G1(RX)
// S3 GPIO18(RX) ← Cardputer Grove G2(TX)   [optional, for commands back]
// + shared GND
#define LINK_TX 17
#define LINK_RX 18
HardwareSerial Link(1);

// ── Radar sim state ──────────────────────────────────────────────────────────
static uint32_t seq = 0, lastFrame = 0, until = 0;
static float    phase = 0.0f, thr = 0.35f;
static bool     person = false;

// ── Helpers ──────────────────────────────────────────────────────────────────
static uint8_t rainbowHue = 0;

static void updateLED(bool present, uint32_t now) {
  if (!present) {
    // slow rainbow at low brightness
    leds[0] = CHSV(rainbowHue++, 220, 28);
  } else {
    // breathe red — faster pulse = more excited
    float breath = 0.5f + 0.5f * sinf((float)now / 280.0f);
    leds[0] = CHSV(0, 255, (uint8_t)(18 + 70 * breath));
  }
  FastLED.show();
}

void setup() {
  Serial.begin(115200);
  Link.begin(115200, SERIAL_8N1, LINK_RX, LINK_TX);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);  // brightness controlled per-pixel above
  leds[0] = CRGB::Black;
  FastLED.show();
  randomSeed(micros());
  Serial.println("# fake_c5 ready — radar sim output on USB serial");
}

void loop() {
  uint32_t now = millis();

  updateLED(person, now);

  if (now - lastFrame < 66) return;  // ~15 Hz
  lastFrame = now;

  // toggle person presence on a random timer
  if (now > until) {
    person = !person;
    until  = now + (person ? random(4000, 9000) : random(3000, 7000));
    Serial.printf("# --> %s\n", person ? "PERSON IN SCENE" : "scene clear");
  }

  float base  = person ? 0.25f : 0.02f;
  phase      += 0.20f;
  float wig   = person ? 0.18f * (0.5f + 0.5f * sinf(phase)) : 0.0f;
  float noise = (random(0, 100) / 100.0f) * (person ? 0.12f : 0.04f);
  float mot   = base + wig + noise;
  if (mot > 1.0f) mot = 1.0f;

  int pres = (mot > thr) ? 1 : 0;
  int rssi = -45 - (int)random(0, 25);

  Serial.printf("R,%lu,%d,%.3f,%d,RUN\n",
                (unsigned long)(++seq), pres, mot, rssi);
  Link.printf("R,%lu,%d,%.3f,%d,RUN\n",
              (unsigned long)(seq), pres, mot, rssi);
}

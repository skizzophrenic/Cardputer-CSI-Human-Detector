#pragma once
#include <Arduino.h>

// =============================================================================
// RadarLink  -  Cardputer ADV (S3) side of the WiFi-CSI people sensor.
//
// Parses the inbound serial stream from the Monster C5 (running esp-radar),
// keeps the latest sensor state plus a rolling motion history for graphing,
// and sends commands back to the C5.
//
// Wire protocol (newline-terminated ASCII, default 115200 baud over Grove UART):
//   C5  -> Cardputer :  R,<seq>,<presence 0|1>,<motion 0.000-1.000>,<rssi>,<mode>
//                       e.g.  R,1042,1,0.37,-54,RUN
//   Cardputer -> C5  :  CAL | CALSTOP | THR <float> | RATE <hz> | PING
//
// IMPORTANT: this project builds with ARDUINO_USB_CDC_ON_BOOT=1, so `Serial`
// is the USB-C port. Always hand this class a HardwareSerial (Serial1/Serial2)
// bound to the Grove pins -- never `Serial`.
//
// Design notes:
//   * No String / no heap churn (no PSRAM on the StampS3) - fixed char buffers.
//   * poll() is non-blocking; call it every loop().
//   * Robust against partial lines, overruns, and junk (counts, never crashes).
//   * _hist is zero-initialized so the test/fake path works even if begin()
//     is never called (no live UART).
// =============================================================================
class RadarLink {
public:
  static constexpr int kHistory = 240;  // motion samples retained for the graph
  static constexpr int kLineMax = 96;   // max inbound line length (bytes)

  struct State {
    uint32_t seq      = 0;        // last sequence number from the C5
    bool     presence = false;    // someone detected?
    float    motion   = 0.0f;     // normalized activity 0..1
    int      rssi     = 0;        // dBm of the link being sensed
    char     mode[8]  = "INIT";   // CAL / RUN / IDLE ...
    uint32_t lastRxMs = 0;        // millis() of last good frame
    uint32_t frames   = 0;        // good frames parsed
    uint32_t dropped  = 0;        // frames missed (detected via seq gaps)
    uint32_t parseErr = 0;        // malformed lines / overruns
  };

  // Bind to a HardwareSerial on the Grove pins. Example (Cardputer Grove = G1/G2):
  //   radar.begin(Serial1, /*rx=*/1, /*tx=*/2);
  void begin(HardwareSerial& uart, int rxPin, int txPin, uint32_t baud = 115200) {
    _uart = &uart;
    _uart->begin(baud, SERIAL_8N1, rxPin, txPin);
    _len  = 0;
    _head = 0;
    for (int i = 0; i < kHistory; ++i) _hist[i] = 0.0f;
  }

  // Non-blocking. Returns true if at least one new frame was parsed this call.
  bool poll() {
    if (!_uart) return false;
    bool updated = false;
    while (_uart->available()) {
      char c = (char)_uart->read();
      if (c == '\r') continue;
      if (c == '\n') {
        _line[_len] = '\0';
        if (_len > 0) updated |= parseLine(_line);
        _len = 0;
      } else if (_len < kLineMax - 1) {
        _line[_len++] = c;
      } else {
        _len = 0;                 // line overrun -> drop it
        _state.parseErr++;
      }
    }
    return updated;
  }

  const State& state() const { return _state; }

  // True if no good frame has arrived for `ms` (sensor unplugged / hung).
  bool stale(uint32_t ms = 500) const { return (millis() - _state.lastRxMs) > ms; }

  // Motion history for plotting. idx 0 = oldest, historySize()-1 = newest.
  float historyAt(int idx) const { return _hist[(_head + idx) % kHistory]; }
  static constexpr int historySize() { return kHistory; }

  // ---- commands to the C5 ----------------------------------------------------
  void calibrate()           { send("CAL"); }
  void stopCalibrate()       { send("CALSTOP"); }
  void setThreshold(float t) { char b[24]; snprintf(b, sizeof(b), "THR %.2f", t); send(b); }
  void setRate(int hz)       { char b[16]; snprintf(b, sizeof(b), "RATE %d", hz); send(b); }
  void ping()                { send("PING"); }

  // ---- optional test hook: inject a synthetic frame (no C5 needed) -----------
  // Lets you bring up the UI before the sensor exists: feed it "R,..." strings.
  bool injectLine(const char* s) {
    char buf[kLineMax];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    return parseLine(buf);
  }

private:
  void send(const char* cmd) {
    if (!_uart) return;
    _uart->print(cmd);
    _uart->print('\n');
  }

  // Parses one "R,seq,presence,motion,rssi,mode" line in place. Returns true on
  // a valid frame. Ignores anything not starting with "R," (e.g. PONG) for now.
  bool parseLine(char* s) {
    if (s[0] != 'R' || s[1] != ',') return false;

    char* tok[6];
    int   n = 0;
    char* p = s;
    tok[n++] = p;                       // tok[0] -> "R"
    while (*p && n < 6) {
      if (*p == ',') { *p = '\0'; tok[n++] = p + 1; }
      p++;
    }
    if (n < 6) { _state.parseErr++; return false; }

    uint32_t seq  = strtoul(tok[1], nullptr, 10);
    int      pres = atoi(tok[2]);
    float    mot  = atof(tok[3]);
    int      rssi = atoi(tok[4]);
    if (mot < 0.0f) mot = 0.0f;
    if (mot > 1.0f) mot = 1.0f;

    if (_state.frames > 0 && seq > _state.seq + 1)
      _state.dropped += (seq - _state.seq - 1);

    _state.seq      = seq;
    _state.presence = (pres != 0);
    _state.motion   = mot;
    _state.rssi     = rssi;
    strncpy(_state.mode, tok[5], sizeof(_state.mode) - 1);
    _state.mode[sizeof(_state.mode) - 1] = '\0';
    _state.lastRxMs = millis();
    _state.frames++;

    _hist[_head] = mot;                 // push into ring buffer
    _head = (_head + 1) % kHistory;
    return true;
  }

  HardwareSerial* _uart = nullptr;
  char  _line[kLineMax];
  int   _len  = 0;
  State _state;
  float _hist[kHistory] = {};           // zero-init: safe without begin()
  int   _head = 0;
};

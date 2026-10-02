#pragma once
// Deterministic native ESP32 serial surface. Physics evolves only with the
// simulator clock, including intervals with no transaction or firmware call.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <vector>
using std::max;
using std::min;
constexpr int SERIAL_8N1 = 0;
// Waveshare documents DDSM115 0x64 position feedback as 0..32767 per turn.
inline float simCountsPerRev = 32768;
struct SimMotor {
  uint8_t mode = 3, error = 0;
  float rpm = 0, currentMa = 0, position = 32762, externalLoad = 0;
  int temp = 25;
  bool absent = false, corrupt = false, ignoreMode = false, stuck = false;
  int feedbackCurrent = 0;
  unsigned queries = 0, commands = 0;
};
inline SimMotor simMotor[4];
inline uint64_t simTimeUs = 0;
inline unsigned unsafePositionCommands = 0;
inline std::vector<unsigned> currentModeQueries;
inline unsigned allCurrentModesQueried = 0;
inline int injectStopAfterWheel = 0;
inline uint32_t millis() { return static_cast<uint32_t>(simTimeUs / 1000); }
inline uint32_t micros() { return static_cast<uint32_t>(simTimeUs); }
inline void advanceUs(uint64_t us) {
  while (us) {
    const uint64_t step = min(us, uint64_t(1000));
    const float dt = step / 1000000.0f;
    for (auto &m : simMotor) {
      if (m.stuck)
        m.rpm = 0;
      else if (m.mode == 1)
        m.rpm += (m.currentMa * 0.16f - m.rpm * 5.0f + m.externalLoad) * dt;
      else if (m.mode == 2)
        m.rpm *= std::exp(-35.0f * dt);
      m.position = std::fmod(m.position + m.rpm * (simCountsPerRev / 60.0f) * dt +
                                 simCountsPerRev,
                             simCountsPerRev);
    }
    simTimeUs += step;
    us -= step;
  }
}
inline void delay(unsigned long ms) { advanceUs(ms * 1000); }
inline void delayMicroseconds(unsigned int us) { advanceUs(us); }
inline uint8_t simCrc8(const uint8_t *p, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int b = 0; b < 8; b++)
      c = (c & 1) ? uint8_t((c >> 1) ^ 0x8c) : uint8_t(c >> 1);
  }
  return c;
}
inline uint16_t simCrc16(const uint8_t *p, size_t n) {
  uint16_t c = 65535;
  for (size_t i = 0; i < n; i++) {
    c ^= uint16_t(p[i]) << 8;
    for (int b = 0; b < 8; b++)
      c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
  }
  return c;
}
struct FakeSerial {
  bool motor = false;
  std::deque<uint8_t> incoming;
  std::vector<uint8_t> outgoing;
  uint64_t ready = 0;
  int writeRoom = 4096;
  void begin(unsigned long) {}
  void begin(unsigned long, int, int, int) {}
  void setRxFIFOFull(int) {}
  void setTxBufferSize(int) {}
  int available() { return simTimeUs >= ready ? int(incoming.size()) : 0; }
  int availableForWrite() { return writeRoom; }
  int read() {
    if (!available())
      return -1;
    int v = incoming.front();
    incoming.pop_front();
    return v;
  }
  size_t write(const uint8_t *p, size_t n);
};
inline FakeSerial Serial;
inline FakeSerial Serial1{true, {}, {}, 0, 4096};
inline void injectStopFrame() {
  uint8_t p[9] = {0xa5, 0x5a, 2, 5, 1, 0, 0, 0, 0};
  const uint16_t c = simCrc16(p + 2, 5);
  p[7] = c;
  p[8] = c >> 8;
  for (auto b : p)
    Serial.incoming.push_back(b);
}
inline size_t FakeSerial::write(const uint8_t *p, size_t n) {
  if (!motor) {
    outgoing.insert(outgoing.end(), p, p + n);
    return n;
  }
  if (n != 10 || p[0] < 1 || p[0] > 4)
    return n;
  auto &m = simMotor[p[0] - 1];
  if (p[1] == 0xa0) {
    if (!m.ignoreMode)
      m.mode = p[9];
    return n;
  }
  if (p[1] == 0x74) {
    m.queries++;
    if (m.mode == 1)
      allCurrentModesQueried |= 1U << (p[0] - 1);
  } else {
    m.commands++;
    if (m.mode == 3)
      unsafePositionCommands++;
    if (m.mode == 1) {
      const int16_t raw = int16_t((uint16_t(p[2]) << 8) | p[3]);
      m.currentMa = raw * 8000.0f / 32767.0f;
    }
  }
  if (injectStopAfterWheel == int(p[0]) && p[1] == 0x64) {
    injectStopAfterWheel = 0;
    injectStopFrame();
  }
  if (m.absent)
    return n;
  uint8_t reply[10] = {p[0], m.mode, 0, 0, 0, 0, 0, 0, m.error, 0};
  const int16_t ma = int16_t(
                    (m.feedbackCurrent ? m.feedbackCurrent : m.currentMa) *
                    32767.0f / 8000.0f),
                rpm = int16_t(std::lround(m.rpm));
  reply[2] = uint16_t(ma) >> 8;
  reply[3] = uint8_t(ma);
  reply[4] = uint16_t(rpm) >> 8;
  reply[5] = uint8_t(rpm);
  const uint16_t pos = uint16_t(m.position);
  if (p[1] == 0x74) {
    reply[6] = uint8_t(m.temp);
    reply[7] = uint8_t(pos * 256 / uint32_t(simCountsPerRev));
  } else {
    reply[6] = pos >> 8;
    reply[7] = uint8_t(pos);
  }
  reply[9] = simCrc8(reply, 9);
  if (m.corrupt)
    reply[9] ^= 0x40;
  for (auto b : reply)
    incoming.push_back(b);
  ready = simTimeUs + 1000;
  return n;
}

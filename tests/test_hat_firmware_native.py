"""Compile the actual HAT sketch against a tiny serial stub and check control safety."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]

ARDUINO_STUB = r"""
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
using std::min;
using std::max;
constexpr int SERIAL_8N1 = 0;
inline uint32_t fakeMillis = 0;
inline uint32_t millis() { return fakeMillis; }
inline uint32_t micros() { return fakeMillis * 1000; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
inline uint8_t fakeCrc8(const uint8_t *data, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? static_cast<uint8_t>((crc >> 1) ^ 0x8C)
                      : static_cast<uint8_t>(crc >> 1);
  }
  return crc;
}
struct FakeSerial {
  bool motor = false;
  uint8_t reply[10] = {};
  int unread = 0;
  void begin(unsigned long) {}
  void begin(unsigned long, int, int, int) {}
  void setRxFIFOFull(int) {}
  int available() { return unread; }
  int read() {
    if (!unread) return -1;
    return reply[10 - unread--];
  }
  size_t write(const uint8_t *request, size_t n) {
    if (motor && n == 10 && request[1] != 0xA0) {
      std::fill(reply, reply + 10, 0);
      reply[0] = request[0];
      reply[1] = 1; // Current mode, stationary, no motor error.
      reply[6] = 25;
      reply[9] = fakeCrc8(reply, 9);
      unread = 10;
    }
    return n;
  }
  void flush() {}
};
inline FakeSerial Serial;
inline FakeSerial Serial1{true};
"""

NATIVE_CHECK = r"""
#include <cassert>
#include <cmath>
#include "hat_firmware/robot_hat/robot_hat.ino"
static void near(float actual, float expected) {
  assert(std::fabs(actual - expected) < 0.001f);
}
int main() {
  cfg.accelRpmS = 120;
  cfg.decelRpmS = 180;
  near(advanceRampRpm(0, 40, 0.020f), 2.4f);
  near(advanceRampRpm(40, 0, 0.020f), 36.4f);
  near(advanceRampRpm(40, -40, 0.100f), 22.0f);
  near(advanceRampRpm(1, -40, 0.015f), -1.133333f);
  near(advanceRampRpm(-1, 40, 0.015f), 1.133333f);
  near(advanceRampRpm(-40, 40, 0.100f), -22.0f);
  wheel[0].rpm = 0;
  wheel[0].rampRpm = 8.0f;
  wheel[0].integralRpmS = 1.0f;
  targetRpm[0] = 0;
  assert(calculateCurrent(0, 0.015f) == 0);
  near(wheel[0].rampRpm, 0);
  near(wheel[0].integralRpmS, 0);

  // Temperature readings every 500 ms must not erase a continuous stall.
  cfg.stallMs = 1000;
  state = ARMED;
  faultCode = NO_FAULT;
  stopPending = false;
  haveTargets = true;
  stagedTargetRpm[0] = 20;
  stagedCurrentCapMa = 1000;
  wheel[0].rampRpm = 20;
  wheel[0].lastInfoMs = 0;
  for (uint32_t time : {600U, 1100U, 1600U}) {
    fakeMillis = time;
    lastTargetMs = time;
    previousSweepUs = time * 1000 - 15000;
    for (uint8_t i = 1; i < WHEEL_COUNT; ++i) wheel[i].lastInfoMs = time;
    controlSweep();
    if (time == 600) {
      assert(wheel[0].lastCommandedMa >= 250);
      assert(wheel[0].stallSinceMs == 600);
    }
    if (time == 1100) assert(wheel[0].stallSinceMs == 600);
  }
  assert(faultCode == STALL);
  assert(stopPending);
}
"""


class HatFirmwareNativeTests(unittest.TestCase):
    def test_ramp_reversal_and_stall_survive_temperature_queries(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if compiler is None:
            self.skipTest("No native C++ compiler")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "Arduino.h").write_text(ARDUINO_STUB)
            (path / "check.cpp").write_text(NATIVE_CHECK)
            executable = path / "check"
            subprocess.run(
                [compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I", str(path),
                 "-I", str(ROOT), str(path / "check.cpp"), "-o", str(executable)],
                check=True, capture_output=True, text=True,
            )
            subprocess.run([str(executable)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()

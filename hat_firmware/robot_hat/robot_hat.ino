// Four-wheel DDSM115 controller for Waveshare DDSM Driver HAT (A).
// Binary Pi protocol: ../../HAT_PROTOCOL.md. This replaces factory firmware.
// GPIO18/19 and Serial1 match Waveshare's ddsm_example for this HAT.
// Serial (UART0) connects to the Raspberry Pi header; it is binary only.
#include <Arduino.h>
#include <math.h>
#include <string.h>

static constexpr uint32_t PI_BAUD = 230400;
static constexpr uint32_t MOTOR_BAUD = 115200;
static constexpr uint8_t MOTOR_RX_PIN = 18;
static constexpr uint8_t MOTOR_TX_PIN = 19;
static constexpr uint8_t WHEEL_COUNT = 4;
static constexpr uint8_t HOST_MAX_PAYLOAD = 64;
static constexpr uint8_t HOST_MAX_FRAME = 9 + HOST_MAX_PAYLOAD;
static constexpr uint8_t MOTOR_FRAME_SIZE = 10;
static constexpr uint32_t MOTOR_REPLY_TIMEOUT_US = 8000;
static constexpr uint32_t FRESH_FEEDBACK_MS = 250;
static constexpr uint32_t FRESH_TEMPERATURE_MS = 1500;
static constexpr uint16_t HARD_MAX_RPM = 200;
static constexpr uint16_t HARD_MAX_CURRENT_MA = 1200;
static constexpr uint8_t HARD_MAX_TEMP_C = 70;
static constexpr uint16_t STOP_VERIFY_MS = 1500;
static constexpr uint8_t STATIONARY_RPM = 2;

enum : uint8_t { HELLO = 1, CONFIG = 2, ARM = 3, TARGETS = 4,
                 STOP = 5, STATUS_REQ = 6, STATUS_FRAME = 0x80 };
enum DriveState : uint8_t { BOOT_STOPPING = 0, DISARMED = 1, ARMED = 2, FAULT = 3 };
enum FaultCode : uint8_t { NO_FAULT = 0, COMMAND_TIMEOUT = 1, MOTOR_TIMEOUT = 2,
                         BAD_MOTOR_FRAME = 3, MOTOR_FAULT = 4, OVERSPEED = 5,
                         OVERTEMPERATURE = 6, STALL = 7, ABNORMAL_CURRENT = 8,
                         CONFIGURATION_FAULT = 9 };
enum MotorResult : uint8_t { MOTOR_OK, MOTOR_NO_REPLY, MOTOR_BAD_REPLY,
                             MOTOR_WRONG_MODE, MOTOR_INTERRUPTED };

struct ControllerConfig {
  uint16_t maxRpm = 40;
  uint16_t maxCurrentMa = 1000;
  uint16_t neutralBrakeMa = 300;
  uint16_t accelRpmS = 120;
  uint16_t decelRpmS = 180;
  uint16_t kpMaPerRpm = 20;
  uint16_t kiMaPerRpmS = 20;
  uint16_t ffMaPerRpmS = 0;
  uint16_t watchdogMs = 250;
  uint16_t periodMs = 15;
  uint16_t stallMs = 700;
  uint8_t tempLimitC = 65;
};

struct Wheel {
  int16_t rpm = 0;
  int16_t currentMa = 0;
  int16_t lastCommandedMa = 0;
  int8_t tempC = 127;
  uint8_t error = 0;
  uint8_t mode = 0;
  uint32_t lastFeedbackMs = 0;
  uint32_t lastInfoMs = 0;
  uint32_t stallSinceMs = 0;
  uint32_t currentSinceMs = 0;
  uint32_t reverseSinceMs = 0;
  float rampRpm = 0.0f;
  float integralRpmS = 0.0f;
  bool valid = false;
};

static ControllerConfig cfg;
static Wheel wheel[WHEEL_COUNT];
static DriveState state = BOOT_STOPPING;
static FaultCode faultCode = NO_FAULT;
static bool configured = false;
static bool stopPending = false;
static bool armPending = false;
static bool stopping = false;
static bool stopVerificationPending = false;
static bool haveCommandSequence = false;
static bool haveTargets = false;
static uint16_t lastCommandSequence = 0;
static uint16_t lastAcceptedSequence = 0;
static uint16_t armSequence = 0;
static uint16_t statusSequence = 0;
static uint16_t commandCurrentCapMa = 0;
static uint16_t stagedCurrentCapMa = 0;
static uint16_t lastSweepUs = 0;
static int16_t targetRpm[WHEEL_COUNT] = {0, 0, 0, 0};
static int16_t stagedTargetRpm[WHEEL_COUNT] = {0, 0, 0, 0};
static uint32_t lastTargetMs = 0;
static uint32_t stopVerifyStartedMs = 0;
static uint32_t nextSweepUs = 0;
static uint32_t previousSweepUs = 0;
static uint32_t nextDisarmedPollMs = 0;
static uint32_t lastStatusMs = 0;
static uint8_t infoCursor = 0;
static uint8_t hostRx[HOST_MAX_FRAME + 16];
static uint8_t hostRxLength = 0;

static uint16_t u16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
static int16_t s16(const uint8_t *p) { return static_cast<int16_t>(u16(p)); }
static void put16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
static uint16_t saturate16(uint32_t v) { return v > 65535UL ? 65535U : static_cast<uint16_t>(v); }
static uint8_t saturate8(uint32_t v) { return v > 255UL ? 255U : static_cast<uint8_t>(v); }
static int16_t clampS16(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return static_cast<int16_t>(v);
}
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static uint16_t crc16(const uint8_t *data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}
static uint8_t crc8Maxim(const uint8_t *data, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? static_cast<uint8_t>((crc >> 1) ^ 0x8C)
                      : static_cast<uint8_t>(crc >> 1);
  }
  return crc;
}

static void sendStatus() {
  uint8_t frame[9 + 36] = {0xA5, 0x5A, 1, STATUS_FRAME};
  put16(frame + 4, statusSequence++);
  frame[6] = 36;
  uint8_t *p = frame + 7;
  put16(p, lastAcceptedSequence); p += 2;
  *p++ = static_cast<uint8_t>(state);
  *p++ = static_cast<uint8_t>(faultCode);
  put16(p, lastSweepUs); p += 2;
  put16(p, haveTargets ? saturate16(millis() - lastTargetMs) : 65535U); p += 2;
  const uint32_t now = millis();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const Wheel &w = wheel[i];
    put16(p, static_cast<uint16_t>(w.rpm)); p += 2;
    put16(p, static_cast<uint16_t>(w.currentMa)); p += 2;
    *p++ = static_cast<uint8_t>(w.tempC);
    *p++ = w.error;
    *p++ = w.valid ? saturate8(now - w.lastFeedbackMs) : 255;
  }
  const uint16_t crc = crc16(frame + 2, 5 + 36);
  put16(frame + 7 + 36, crc);
  Serial.write(frame, sizeof(frame));
  lastStatusMs = now;
}

static bool freshCommand(uint16_t seq) {
  return !haveCommandSequence || static_cast<int16_t>(seq - lastCommandSequence) > 0;
}
static void acceptCommand(uint16_t seq) {
  haveCommandSequence = true;
  lastCommandSequence = seq;
  lastAcceptedSequence = seq;
}
static bool allStationaryFresh() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    const Wheel &w = wheel[i];
    if (!w.valid || now - w.lastFeedbackMs > FRESH_FEEDBACK_MS ||
        now - w.lastInfoMs > FRESH_TEMPERATURE_MS || w.mode != 2 ||
        w.error != 0 || abs(static_cast<int>(w.rpm)) > STATIONARY_RPM ||
        w.tempC == 127 || w.tempC >= cfg.tempLimitC)
      return false;
  }
  return true;
}
static void trip(FaultCode code) {
  if (faultCode == NO_FAULT) faultCode = code;
  state = BOOT_STOPPING;
  stopPending = true;
  armPending = false;
}

static bool validConfig(const ControllerConfig &c) {
  return c.maxRpm >= 1 && c.maxRpm <= HARD_MAX_RPM &&
         c.maxCurrentMa >= 1 && c.maxCurrentMa <= HARD_MAX_CURRENT_MA &&
         c.neutralBrakeMa <= c.maxCurrentMa &&
         c.accelRpmS >= 1 && c.accelRpmS <= 5000 &&
         c.decelRpmS >= 1 && c.decelRpmS <= 5000 &&
         c.kpMaPerRpm <= 1000 && c.kiMaPerRpmS <= 1000 &&
         c.ffMaPerRpmS <= 1000 &&
         c.watchdogMs >= 100 && c.watchdogMs <= 1000 &&
         c.periodMs >= 10 && c.periodMs <= 100 &&
         c.stallMs >= 100 && c.stallMs <= 5000 &&
         c.tempLimitC >= 40 && c.tempLimitC <= HARD_MAX_TEMP_C;
}

static void handleHostFrame(uint8_t type, uint16_t seq, const uint8_t *data, uint8_t n) {
  if (type == HELLO && n == 0) {
    // A reconnect is a new session. Never inherit a previous host's motion.
    haveCommandSequence = false;
    lastCommandSequence = 0;
    lastAcceptedSequence = seq; // HELLO acknowledgement; next command is a new session.
    haveTargets = false;
    armPending = false;
    state = BOOT_STOPPING;
    if (!stopping) stopPending = true;
    return;
  }
  if (type == STOP && n == 0) {
    // STOP is accepted even with an old sequence; safety outranks freshness.
    acceptCommand(seq);
    haveTargets = false;
    armPending = false;
    state = BOOT_STOPPING;
    if (!stopping) stopPending = true;
    return;
  }
  if (type == STATUS_REQ && n == 0) {
    if (freshCommand(seq)) acceptCommand(seq);
    sendStatus();
    return;
  }
  if (stopPending || stopping || !freshCommand(seq)) return;
  if (type == CONFIG && n == 23 && state == DISARMED && allStationaryFresh()) {
    ControllerConfig candidate;
    candidate.maxRpm = u16(data);
    candidate.maxCurrentMa = u16(data + 2);
    candidate.neutralBrakeMa = u16(data + 4);
    candidate.accelRpmS = u16(data + 6);
    candidate.decelRpmS = u16(data + 8);
    candidate.kpMaPerRpm = u16(data + 10);
    candidate.kiMaPerRpmS = u16(data + 12);
    candidate.ffMaPerRpmS = u16(data + 14);
    candidate.watchdogMs = u16(data + 16);
    candidate.periodMs = u16(data + 18);
    candidate.stallMs = u16(data + 20);
    candidate.tempLimitC = data[22];
    if (!validConfig(candidate)) { trip(CONFIGURATION_FAULT); return; }
    cfg = candidate;
    configured = true;
    acceptCommand(seq);
    sendStatus();
    return;
  }
  if (type == ARM && n == 0 && state == DISARMED && configured &&
      !stopVerificationPending && faultCode == NO_FAULT && allStationaryFresh()) {
    armSequence = seq;
    armPending = true;
    return;
  }
  if (type == TARGETS && n == 10 && state == ARMED) {
    int16_t requested[WHEEL_COUNT];
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
      requested[i] = s16(data + 2 * i);
      if (abs(static_cast<int>(requested[i])) > cfg.maxRpm) {
        trip(CONFIGURATION_FAULT); return;
      }
    }
    const uint16_t cap = u16(data + 8);
    if (cap > cfg.maxCurrentMa || cap > HARD_MAX_CURRENT_MA) {
      trip(CONFIGURATION_FAULT); return;
    }
    // Apply all four targets together at the start of the next sweep. Host
    // packets may arrive while the current sweep is between two motors.
    memcpy(stagedTargetRpm, requested, sizeof(stagedTargetRpm));
    stagedCurrentCapMa = cap;
    lastTargetMs = millis();
    haveTargets = true;
    acceptCommand(seq);
  }
}

static void pollHost() {
  // Read a bounded batch so a noisy host cannot starve motor transactions.
  for (uint16_t readCount = 0; readCount < 256 && Serial.available(); ++readCount) {
    const int value = Serial.read();
    if (value < 0) break;
    if (hostRxLength == sizeof(hostRx)) {
      memmove(hostRx, hostRx + 1, --hostRxLength);
    }
    hostRx[hostRxLength++] = static_cast<uint8_t>(value);
    while (hostRxLength >= 2) {
      if (hostRx[0] != 0xA5 || hostRx[1] != 0x5A) {
        memmove(hostRx, hostRx + 1, --hostRxLength);
        continue;
      }
      if (hostRxLength < 7) break;
      const uint8_t payloadLength = hostRx[6];
      if (hostRx[2] != 1 || payloadLength > HOST_MAX_PAYLOAD) {
        memmove(hostRx, hostRx + 1, --hostRxLength);
        continue;
      }
      const uint8_t frameLength = 9 + payloadLength;
      if (hostRxLength < frameLength) break;
      if (crc16(hostRx + 2, 5 + payloadLength) != u16(hostRx + 7 + payloadLength)) {
        memmove(hostRx, hostRx + 1, --hostRxLength);
        continue;
      }
      // Copy payload before shifting the receive buffer.
      uint8_t payload[HOST_MAX_PAYLOAD];
      if (payloadLength) memcpy(payload, hostRx + 7, payloadLength);
      const uint8_t type = hostRx[3];
      const uint16_t seq = u16(hostRx + 4);
      hostRxLength -= frameLength;
      memmove(hostRx, hostRx + frameLength, hostRxLength);
      handleHostFrame(type, seq, payload, payloadLength);
    }
  }
}

static void drainMotorInput() {
  while (Serial1.available()) Serial1.read();
}
static void sendMotorMode(uint8_t id, uint8_t mode) {
  const uint8_t packet[MOTOR_FRAME_SIZE] = {id, 0xA0, 0, 0, 0, 0, 0, 0, 0, mode};
  drainMotorInput();
  Serial1.write(packet, sizeof(packet));
  Serial1.flush();
  delay(4); // Waveshare's factory firmware spaces mode commands by 4 ms.
}
static MotorResult motorTransaction(uint8_t id, bool info, int16_t command,
                                    uint8_t expectedMode, bool allowAbort) {
  uint8_t packet[MOTOR_FRAME_SIZE] = {id, static_cast<uint8_t>(info ? 0x74 : 0x64),
                                     0, 0, 0, 0, 0, 0, 0, 0};
  if (!info) {
    packet[2] = static_cast<uint8_t>(command >> 8);
    packet[3] = static_cast<uint8_t>(command);
  }
  packet[9] = crc8Maxim(packet, 9);
  drainMotorInput(); // Do not allow an old reply to satisfy a new request.
  Serial1.write(packet, sizeof(packet));
  Serial1.flush();
  uint8_t reply[MOTOR_FRAME_SIZE];
  uint8_t received = 0;
  bool sawBadFrame = false;
  const uint32_t started = micros();
  while (static_cast<uint32_t>(micros() - started) < MOTOR_REPLY_TIMEOUT_US) {
    pollHost();
    if (allowAbort && stopPending) return MOTOR_INTERRUPTED;
    if (state == ARMED && haveTargets && millis() - lastTargetMs >= cfg.watchdogMs) {
      trip(COMMAND_TIMEOUT);
      return MOTOR_INTERRUPTED;
    }
    while (Serial1.available()) {
      const int b = Serial1.read();
      if (b < 0) break;
      if (received < MOTOR_FRAME_SIZE) reply[received++] = static_cast<uint8_t>(b);
      if (received != MOTOR_FRAME_SIZE) continue;
      if (crc8Maxim(reply, 9) == reply[9] && reply[0] == id &&
          (reply[1] == 1 || reply[1] == 2 || reply[1] == 3)) {
        Wheel &w = wheel[id - 1];
        w.mode = reply[1];
        // DDSM motor fields are big endian; the host protocol is little endian.
        const int16_t beCurrent = static_cast<int16_t>((reply[2] << 8) | reply[3]);
        const int16_t beRpm = static_cast<int16_t>((reply[4] << 8) | reply[5]);
        w.currentMa = clampS16(static_cast<int32_t>(beCurrent) * 8000 / 32767, -8000, 8000);
        w.rpm = beRpm;
        w.error = reply[8];
        w.lastFeedbackMs = millis();
        w.valid = true;
        if (info) {
          // The host reserves 127 for unavailable; any larger raw reading is
          // over the configured temperature limit and is reported as 126.
          w.tempC = static_cast<int8_t>(min(static_cast<int>(reply[6]), 126));
          w.lastInfoMs = w.lastFeedbackMs;
        }
        if (expectedMode != 0 && w.mode != expectedMode) return MOTOR_WRONG_MODE;
        return MOTOR_OK;
      }
      sawBadFrame = true;
      memmove(reply, reply + 1, MOTOR_FRAME_SIZE - 1);
      received = MOTOR_FRAME_SIZE - 1;
    }
    delayMicroseconds(50);
  }
  return sawBadFrame ? MOTOR_BAD_REPLY : MOTOR_NO_REPLY;
}
static MotorResult sendCurrentMa(uint8_t id, int16_t currentMa,
                                 uint8_t expectedMode, bool allowAbort) {
  // DDSM115 protocol maps signed 32767 to 8 A; the compiled limit is 1.2 A.
  const int16_t bounded = clampS16(currentMa, -HARD_MAX_CURRENT_MA, HARD_MAX_CURRENT_MA);
  const int16_t raw = clampS16(static_cast<int32_t>(bounded) * 32767 / 8000, -32767, 32767);
  return motorTransaction(id, false, raw, expectedMode, allowAbort);
}
static FaultCode motorFailure(MotorResult result) {
  if (result == MOTOR_NO_REPLY) return MOTOR_TIMEOUT;
  if (result == MOTOR_BAD_REPLY) return BAD_MOTOR_FRAME;
  return MOTOR_FAULT;
}
static void performStop() {
  stopping = true;
  stopPending = false;
  state = BOOT_STOPPING;
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    targetRpm[i] = 0;
    stagedTargetRpm[i] = 0;
    wheel[i].rampRpm = 0;
    wheel[i].integralRpmS = 0;
    wheel[i].lastCommandedMa = 0;
    wheel[i].stallSinceMs = 0;
    wheel[i].currentSinceMs = 0;
    wheel[i].reverseSinceMs = 0;
  }
  commandCurrentCapMa = 0;
  stagedCurrentCapMa = 0;
  haveTargets = false;
  uint8_t observedMode[WHEEL_COUNT] = {0, 0, 0, 0};
  bool speedVerified[WHEEL_COUNT] = {false, false, false, false};
  // First query every mode without commanding motion. A 0x64 zero command
  // means a position move to zero in position mode.
  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) {
    const MotorResult r = motorTransaction(id, true, 0, 0, false);
    if (r == MOTOR_OK) observedMode[id - 1] = wheel[id - 1].mode;
    if (r != MOTOR_OK && faultCode == NO_FAULT) faultCode = motorFailure(r);
  }
  // Stop all wheels confirmed to be in current or speed mode before spending
  // time verifying any individual mode switch.
  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) {
    const uint8_t mode = observedMode[id - 1];
    if (mode != 1 && mode != 2) continue;
    const MotorResult r = motorTransaction(id, false, 0, mode, false);
    if (r != MOTOR_OK && faultCode == NO_FAULT) faultCode = motorFailure(r);
  }
  // A mode-write packet has no acknowledgement. Switch every motor, then
  // verify each one with a safe information query before sending speed zero.
  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) sendMotorMode(id, 2);
  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) {
    MotorResult modeResult = motorTransaction(id, true, 0, 2, false);
    for (uint8_t retry = 0; retry < 2 && modeResult != MOTOR_OK; ++retry) {
      sendMotorMode(id, 2);
      modeResult = motorTransaction(id, true, 0, 2, false);
    }
    speedVerified[id - 1] = modeResult == MOTOR_OK;
    if (modeResult != MOTOR_OK && faultCode == NO_FAULT)
      faultCode = motorFailure(modeResult);
  }

  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) {
    if (!speedVerified[id - 1]) continue; // Never risk a position command.
    const MotorResult r = motorTransaction(id, false, 0, 2, false);
    if (r != MOTOR_OK && faultCode == NO_FAULT) faultCode = motorFailure(r);
  }
  stopVerificationPending = true;
  stopVerifyStartedMs = millis();
  nextDisarmedPollMs = 0;
  // Stay in BOOT_STOPPING until fresh information from all four motors proves
  // that they are in speed mode and stationary. The host must not be able to
  // send CONFIG or ARM against stale pre-stop feedback.
  state = faultCode == NO_FAULT ? BOOT_STOPPING : FAULT;
  stopping = false;
  sendStatus();
}

static void processArm() {
  armPending = false;
  if (state != DISARMED || faultCode != NO_FAULT || !allStationaryFresh()) return;
  state = BOOT_STOPPING;
  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) {
    if (stopPending) return;
    sendMotorMode(id, 1);
  }
  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) {
    if (stopPending) return;
    const MotorResult r = sendCurrentMa(id, 0, 1, true);
    if (r == MOTOR_INTERRUPTED) return;
    if (r != MOTOR_OK) { trip(motorFailure(r)); return; }
    if (wheel[id - 1].error != 0 || abs(static_cast<int>(wheel[id - 1].rpm)) > STATIONARY_RPM) {
      trip(MOTOR_FAULT); return;
    }
  }
  acceptCommand(armSequence);
  state = ARMED;
  lastTargetMs = millis();
  haveTargets = true; // ARM starts a watchdog, even before first TARGETS.
  commandCurrentCapMa = 0;
  previousSweepUs = micros();
  nextSweepUs = previousSweepUs;
  sendStatus();
}

static void checkWheelHealth(uint8_t i, int16_t commandedMa) {
  Wheel &w = wheel[i];
  const uint32_t now = millis();
  if (w.error) { trip(MOTOR_FAULT); return; }
  if (w.tempC != 127 && w.tempC >= cfg.tempLimitC) { trip(OVERTEMPERATURE); return; }
  if (abs(static_cast<int>(w.rpm)) > cfg.maxRpm + max(10, cfg.maxRpm / 4)) {
    trip(OVERSPEED); return;
  }
  const bool stall = abs(static_cast<int>(targetRpm[i])) >= 5 &&
                     abs(static_cast<int>(commandedMa)) >= 250 &&
                     abs(static_cast<int>(w.rpm)) <= STATIONARY_RPM;
  if (stall) {
    if (!w.stallSinceMs) w.stallSinceMs = now;
    if (now - w.stallSinceMs >= cfg.stallMs) { trip(STALL); return; }
  } else w.stallSinceMs = 0;
  // A new reverse target may take several sweeps to ramp through zero. Check
  // against the speed we are actually commanding, and allow the configured
  // stall interval for the wheel to change direction after that crossing.
  const bool reverse = abs(static_cast<int>(w.rpm)) >= 5 &&
                       fabsf(w.rampRpm) >= 5.0f &&
                       (static_cast<float>(w.rpm) * w.rampRpm < 0.0f);
  if (reverse) {
    if (!w.reverseSinceMs) w.reverseSinceMs = now;
    if (now - w.reverseSinceMs >= cfg.stallMs) { trip(STALL); return; }
  } else w.reverseSinceMs = 0;
  const int currentLimit = min(2000, static_cast<int>(cfg.maxCurrentMa) + 600);
  if (abs(static_cast<int>(w.currentMa)) > currentLimit) {
    if (!w.currentSinceMs) w.currentSinceMs = now;
    if (now - w.currentSinceMs >= 200) trip(ABNORMAL_CURRENT);
  } else w.currentSinceMs = 0;
}

static float advanceRampRpm(float prior, float desired, float dt) {
  // A reversal first brakes to zero at the deceleration rate, then accelerates
  // in the other direction. Preserve any unused part of this sweep's dt so a
  // short step across zero does not introduce a one-sweep pause or accelerate
  // at the braking rate.
  if (prior * desired < 0.0f) {
    const float timeToZero = fabsf(prior) / cfg.decelRpmS;
    if (dt <= timeToZero) {
      const float remaining = max(0.0f, fabsf(prior) - cfg.decelRpmS * dt);
      return prior > 0.0f ? remaining : -remaining;
    }
    const float awayStep = cfg.accelRpmS * (dt - timeToZero);
    return desired > 0.0f ? min(desired, awayStep) : max(desired, -awayStep);
  }
  const float rate = fabsf(desired) < fabsf(prior) ? cfg.decelRpmS : cfg.accelRpmS;
  const float step = rate * dt;
  return desired > prior ? min(desired, prior + step) : max(desired, prior - step);
}

static int16_t calculateCurrent(uint8_t i, float dt) {
  Wheel &w = wheel[i];
  const float desired = targetRpm[i];
  if (desired == 0.0f && abs(static_cast<int>(w.rpm)) <= STATIONARY_RPM) {
    // A stationary wheel must not restart to chase a decaying speed ramp.
    w.rampRpm = 0;
    w.integralRpmS = 0;
    return 0;
  }
  const float prior = w.rampRpm;
  w.rampRpm = advanceRampRpm(prior, desired, dt);
  const float error = w.rampRpm - w.rpm;
  const float accel = (w.rampRpm - prior) / dt;
  const float pff = cfg.kpMaPerRpm * error + cfg.ffMaPerRpmS * accel;
  // A zero driver dial must still allow the separately configured neutral
  // braking current while a wheel is moving. The stationary guard above
  // prevents that allowance from restarting a stopped wheel.
  float cap = desired == 0.0f
                  ? min(static_cast<int>(cfg.neutralBrakeMa), static_cast<int>(cfg.maxCurrentMa))
                  : min(static_cast<int>(commandCurrentCapMa), static_cast<int>(cfg.maxCurrentMa));
  cap = min(cap, static_cast<float>(HARD_MAX_CURRENT_MA));
  if (cap <= 0.0f) { w.integralRpmS = 0; return 0; }
  const float proposed = w.integralRpmS + error * dt;
  const float proposedOutput = pff + cfg.kiMaPerRpmS * proposed;
  if (!((proposedOutput > cap && error > 0.0f) ||
        (proposedOutput < -cap && error < 0.0f)))
    w.integralRpmS = proposed;
  if (cfg.kiMaPerRpmS > 0)
    w.integralRpmS = clampf(w.integralRpmS, -cap / cfg.kiMaPerRpmS,
                           cap / cfg.kiMaPerRpmS);
  else w.integralRpmS = 0;
  const float output = clampf(pff + cfg.kiMaPerRpmS * w.integralRpmS, -cap, cap);
  return static_cast<int16_t>(lroundf(output));
}

static void controlSweep() {
  const uint32_t sweepStarted = micros();
  memcpy(targetRpm, stagedTargetRpm, sizeof(targetRpm));
  commandCurrentCapMa = stagedCurrentCapMa;
  float dt = static_cast<uint32_t>(sweepStarted - previousSweepUs) / 1000000.0f;
  // Use real elapsed time, including an overrun. Cap a long scheduling stall
  // so the speed setpoint cannot jump sharply when control resumes.
  dt = clampf(dt, 0.001f, 0.1f);
  previousSweepUs = sweepStarted;
  if (millis() - lastTargetMs >= cfg.watchdogMs) { trip(COMMAND_TIMEOUT); return; }
  for (uint8_t i = 0; i < WHEEL_COUNT; ++i) {
    pollHost();
    if (stopPending) return;
    const int16_t current = calculateCurrent(i, dt);
    const MotorResult r = sendCurrentMa(i + 1, current, 1, true);
    if (r == MOTOR_INTERRUPTED) return;
    if (r != MOTOR_OK) { trip(motorFailure(r)); return; }
    wheel[i].lastCommandedMa = current;
    checkWheelHealth(i, current);
    if (stopPending) return;
  }
  // Each motor gets a temperature query about twice per second. It is an
  // additional transaction, never a replacement for speed/current feedback.
  for (uint8_t offset = 0; offset < WHEEL_COUNT; ++offset) {
    const uint8_t i = (infoCursor + offset) % WHEEL_COUNT;
    if (millis() - wheel[i].lastInfoMs < 500) continue;
    const MotorResult r = motorTransaction(i + 1, true, 0, 1, true);
    if (r == MOTOR_INTERRUPTED) return;
    if (r != MOTOR_OK) { trip(motorFailure(r)); return; }
    // An information query does not change the current command. Use the last
    // sent current so a periodic temperature reading cannot reset a stall
    // timer that is still active.
    checkWheelHealth(i, wheel[i].lastCommandedMa);
    infoCursor = (i + 1) % WHEEL_COUNT;
    break;
  }
  lastSweepUs = saturate16(micros() - sweepStarted);
  sendStatus();
}

static void pollDisarmed() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextDisarmedPollMs) < 0) return;
  nextDisarmedPollMs = now + 100;
  for (uint8_t id = 1; id <= WHEEL_COUNT; ++id) {
    const MotorResult r = motorTransaction(id, true, 0, 2, true);
    if (r == MOTOR_INTERRUPTED) return;
    if (r != MOTOR_OK) { trip(motorFailure(r)); return; }
    if (wheel[id - 1].error != 0) { trip(MOTOR_FAULT); return; }
    if (wheel[id - 1].tempC >= cfg.tempLimitC) { trip(OVERTEMPERATURE); return; }
  }
  if (stopVerificationPending) {
    if (allStationaryFresh()) {
      stopVerificationPending = false;
      state = DISARMED;
    }
    else if (millis() - stopVerifyStartedMs >= STOP_VERIFY_MS) {
      trip(MOTOR_FAULT); return; // zero-speed commands failed to stop a wheel
    }
  }
  sendStatus();
}

void setup() {
  Serial.begin(PI_BAUD);
  Serial1.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  // The ESP32 core otherwise raises RX FIFO threshold at these baud rates;
  // expose short command and motor-reply frames promptly to the controller.
  Serial.setRxFIFOFull(1);
  Serial1.setRxFIFOFull(1);
  delay(20);
  while (Serial.available()) Serial.read();
  drainMotorInput();
  hostRxLength = 0;
  performStop(); // Never depend on the motor's power-on control mode.
}

void loop() {
  pollHost();
  if (stopPending) { performStop(); return; }
  if (armPending) { processArm(); return; }
  if (state == ARMED) {
    if (millis() - lastTargetMs >= cfg.watchdogMs) { trip(COMMAND_TIMEOUT); return; }
    if (static_cast<int32_t>(micros() - nextSweepUs) >= 0) {
      const uint32_t sweepStartUs = micros();
      controlSweep();
      // Schedule start-to-start. If a sweep takes longer than one period,
      // begin the next as soon as possible; never queue catch-up sweeps.
      nextSweepUs = sweepStartUs + static_cast<uint32_t>(cfg.periodMs) * 1000UL;
    }
  } else if (state == DISARMED || (state == BOOT_STOPPING && stopVerificationPending)) {
    pollDisarmed();
  } else if (millis() - lastStatusMs >= 100) {
    sendStatus();
  }
}

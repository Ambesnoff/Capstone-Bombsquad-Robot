#pragma once
#include "protocol_v2.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

static constexpr uint32_t PI_BAUD = 230400, MOTOR_BAUD = 115200;
static constexpr uint8_t MOTOR_RX_PIN = 18, MOTOR_TX_PIN = 19, WHEEL_COUNT = 4;
static constexpr uint8_t HOST_MAX_PAYLOAD = 240, MOTOR_FRAME_SIZE = 10;
static constexpr uint16_t HOST_MAX_FRAME = 9 + HOST_MAX_PAYLOAD;
static constexpr uint32_t MOTOR_REPLY_TIMEOUT_US = 8000;
// A missed or garbled motor reply is retried at once, MOTOR_ATTEMPTS in all.
// REPLY_RETRY stays reported for REPLY_RETRY_MS after a wheel's last failure.
static constexpr uint8_t MOTOR_ATTEMPTS = 3;
static constexpr uint16_t REPLY_RETRY_MS = 1000;
static constexpr uint16_t HARD_MAX_RPM = ProtocolV2::FIRMWARE_MAX_RPM,
                          HARD_MAX_CURRENT_MA = 2700;
static constexpr uint8_t HARD_MAX_TEMP_C = 70, STATIONARY_RPM = 2;
// Whole-chassis neutral enters position holding once every wheel stays at or
// below this speed for the settle dwell; a slope keeps a coasting wheel just
// above STATIONARY_RPM, so requiring true rest there can never complete.
static constexpr uint8_t SETTLE_ENTRY_RPM = 6;
enum DriveState : uint8_t {
  BOOT_STOPPING = 0,
  DISARMED = 1,
  ARMED = 2,
  FAULT = 3,
  SETTLING = 4,
  HOLDING = 5,
  STOPPING = 6
};
enum FaultCode : uint8_t {
  NO_FAULT = 0,
  COMMAND_TIMEOUT = 1,
  MOTOR_TIMEOUT = 2,
  BAD_MOTOR_FRAME = 3,
  MOTOR_FAULT = 4,
  OVERSPEED = 5,
  OVERTEMPERATURE = 6,
  STALL = 7,
  ABNORMAL_CURRENT = 8,
  CONFIGURATION_FAULT = 9,
  TEMPERATURE_STALE = 10,
  CONTROL_PROGRESS = 11
};
enum MotorResult : uint8_t {
  MOTOR_OK,
  MOTOR_NO_REPLY,
  MOTOR_BAD_REPLY,
  MOTOR_WRONG_MODE,
  MOTOR_INTERRUPTED
};
enum : uint8_t {
  HELLO = 1,
  CONFIG = 2,
  ARM = 3,
  TARGETS = 4,
  STOP = 5,
  STATUS_REQ = 6,
  STATUS_FRAME = 0x80
};
enum : uint32_t {
  R_CEILING = 1,
  R_BOOST_EMPTY = 2,
  R_TEMP_STALE = 4,
  R_TEMP_WARN = 8,
  R_DERATE = 16,
  R_THERMAL_STOP = 32,
  R_FEEDBACK_STALE = 64,
  R_MOTOR_ERROR = 128,
  R_STALL_WARN = 256,
  R_STALL = 512,
  R_CURRENT = 1024,
  R_COMMAND = 2048,
  R_HOLD_LIMITED = 4096,
  R_COOLDOWN = 8192,
  R_CONFIG = 16384,
  R_SESSION = 32768,
  R_SPEED = 65536,
  R_INSPECTION = 131072,
  R_REPLY_RETRY = 262144
};
// The control code names these values directly; they must stay identical to
// the generated wire contract shared with the Pi.
template <typename T> constexpr uint32_t wireValue(T value) {
  return static_cast<uint32_t>(value);
}
static_assert(BOOT_STOPPING == wireValue(ProtocolV2::HatState::BOOT_STOPPING) &&
                  DISARMED == wireValue(ProtocolV2::HatState::DISARMED) &&
                  ARMED == wireValue(ProtocolV2::HatState::ARMED) &&
                  FAULT == wireValue(ProtocolV2::HatState::FAULT) &&
                  SETTLING == wireValue(ProtocolV2::HatState::SETTLING) &&
                  HOLDING == wireValue(ProtocolV2::HatState::HOLDING) &&
                  STOPPING == wireValue(ProtocolV2::HatState::STOPPING),
              "DriveState must match the generated HatState");
static_assert(
    NO_FAULT == wireValue(ProtocolV2::FaultCode::NONE) &&
        COMMAND_TIMEOUT == wireValue(ProtocolV2::FaultCode::COMMAND_TIMEOUT) &&
        MOTOR_TIMEOUT == wireValue(ProtocolV2::FaultCode::MOTOR_TIMEOUT) &&
        BAD_MOTOR_FRAME == wireValue(ProtocolV2::FaultCode::BAD_MOTOR_FRAME) &&
        MOTOR_FAULT == wireValue(ProtocolV2::FaultCode::MOTOR_FAULT) &&
        OVERSPEED == wireValue(ProtocolV2::FaultCode::OVERSPEED) &&
        OVERTEMPERATURE == wireValue(ProtocolV2::FaultCode::OVERTEMPERATURE) &&
        STALL == wireValue(ProtocolV2::FaultCode::STALL) &&
        ABNORMAL_CURRENT == wireValue(ProtocolV2::FaultCode::ABNORMAL_CURRENT) &&
        CONFIGURATION_FAULT ==
            wireValue(ProtocolV2::FaultCode::CONFIGURATION_FAULT) &&
        TEMPERATURE_STALE == wireValue(ProtocolV2::FaultCode::TEMPERATURE_STALE) &&
        CONTROL_PROGRESS == wireValue(ProtocolV2::FaultCode::CONTROL_PROGRESS),
    "FaultCode must match the generated FaultCode");
static_assert(HELLO == wireValue(ProtocolV2::FrameType::HELLO) &&
                  CONFIG == wireValue(ProtocolV2::FrameType::CONFIG) &&
                  ARM == wireValue(ProtocolV2::FrameType::ARM) &&
                  TARGETS == wireValue(ProtocolV2::FrameType::TARGETS) &&
                  STOP == wireValue(ProtocolV2::FrameType::STOP) &&
                  STATUS_REQ == wireValue(ProtocolV2::FrameType::STATUS_REQ) &&
                  STATUS_FRAME == wireValue(ProtocolV2::FrameType::STATUS),
              "Frame types must match the generated FrameType");
static_assert(
    R_CEILING == wireValue(ProtocolV2::Reason::FIRMWARE_CEILING) &&
        R_BOOST_EMPTY == wireValue(ProtocolV2::Reason::BOOST_EMPTY) &&
        R_TEMP_STALE == wireValue(ProtocolV2::Reason::TEMP_STALE) &&
        R_TEMP_WARN == wireValue(ProtocolV2::Reason::THERMAL_WARNING) &&
        R_DERATE == wireValue(ProtocolV2::Reason::THERMAL_DERATE) &&
        R_THERMAL_STOP == wireValue(ProtocolV2::Reason::THERMAL_STOP) &&
        R_FEEDBACK_STALE == wireValue(ProtocolV2::Reason::FEEDBACK_STALE) &&
        R_MOTOR_ERROR == wireValue(ProtocolV2::Reason::MOTOR_ERROR) &&
        R_STALL_WARN == wireValue(ProtocolV2::Reason::STALL_WARNING) &&
        R_STALL == wireValue(ProtocolV2::Reason::STALL) &&
        R_CURRENT == wireValue(ProtocolV2::Reason::ABNORMAL_CURRENT) &&
        R_COMMAND == wireValue(ProtocolV2::Reason::COMMAND_TIMEOUT) &&
        R_HOLD_LIMITED == wireValue(ProtocolV2::Reason::HOLD_LIMITED) &&
        R_COOLDOWN == wireValue(ProtocolV2::Reason::COOLDOWN) &&
        R_CONFIG == wireValue(ProtocolV2::Reason::CONFIG_REJECTED) &&
        R_SESSION == wireValue(ProtocolV2::Reason::SESSION_MISMATCH) &&
        R_SPEED == wireValue(ProtocolV2::Reason::SPEED_ERROR) &&
        R_INSPECTION == wireValue(ProtocolV2::Reason::INSPECTION_REQUIRED) &&
        R_REPLY_RETRY == wireValue(ProtocolV2::Reason::REPLY_RETRY),
    "Reason bits must match the generated Reason flags");
static_assert(HARD_MAX_CURRENT_MA == ProtocolV2::FIRMWARE_MAX_CURRENT_MA &&
                  HOST_MAX_PAYLOAD == ProtocolV2::MAX_PAYLOAD,
              "Firmware limits must match the generated contract");
struct ControllerConfig {
  uint16_t maxRpm = 40, maxCurrentMa = 2700, neutralBrakeMa = 300,
           accelRpmS = 120, decelRpmS = 180;
  uint16_t kpMaPerRpm = 20, kiMaPerRpmS = 4, ffMaPerRpmS = 0, watchdogMs = 300,
           periodMs = 15, stallMs = 1000;
  uint16_t gentleMa = 800, normalMa = 1500, boostMa = 2500,
           boostCapacityMs = 20000, boostRefillMs = 60000;
  uint16_t tempPollMs = 500, boostFreshMs = 750, tempFreshMs = 1500,
           cooldownMs = 3000, capRampMaS = 1000;
  uint16_t holdMa = 300, holdKp = 2, holdKi = 1, holdDamping = 20,
           settleMs = 300, feedbackMs = 150;
  uint16_t stallTargetCenti = 800, stallSpeedCenti = 200, stallCurrentMa = 250,
           abnormalCurrentMa = 2700;
  uint16_t abnormalMs = 200, abnormalMarginMa = 400, saturationWarnMs = 500,
           stopVerifyMs = 1500;
  uint8_t tempWarnC = 50, tempDerateC = 55, tempLimitC = 65, tempReleaseC = 45,
          tempHysteresisC = 3;
  uint8_t holdEnabled = 1, disarmedHoldEnabled = 0, stallEnabled = 1,
          holdTempC = 55;
  uint32_t encoderCountsPerRev = 32768;
};
struct Wheel {
  int16_t rpm = 0, currentMa = 0, lastCommandedMa = 0;
  int16_t tempC = 127;
  uint16_t positionRaw = 0, lastPositionRaw = 0;
  int64_t positionUnwrapped = 0, holdAnchor = 0;
  uint8_t error = 0, mode = 0;
  // MotorResult of the latest failed attempt; MOTOR_OK once a reply is valid.
  uint8_t lastFailure = MOTOR_OK;
  uint32_t lastFeedbackMs = 0, lastInfoMs = 0, lastPositionMs = 0,
           stallSinceMs = 0, currentSinceMs = 0, reverseSinceMs = 0,
           saturationSinceMs = 0, reason = 0, failedMs = 0;
  float rampRpm = 0, integralRpmS = 0, holdIntegral = 0, effectiveCapMa = 0,
        holdCapMa = 0, protectionCapMa = 0, controlCapMa = 0;
  bool valid = false, positionValid = false, temperatureValid = false,
       thermalWarning = false, thermalDerating = false, holdLimited = false;
  // The last current command got no valid reply: feedback is older than it.
  bool unacked = false;
};
static ControllerConfig cfg;
static Wheel wheel[WHEEL_COUNT];
static DriveState state = BOOT_STOPPING;
static FaultCode faultCode = NO_FAULT;
// The first cause is diagnostic; a later inspection fault still requires reset.
static bool inspectionRequired = false;
static uint8_t faultWheel = 0, stopState = 1, requestedProfile = 0,
               appliedProfile = 0, stagedProfile = 0, configResult = 0;
static bool configured = false, stopPending = false, armPending = false,
            stopping = false, stopVerificationPending = true,
            haveCommandSequence = false, haveTargets = false,
            disarmedHolding = false;
static uint16_t lastCommandSequence = 0, lastAcceptedSequence = 0,
                lastAppliedSequence = 0, armSequence = 0, statusSequence = 0,
                stagedSequence = 0, configAckSequence = 0;
static uint32_t lastAcceptedMs = 0, lastAppliedMs = 0, lastSweepUs = 0,
                configId = 0, lastTargetMs = 0, stopVerifyStartedMs = 0,
                nextSweepUs = 0, previousSweepUs = 0, nextDisarmedPollMs = 0,
                lastStatusMs = 0, settleStartedMs = 0, coolSinceMs = 0,
                protectionPreviousMs = 0, reasonFlags = 0;
static uint64_t bootId = 0, hostSession = 0;
static float boostRemainingMs = 0;
static int16_t targetCentiRpm[WHEEL_COUNT] = {},
               stagedTargetCentiRpm[WHEEL_COUNT] = {};
static uint8_t infoCursor = 0;
static uint8_t hostRx[HOST_MAX_FRAME + 16];
static uint16_t hostRxLength = 0;
static uint16_t u16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
static int16_t s16(const uint8_t *p) { return static_cast<int16_t>(u16(p)); }
static uint32_t u32(const uint8_t *p) {
  return static_cast<uint32_t>(u16(p)) |
         (static_cast<uint32_t>(u16(p + 2)) << 16);
}
static uint64_t u64(const uint8_t *p) {
  return static_cast<uint64_t>(u32(p)) |
         (static_cast<uint64_t>(u32(p + 4)) << 32);
}
static void put16(uint8_t *p, uint16_t v) {
  p[0] = v;
  p[1] = v >> 8;
}
static void put32(uint8_t *p, uint32_t v) {
  put16(p, v);
  put16(p + 2, v >> 16);
}
static void put64(uint8_t *p, uint64_t v) {
  put32(p, v);
  put32(p + 4, v >> 32);
}
static uint16_t saturate16(uint32_t v) {
  return v > 65535 ? 65535 : static_cast<uint16_t>(v);
}
static int16_t clampS16(int32_t v, int32_t lo, int32_t hi) {
  return static_cast<int16_t>(v < lo ? lo : (v > hi ? hi : v));
}
static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static uint16_t crc16(const uint8_t *data, size_t n) {
  uint16_t c = 65535;
  for (size_t i = 0; i < n; i++) {
    c ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x1021)
                       : static_cast<uint16_t>(c << 1);
  }
  return c;
}
static uint8_t crc8Maxim(const uint8_t *data, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 1) ? static_cast<uint8_t>((c >> 1) ^ 0x8C)
                  : static_cast<uint8_t>(c >> 1);
  }
  return c;
}
static bool motionState() {
  return state == ARMED || state == SETTLING ||
         (state == HOLDING && !disarmedHolding);
}
static bool currentState() { return motionState() || disarmedHolding; }
static bool freshCommand(uint16_t seq) {
  return !haveCommandSequence ||
         static_cast<int16_t>(seq - lastCommandSequence) > 0;
}
static void acceptCommand(uint16_t seq) {
  haveCommandSequence = true;
  lastCommandSequence = lastAcceptedSequence = seq;
  lastAcceptedMs = millis();
}
static void markApplied(uint16_t seq) {
  lastAppliedSequence = seq;
  lastAppliedMs = millis();
}
static bool stationaryFresh(bool ready = false) {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    const Wheel &w = wheel[i];
    if (!w.valid || now - w.lastFeedbackMs > cfg.feedbackMs ||
        w.mode != (disarmedHolding ? 1 : 2) ||
        abs(static_cast<int>(w.rpm)) > STATIONARY_RPM)
      return false;
    if (ready && (!w.temperatureValid || now - w.lastInfoMs > cfg.tempFreshMs ||
                  w.error || w.tempC >= cfg.tempLimitC))
      return false;
  }
  return true;
}
static bool recoverableFaultCode(FaultCode code) {
  return code == COMMAND_TIMEOUT || code == MOTOR_TIMEOUT ||
         code == BAD_MOTOR_FRAME || code == OVERTEMPERATURE ||
         code == TEMPERATURE_STALE;
}
static void latchFault(FaultCode code, uint8_t id = 0) {
  if (code != NO_FAULT && !recoverableFaultCode(code))
    inspectionRequired = true;
  if (faultCode == NO_FAULT) {
    faultCode = code;
    faultWheel = id;
  }
}
static void trip(FaultCode code, uint8_t id = 0) {
  latchFault(code, id);
  armPending = false;
  haveTargets = false;
  if (!stopping && state != FAULT) {
    stopPending = true;
    stopState = 1;
  }
  state = FAULT;
}
static void pollHost();
static void sendStatus();
static void completedProgress();

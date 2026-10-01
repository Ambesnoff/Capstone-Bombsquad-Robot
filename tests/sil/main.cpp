#include "hat_firmware/robot_hat/robot_hat.ino"
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
static uint16_t seq = 1;
static void near(float actual, float expected, float tolerance = 0.01f) {
  assert(std::fabs(actual - expected) < tolerance);
}
static void sendTargets(int16_t target = 0, uint8_t profile = 0,
                        bool innerZero = false) {
  ProtocolV2::TargetsPayload p{};
  p.session.boot_id = bootId;
  p.session.host_session = hostSession;
  p.profile = profile;
  for (int i = 0; i < 4; i++)
    p.target_centi_rpm[i] = (innerZero && i == 0) ? 0 : target;
  handleHostFrame(TARGETS, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
}
static void runFor(uint32_t ms, bool renew = false, int16_t target = 0,
                   uint8_t profile = 0, bool innerZero = false) {
  const uint32_t end = millis() + ms;
  uint32_t refresh = millis();
  while (static_cast<int32_t>(millis() - end) < 0) {
    if (renew && static_cast<int32_t>(millis() - refresh) >= 0) {
      sendTargets(target, profile, innerZero);
      refresh = millis() + 50;
    }
    loop();
    advanceUs(1000);
  }
}
static void boot() {
  setup();
  runFor(300);
  assert(state == DISARMED);
  assert(stopState == 3);
  assert(!unsafePositionCommands);
}
static void hello() {
  uint8_t p[8];
  put64(p, 0x1234567890ULL);
  handleHostFrame(HELLO, seq, p, sizeof(p));
  runFor(300);
  assert(state == DISARMED);
  assert(hostSession);
}
static void configure() {
  ProtocolV2::ConfigPayload p{};
  p.session.boot_id = bootId;
  p.session.host_session = hostSession;
  // ControllerConfig follows the checked wire scalar order; copying through
  // fields is replaced by generated defaults when schema adds new fields.
  p.config = ProtocolV2::defaultConfig();
  p.config.disarmed_hold_enabled = cfg.disarmedHoldEnabled;
  p.config_id =
      configCrc32(reinterpret_cast<uint8_t *>(&p.config), sizeof(p.config));
  handleHostFrame(CONFIG, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
  assert(configured);
  assert(configResult == 1);
}
static void arm() {
  ProtocolV2::ArmPayload p{{bootId, hostSession}, configId};
  handleHostFrame(ARM, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
  loop();
  assert(motionState());
  assert(!unsafePositionCommands);
}
static void ready() {
  boot();
  hello();
  configure();
  arm();
}
static void refreshFeedback() {
  for (int i = 0; i < 4; i++) {
    wheel[i].valid = wheel[i].temperatureValid = true;
    wheel[i].lastFeedbackMs = wheel[i].lastInfoMs = millis();
    wheel[i].tempC = 25;
    wheel[i].error = 0;
  }
}
static void assertInspectionLockout(FaultCode firstCause, uint8_t firstWheel) {
  assert(inspectionRequired);
  assert(faultCode == firstCause);
  assert(faultWheel == firstWheel);
  assert(state == FAULT);
  assert(stopState == 3);
  assert(!haveTargets);
  for (const auto &w : wheel)
    assert(w.error == 0);
  const uint16_t acceptedBefore = lastAcceptedSequence;
  ProtocolV2::ArmPayload p{{bootId, hostSession}, configId};
  handleHostFrame(ARM, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
  assert(!armPending);
  loop();
  assert(!motionState());
  assert(lastAcceptedSequence == acceptedBefore);
}
static void execute(const std::string &name) {
  if (name == "ramp") {
    near(advanceRampRpm(0, 40, .02f), 2.4f);
    near(advanceRampRpm(40, 0, .02f), 36.4f);
    near(advanceRampRpm(40, -40, .1f), 22);
    near(advanceRampRpm(1, -40, .015f), -1.133333f);
    near(advanceRampRpm(-1, 40, .015f), 1.133333f);
    return;
  }
  if (name == "physics") {
    simMotor[0].mode = 1;
    simMotor[0].currentMa = 500;
    const float prior = simMotor[0].rpm;
    advanceUs(1000000);
    assert(simMotor[0].rpm > prior + 10);
    assert(simMotor[0].commands == 0);
    return;
  }
  if (name == "boot_modes") {
    boot();
    assert(stopState == 3);
    assert(!unsafePositionCommands);
    return;
  }
  if (name == "arm_mode_failure") {
    boot();
    hello();
    configure();
    simMotor[2].ignoreMode = true;
    ProtocolV2::ArmPayload p{{bootId, hostSession}, configId};
    handleHostFrame(ARM, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
    assert(armPending);
    processArm();
    assert(faultCode == MOTOR_FAULT);
    assert(stopPending);
    assert(!unsafePositionCommands);
    assert(allCurrentModesQueried != 15);
    return;
  }
  if (name == "faulted_stop_polling") {
    ready();
    simMotor[3].absent = true;
    trip(MOTOR_TIMEOUT, 4);
    loop();
    const unsigned before = simMotor[0].queries;
    runFor(2200);
    assert(state == FAULT);
    assert(stopState == 4);
    assert(simMotor[0].queries > before + 10);
    simMotor[3].absent = false;
    runFor(4000);
    assert(stopState == 3);
    assert(state == DISARMED);
    assert(!haveTargets);
    return;
  }
  if (name == "combined_motor_fault") {
    ready();
    simMotor[0].absent = true;
    simMotor[1].error = 1;
    runFor(200);
    assert(faultCode == MOTOR_TIMEOUT);
    assert(faultWheel == 1);
    assert(wheel[1].error == 1);
    simMotor[0].absent = false;
    runFor(500);
    simMotor[1].error = 0;
    runFor(4000);
    assertInspectionLockout(MOTOR_TIMEOUT, 1);
    return;
  }
  if (name == "stop_motor_fault") {
    ready();
    trip(COMMAND_TIMEOUT);
    // The transient error exists only during the actual stop transactions;
    // later recovery polls never see it, so stop must preserve its severity.
    simMotor[1].error = 1;
    performStop();
    simMotor[1].error = 0;
    runFor(4000);
    assertInspectionLockout(COMMAND_TIMEOUT, 0);
    return;
  }
  if (name == "command_reply_motor_fault") {
    ready();
    trip(COMMAND_TIMEOUT);
    loop();
    runFor(100);
    assert(stopState == 3);
    // An error in a valid non-info reply must latch before a later healthy
    // info reply overwrites the wheel's reported error byte.
    simMotor[1].error = 1;
    assert(motorTransaction(2, false, 0, 2, false) == MOTOR_OK);
    simMotor[1].error = 0;
    runFor(4000);
    assertInspectionLockout(COMMAND_TIMEOUT, 0);
    return;
  }
  if (name == "repeat_stop_deadline") {
    ready();
    simMotor[3].absent = true;
    handleHostFrame(STOP, ++seq, nullptr, 0);
    loop();
    const uint32_t originalDeadline = stopVerifyStartedMs;
    const unsigned before = simMotor[0].queries;
    for (int i = 0; i < 8; i++) {
      runFor(300);
      handleHostFrame(STOP, ++seq, nullptr, 0);
      assert(lastAcceptedSequence == seq);
      assert(stopVerifyStartedMs == originalDeadline);
      assert(!stopPending);
    }
    assert(stopState == 4);
    assert(state == FAULT);
    assert(simMotor[0].queries > before + 10);
    simMotor[3].absent = false;
    runFor(4000);
    assert(stopState == 3);
    assert(state == DISARMED);
    return;
  }
  if (name == "old_stop") {
    ready();
    sendTargets(2000);
    const uint16_t boundary = lastCommandSequence;
    handleHostFrame(STOP, 0, nullptr, 0);
    assert(lastCommandSequence == boundary);
    assert(!freshCommand(boundary));
    assert(!freshCommand(boundary - 1));
    runFor(400);
    assert(stopState == 3);
    return;
  }
  if (name == "session_replay") {
    ready();
    const uint64_t oldSession = hostSession;
    const uint16_t oldSeq = seq;
    ProtocolV2::TargetsPayload replay{
        {bootId, oldSession}, {1000, 1000, 1000, 1000}, 2};
    hello();
    assert(hostSession != oldSession);
    assert(!configured);
    handleHostFrame(TARGETS, oldSeq + 100, reinterpret_cast<uint8_t *>(&replay),
                    sizeof(replay));
    assert(!haveTargets);
    assert(!motionState());
    assert(reasonFlags & R_SESSION);
    uint8_t p[8];
    put64(p, 0x1234567890ULL);
    const uint64_t previous = hostSession;
    handleHostFrame(HELLO, 1, p, 8);
    assert(hostSession != previous);
    assert(!haveTargets);
    return;
  }
  if (name == "applied_complete") {
    ready();
    sendTargets(3000);
    const uint16_t prior = lastAppliedSequence,
                   targetSeq = lastAcceptedSequence;
    assert(prior != targetSeq);
    injectStopAfterWheel = 2;
    previousSweepUs = micros() - 15000;
    controlSweep();
    assert(stopPending);
    assert(lastAppliedSequence == prior);
    assert(!unsafePositionCommands);
    return;
  }
  if (name == "full_profiles") {
    ready();
    assert(cfg.maxCurrentMa == 2700);
    runFor(1000, true, 2000, 0);
    for (const auto &w : wheel)
      near(w.effectiveCapMa, 800);
    runFor(300, true, 2000, 1);
    assert(appliedProfile == 1);
    for (const auto &w : wheel) {
      assert(w.effectiveCapMa > 800);
      assert(w.effectiveCapMa < 1500);
    }
    runFor(600, true, 2000, 1);
    for (const auto &w : wheel) {
      near(w.effectiveCapMa, 1500);
      assert(w.effectiveCapMa > 1200);
    }
    // Production boot starts at zero; only this isolated test supplies a
    // budget so it can verify the full Boost envelope without a 60 s refill.
    boostRemainingMs = 15000;
    runFor(300, true, 2000, 2);
    assert(appliedProfile == 2);
    for (const auto &w : wheel) {
      assert(w.effectiveCapMa > 1500);
      assert(w.effectiveCapMa < 2500);
    }
    runFor(1000, true, 2000, 2);
    for (const auto &w : wheel) {
      near(w.effectiveCapMa, 2500);
      assert(!(w.reason & R_CEILING));
      assert(std::fabs(w.lastCommandedMa) <= 2500);
    }
    assert(boostRemainingMs < 15000);
    assert(boostRemainingMs > 13000);
    assert(faultCode == NO_FAULT);
    return;
  }
  if (name == "profiles_ceiling") {
    ready();
    cfg.maxCurrentMa = 1200;
    runFor(1500, true, 4000, 1);
    for (auto &w : wheel) {
      assert(w.effectiveCapMa <= 1200);
      assert(w.reason & R_CEILING);
      assert(std::fabs(w.lastCommandedMa) <= 1200);
    }
    assert(appliedProfile == 1);
    return;
  }
  if (name == "boost_budget") {
    boot();
    refreshFeedback();
    boostRemainingMs = cfg.boostCapacityMs;
    state = ARMED;
    haveTargets = true;
    requestedProfile = 2;
    for (int i = 0; i < 240; i++) {
      advanceUs(100000);
      refreshFeedback();
      protectionUpdate(.1f);
    }
    assert(boostRemainingMs == 0);
    assert(appliedProfile == 1);
    assert(reasonFlags & R_BOOST_EMPTY);
    for (int i = 0; i < 100; i++) {
      advanceUs(100000);
      refreshFeedback();
      protectionUpdate(.1f);
      assert(boostRemainingMs == 0);
      assert(appliedProfile == 1);
    }
    requestedProfile = 0;
    for (int i = 0; i < 600; i++) {
      advanceUs(100000);
      refreshFeedback();
      protectionUpdate(.1f);
    }
    near(boostRemainingMs, cfg.boostCapacityMs, 1);
    const float budget = boostRemainingMs;
    hello();
    assert(boostRemainingMs <= budget);
    return;
  }
  if (name == "stop_boost_refill") {
    ready();
    runFor(4000, true, 2000, 0);
    boostRemainingMs = 1000;
    runFor(100, true, 2000, 2);
    assert(requestedProfile == 2);
    const float beforeStop = boostRemainingMs;
    assert(beforeStop < 1000);
    const uint32_t refillStarted = millis();
    handleHostFrame(STOP, ++seq, nullptr, 0);
    loop();
    assert(requestedProfile == 0);
    assert(stagedProfile == 0);
    assert(appliedProfile == 0);
    assert(!haveTargets);
    near(boostRemainingMs, beforeStop, 1);
    runFor(2000);
    assert(state == DISARMED);
    assert(stopState == 3);
    assert(!haveTargets);
    assert(boostRemainingMs > beforeStop);
    const float elapsedRefill = (millis() - refillStarted) *
                                float(cfg.boostCapacityMs) / cfg.boostRefillMs;
    assert(boostRemainingMs <= beforeStop + elapsedRefill + 1);
    assert(boostRemainingMs < 2000);
    const float beforeHello = boostRemainingMs;
    const uint32_t helloStarted = millis();
    hello();
    const float helloRefill = (millis() - helloStarted) *
                              float(cfg.boostCapacityMs) / cfg.boostRefillMs;
    assert(boostRemainingMs <= beforeHello + helloRefill + 1);
    assert(boostRemainingMs < 2500);
    assert(!haveTargets);
    return;
  }
  if (name == "boost_stale") {
    ready();
    boostRemainingMs = 10000;
    requestedProfile = 2;
    refreshFeedback();
    advanceUs(800000);
    for (auto &w : wheel)
      w.lastFeedbackMs = millis();
    protectionUpdate(.8f);
    assert(appliedProfile == 1);
    assert(reasonFlags & R_TEMP_STALE);
    assert(faultCode == NO_FAULT);
    advanceUs(800000);
    for (auto &w : wheel)
      w.lastFeedbackMs = millis();
    protectionUpdate(.8f);
    assert(faultCode == TEMPERATURE_STALE);
    assert(stopPending);
    return;
  }
  if (name == "hot_driving_neutral") {
    ready();
    runFor(1000, true, 2000, 1);
    refreshFeedback();
    state = ARMED;
    Wheel &w = wheel[0];
    w.tempC = cfg.holdTempC;
    protectionUpdate(.1f);
    assert(w.holdCapMa == 0);
    assert(w.effectiveCapMa == 800);
    w.rpm = 0;
    w.rampRpm = 20;
    targetCentiRpm[0] = 2000;
    const int driving = calculateCurrent(0, .015f);
    assert(driving > 0);
    assert(driving <= 800);
    near(w.controlCapMa, 800);
    targetCentiRpm[0] = 0;
    w.rampRpm = 0;
    w.integralRpmS = 0;
    w.rpm = 10;
    const int braking = calculateCurrent(0, .015f);
    assert(braking < 0);
    assert(std::abs(braking) <= cfg.neutralBrakeMa);
    near(w.controlCapMa, cfg.neutralBrakeMa);
    state = HOLDING;
    w.lastPositionMs = millis();
    w.positionValid = true;
    w.positionUnwrapped = w.holdAnchor + 1000;
    assert(calculateCurrent(0, .015f) == 0);
    near(w.controlCapMa, 0);
    assert(reasonFlags & R_HOLD_LIMITED);
    return;
  }
  if (name == "gentle_overcurrent") {
    ready();
    runFor(1000, true, 2000, 0);
    simMotor[0].feedbackCurrent = 1500;
    runFor(100, true, 2000, 0);
    assert(faultCode == NO_FAULT);
    simMotor[0].feedbackCurrent = 0;
    runFor(100, true, 2000, 0);
    assert(faultCode == NO_FAULT);
    assert(wheel[0].currentSinceMs == 0);
    simMotor[0].feedbackCurrent = 1500;
    runFor(400, true, 2000, 0);
    assert(faultCode == ABNORMAL_CURRENT);
    assert(faultWheel == 1);
    return;
  }
  if (name == "hold_neutral_overcurrent") {
    ready();
    runFor(800, true);
    assert(state == HOLDING);
    simMotor[0].feedbackCurrent = 800;
    runFor(400, true);
    assert(faultCode == ABNORMAL_CURRENT);
    assert(faultWheel == 1);
    return;
  }
  if (name == "thermal") {
    ready();
    refreshFeedback();
    wheel[0].tempC = 50;
    protectionUpdate(.1f);
    assert(wheel[0].reason & R_TEMP_WARN);
    wheel[0].tempC = 55;
    protectionUpdate(.1f);
    assert(wheel[0].reason & R_DERATE);
    assert(wheel[0].effectiveCapMa <= 800);
    wheel[0].tempC = 53;
    protectionUpdate(.1f);
    assert(wheel[0].thermalDerating);
    wheel[0].tempC = 52;
    protectionUpdate(.1f);
    assert(!wheel[0].thermalDerating);
    wheel[0].tempC = 65;
    protectionUpdate(.1f);
    assert(faultCode == OVERTEMPERATURE);
    loop();
    runFor(500);
    assert(stopState == 3);
    assert(state == FAULT);
    for (auto &m : simMotor)
      m.temp = 25;
    runFor(4000);
    assert(state == DISARMED);
    assert(!haveTargets);
    return;
  }
  if (name == "hold_wrap") {
    ready();
    runFor(800, true);
    assert(state == HOLDING);
    Wheel &w = wheel[0];
    w.positionValid = true;
    w.positionUnwrapped = w.holdAnchor = 65530;
    w.lastPositionRaw = 65530;
    w.lastPositionMs = millis();
    updatePosition(w, 6, millis());
    assert(w.positionUnwrapped == 65542);
    w.holdCapMa = 300;
    cfg.holdKp = 100;
    assert(calculateCurrent(0, .015f) < 0);
    w.positionUnwrapped = w.holdAnchor + 65536;
    const int effort = calculateCurrent(0, .015f);
    assert(effort >= -300);
    assert(effort <= 0);
    assert(reasonFlags & R_HOLD_LIMITED);
    cfg.encoderCountsPerRev = 32768;
    w.positionUnwrapped = w.holdAnchor = 32760;
    w.lastPositionRaw = 32760;
    w.lastPositionMs = millis();
    updatePosition(w, 4, millis());
    assert(w.positionUnwrapped == 32772);
    return;
  }
  if (name == "encoder_long_run") {
    ready();
    runFor(800, true);
    assert(state == HOLDING);
    Wheel &w = wheel[0];
    cfg.holdKp = 100;
    w.lastPositionRaw = 100;
    w.positionUnwrapped = w.holdAnchor = int64_t(INT32_MAX) - 5;
    w.lastPositionMs = millis();
    updatePosition(w, 120, millis());
    assert(w.positionUnwrapped == int64_t(INT32_MAX) + 15);
    assert(calculateCurrent(0, .015f) < 0);
    w.lastPositionRaw = 100;
    w.positionUnwrapped = w.holdAnchor = int64_t(INT32_MIN) + 5;
    w.lastPositionMs = millis();
    updatePosition(w, 80, millis());
    assert(w.positionUnwrapped == int64_t(INT32_MIN) - 15);
    assert(calculateCurrent(0, .015f) > 0);
    return;
  }
  if (name == "hold_independent") {
    ready();
    runFor(800, true);
    assert(state == HOLDING);
    cfg.gentleMa = 50;
    requestedProfile = 0;
    refreshFeedback();
    protectionUpdate(.1f);
    assert(wheel[0].holdCapMa == 300);
    wheel[0].tempC = 64;
    protectionUpdate(.1f);
    assert(wheel[0].holdCapMa < 300);
    assert(reasonFlags & R_HOLD_LIMITED);
    return;
  }
  if (name == "inner_zero") {
    ready();
    runFor(1500, true, 2000, 0, true);
    assert(state == ARMED);
    assert(targetCentiRpm[0] == 0);
    assert(targetCentiRpm[1] == 2000);
    assert(wheel[0].lastCommandedMa == 0);
    return;
  }
  if (name == "disarmed_hold") {
    boot();
    hello();
    cfg.disarmedHoldEnabled = 1;
    configure();
    runFor(500);
    assert(state == HOLDING);
    assert(disarmedHolding);
    wheel[0].holdIntegral = 4;
    const int64_t anchor = wheel[0].holdAnchor;
    arm();
    assert(!disarmedHolding);
    assert(state == HOLDING);
    assert(wheel[0].holdAnchor == anchor);
    assert(wheel[0].holdIntegral == 4);
    return;
  }
  if (name == "stall") {
    ready();
    simMotor[0].stuck = true;
    runFor(1800, true, 4000, 0);
    assert(faultCode == STALL);
    assert(faultWheel == 1);
    assert(state == FAULT);
    const unsigned q = simMotor[0].queries;
    runFor(500);
    assert(simMotor[0].queries > q);
    assert(stopState == 3);
    assert(state == FAULT);
    return;
  }
  if (name == "reversal") {
    ready();
    runFor(1000, true, 2000);
    assert(faultCode == NO_FAULT);
    runFor(1200, true, -2000);
    assert(faultCode == NO_FAULT);
    assert(wheel[0].rpm < 0);
    return;
  }
  if (name == "abnormal_current") {
    ready();
    simMotor[1].feedbackCurrent = 3000;
    runFor(600, true, 2000);
    assert(faultCode == ABNORMAL_CURRENT);
    assert(faultWheel == 2);
    assert(state == FAULT);
    return;
  }
  if (name == "command_expiry") {
    ready();
    runFor(1000, true, 2000);
    runFor(800);
    assert(faultCode == COMMAND_TIMEOUT);
    assert(state == FAULT);
    runFor(4000);
    assert(state == DISARMED);
    assert(!haveTargets);
    for (auto t : targetCentiRpm)
      assert(t == 0);
    return;
  }
  if (name == "crc_recovery") {
    ready();
    simMotor[2].corrupt = true;
    runFor(800, true, 1000);
    assert(faultCode == BAD_MOTOR_FRAME);
    const unsigned q = simMotor[0].queries;
    runFor(500);
    assert(simMotor[0].queries > q);
    simMotor[2].corrupt = false;
    runFor(4000);
    assert(state == DISARMED);
    return;
  }
  if (name == "config_motion_reject") {
    ready();
    const uint32_t old = configId;
    ProtocolV2::ConfigPayload p{{bootId, hostSession}, 99, {}};
    handleHostFrame(CONFIG, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
    assert(configResult == 2);
    assert(configId == old);
    assert(configured);
    return;
  }
  if (name == "parser_noise") {
    boot();
    uint8_t p[17] = {0xa5, 0x5a, 2, HELLO, 1, 0, 8};
    put64(p + 7, 0xa5a5a5a5ULL);
    put16(p + 15, crc16(p + 2, 13));
    Serial.incoming.push_back(0x44);
    Serial.incoming.push_back(0xa5);
    for (auto b : p)
      Serial.incoming.push_back(b);
    pollHost();
    assert(hostSession);
    assert(helloNonce == 0xa5a5a5a5ULL);
    assert(!motionState());
    assert(!haveTargets);
    assert(stopState == 3);
    return;
  }
  if (name == "bounded_drain") {
    boot();
    for (int i = 0; i < 300; i++)
      Serial1.incoming.push_back(0xff);
    const uint32_t began = micros();
    const unsigned q = simMotor[0].queries;
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_BAD_REPLY);
    assert(micros() == began);
    assert(simMotor[0].queries == q);
    return;
  }
  if (name == "progress_watchdog") {
    ready();
    runFor(1000, true, 1000);
    const uint32_t before = nativeProgressMs;
    assert(millis() - before < 150);
    advanceUs(1100000);
    assert(millis() - nativeProgressMs >= 1000);
    boostRemainingMs = 15000;
    // Native harness models a reboot entry after completed-work timeout;
    // actual panic/reset wiring is compiled and must be checked on hardware.
    setup();
    assert(boostRemainingMs == 0);
    assert(!haveTargets);
    runFor(400);
    assert(stopState == 3);
    return;
  }
  if (name == "rollover") {
    ready();
    simTimeUs = (uint64_t(UINT32_MAX) - 150) * 1000;
    lastTargetMs = protectionPreviousMs = millis();
    previousSweepUs = nextSweepUs = micros();
    coolSinceMs = millis() - 1000;
    for (auto &w : wheel) {
      w.lastFeedbackMs = w.lastInfoMs = w.lastPositionMs = millis();
    }
    runFor(600, true, 1000);
    assert(millis() < 1000);
    assert(faultCode == NO_FAULT);
    assert(motionState());
    assert(millis() - lastTargetMs < 100);
    haveCommandSequence = true;
    lastCommandSequence = 65534;
    assert(freshCommand(65535));
    acceptCommand(65535);
    assert(freshCommand(0));
    acceptCommand(0);
    assert(freshCommand(1));
    assert(!freshCommand(65535));
    assert(!freshCommand(32768));
    handleHostFrame(STOP, 65535, nullptr, 0);
    assert(lastCommandSequence == 0);
    runFor(500);
    assert(stopState == 3);
    return;
  }
  if (name == "bounded_io") {
    boot();
    Serial.writeRoom = 0;
    const uint32_t began = micros();
    sendStatus();
    assert(micros() == began);
    Serial1.writeRoom = 0;
    const MotorResult r = motorTransaction(1, true, 0, 0, false);
    assert(r == MOTOR_NO_REPLY);
    assert(micros() == began);
    return;
  }
  throw std::runtime_error("unknown simulator case " + name);
}
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  execute(argv[1]);
  std::cout << "PASS " << argv[1] << " time_ms=" << millis() << "\n";
  return 0;
}

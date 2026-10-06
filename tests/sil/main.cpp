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
  // STATUS must expose the latch even when the first cause was recoverable.
  protectionUpdate(0);
  assert(reasonFlags & R_INSPECTION);
  const uint16_t acceptedBefore = lastAcceptedSequence;
  ProtocolV2::ArmPayload p{{bootId, hostSession}, configId};
  handleHostFrame(ARM, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
  assert(!armPending);
  loop();
  assert(!motionState());
  assert(lastAcceptedSequence == acceptedBefore);
}
static void forceRpm(uint8_t index, float rpm, uint32_t ms) {
  const uint32_t end = millis() + ms;
  while (static_cast<int32_t>(millis() - end) < 0) {
    simMotor[index].rpm = rpm;
    loop();
    advanceUs(1000);
  }
}
// Peak true wheel rotation in degrees, measured on the simulated motor rather
// than through the firmware's own encoder scaling.
static float simRotationDuring(uint8_t index, uint32_t ms) {
  float last = simMotor[index].position, counts = 0, peak = 0;
  const uint32_t end = millis() + ms;
  uint32_t refresh = millis();
  while (static_cast<int32_t>(millis() - end) < 0) {
    if (static_cast<int32_t>(millis() - refresh) >= 0) {
      sendTargets(0);
      refresh = millis() + 50;
    }
    loop();
    advanceUs(1000);
    float delta = simMotor[index].position - last;
    if (delta > simCountsPerRev / 2)
      delta -= simCountsPerRev;
    else if (delta < -simCountsPerRev / 2)
      delta += simCountsPerRev;
    counts += delta;
    last = simMotor[index].position;
    peak = std::max(peak, std::fabs(counts));
  }
  return peak * 360.0f / simCountsPerRev;
}
// Reply-retry tolerance. REPLY_RETRY is reason bit 18 and a motor transaction
// makes at most 3 attempts; both are written out as literals so these tests
// need no firmware symbol that the change itself introduces.
static constexpr uint32_t kReplyRetry = 262144;
static constexpr unsigned kAttempts = 3;
static void clearSimFaults(SimMotor &m) {
  m.passRequests = m.ignoreRequests = m.dropReplies = m.garbleReplies = 0;
}
// Step the real loop() until it has started one more control sweep, renewing
// targets so the command watchdog stays quiet. It returns right after that
// iteration, so simulator counters read here belong to that sweep alone.
static void runOneSweep(int16_t target = 0, uint8_t profile = 0) {
  const uint32_t scheduled = nextSweepUs;
  for (int guard = 0; nextSweepUs == scheduled; guard++) {
    assert(guard < 100);
    if (millis() - lastTargetMs >= 50)
      sendTargets(target, profile);
    loop();
    advanceUs(1000);
  }
}
// REPLY_RETRY names exactly the wheel that missed a reply (-1: no wheel).
static void expectReplyRetry(int missed) {
  assert(bool(reasonFlags & kReplyRetry) == (missed >= 0));
  for (int i = 0; i < 4; i++)
    assert(bool(wheel[i].reason & kReplyRetry) == (i == missed));
}
// Wheel `w` loses one reply of kind `failure` in one sweep of a drive. Retrying
// must hide it: the sweep still reaches every wheel, nothing faults, the sweep
// counts as applied, and only REPLY_RETRY records that it happened.
static void driveThroughFailure(uint8_t w, unsigned SimMotor::*failure) {
  ready();
  runFor(1500, true, 2000, 1);
  // A target step makes every sweep command a clearly different current, so
  // "this sweep's command reached the motor" cannot pass on a stale value.
  sendTargets(3500, 1);
  runOneSweep(3500, 1);
  const float before = simMotor[w].currentMa;
  unsigned commands[4];
  for (int i = 0; i < 4; i++)
    commands[i] = simMotor[i].commands;
  // Fresh targets, so the sweep that meets the failure has a new sequence to apply.
  sendTargets(3500, 1);
  assert(lastAppliedSequence != lastAcceptedSequence);
  simMotor[w].*failure = 1;
  runOneSweep(3500, 1);
  assert(faultCode == NO_FAULT);
  assert(motionState());
  assert(simMotor[w].*failure == 0);
  // The failed request and one immediate retry, not a skipped wheel.
  assert(simMotor[w].commands == commands[w] + 2);
  for (int i = 0; i < 4; i++)
    if (i != w)
      assert(simMotor[i].commands == commands[i] + 1);
  assert(std::fabs(wheel[w].lastCommandedMa - before) > 10);
  near(simMotor[w].currentMa, wheel[w].lastCommandedMa, 0.5f);
  assert(wheel[w].valid && millis() - wheel[w].lastFeedbackMs <= cfg.feedbackMs);
  assert(lastAppliedSequence == lastAcceptedSequence);
  // A miss that a retry answered leaves feedback fresh, so the wheel's
  // controller keeps running on the next sweep: it is not put on hold.
  const float ramp = wheel[w].rampRpm;
  runOneSweep(3500, 1);
  assert(wheel[w].rampRpm > ramp);
  // REPLY_RETRY lasts 1000 ms from the wheel's most recent failed attempt, and
  // only warns: a second failure 600 ms on extends it, and driving carries on.
  runFor(30, true, 3500, 1);
  expectReplyRetry(w);
  runFor(570, true, 3500, 1);
  simMotor[w].*failure = 1;
  runOneSweep(3500, 1);
  assert(faultCode == NO_FAULT);
  assert(simMotor[w].*failure == 0);
  runFor(500, true, 3500, 1);
  expectReplyRetry(w);
  runFor(300, true, 3500, 1);
  expectReplyRetry(w);
  runFor(300, true, 3500, 1);
  expectReplyRetry(-1);
  assert(faultCode == NO_FAULT);
  assert(state == ARMED);
  assert(wheel[w].lastCommandedMa > 100);
  // Holding position at neutral is a current-mode state too: one miss is
  // retried, and a sweep that loses all three attempts does not fault either.
  runFor(800, true);
  assert(state == HOLDING);
  unsigned held = simMotor[w].commands;
  simMotor[w].*failure = 1;
  runOneSweep();
  assert(faultCode == NO_FAULT);
  assert(state == HOLDING);
  assert(simMotor[w].commands == held + 2);
  held = simMotor[w].commands;
  simMotor[w].*failure = kAttempts;
  runOneSweep();
  assert(faultCode == NO_FAULT);
  assert(state == HOLDING);
  assert(simMotor[w].commands == held + kAttempts);
  runOneSweep();
  runOneSweep();
  assert(faultCode == NO_FAULT);
  assert(state == HOLDING);
}
// Wheel `w` goes silent, or garbles every reply, while driving. The firmware
// rides it out for the feedback limit and no longer, with bounded retries, and
// then faults on that wheel and stops. The stop sequence starts as the fault
// latches, so stopVerifyStartedMs is the fault time without the stop's own
// duration, which loop() would otherwise add. With `overcurrent`, the wheel
// last reported a current above the abnormal-current ceiling 120 ms before it
// went silent: re-checking that stale reading would trip ABNORMAL_CURRENT, an
// inspection fault, at 200 ms, before the feedback limit has produced the
// recoverable timeout.
static void driveToLimit(uint8_t w, bool garbled, bool overcurrent = false) {
  ready();
  runFor(1500, true, 2000, 1);
  if (overcurrent) {
    simMotor[w].feedbackCurrent = 2500;
    runFor(120, true, 2000, 1);
    assert(faultCode == NO_FAULT);
    assert(wheel[w].currentSinceMs);
  }
  // Keep the temperature poll out of the window: every request to `w` is then
  // a retry of its current command.
  for (auto &x : wheel)
    x.lastInfoMs = millis();
  const uint8_t healthy = (w + 1) % 4;
  const unsigned requests0 = simMotor[w].commands + simMotor[w].queries,
                 sweeps0 = simMotor[healthy].commands;
  const uint32_t t0 = millis();
  (garbled ? simMotor[w].corrupt : simMotor[w].absent) = true;
  unsigned requests = 0, sweeps = 0;
  while (faultCode == NO_FAULT) {
    assert(millis() - t0 <= cfg.feedbackMs + 60u);
    // Counted before the iteration that trips, so the stop sequence's own
    // requests are not mistaken for retries. Every healthy wheel is commanded
    // once per sweep, so its command count is the number of sweeps that ran.
    requests = simMotor[w].commands + simMotor[w].queries - requests0;
    sweeps = simMotor[healthy].commands - sweeps0;
    if (millis() - lastTargetMs >= 50)
      sendTargets(2000, 1);
    loop();
    advanceUs(1000);
  }
  if (stopPending) {  // latched inside a sweep: the stop starts next iteration
    loop();
    advanceUs(1000);
  }
  const int32_t latched = static_cast<int32_t>(stopVerifyStartedMs - t0);
  std::cerr << (garbled ? "drive_garble_limit"
                        : overcurrent ? "drive_stale_skips_health_check"
                                      : "drive_silence_limit")
            << " latched_ms=" << latched << " sweeps=" << sweeps
            << " requests=" << requests << "\n";
  assert(latched >= static_cast<int>(cfg.feedbackMs) - 20);
  assert(latched <= static_cast<int>(cfg.feedbackMs) + 60);
  assert(faultCode == (garbled ? BAD_MOTOR_FRAME : MOTOR_TIMEOUT));
  assert(faultWheel == w + 1);
  assert(!inspectionRequired);
  assert(sweeps >= 2);
  assert(requests <= kAttempts * sweeps);
  runFor(30);
  assert(!stopPending);
  assert(!haveTargets);
  assert(state == FAULT);
  for (int i = 0; i < 4; i++)
    if (i != w)
      assert(simMotor[i].mode == 2);
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
    // A recoverable timeout is the first cause; a motor error seen afterwards,
    // during stop/recovery polling, forces the inspection lockout while the
    // first cause stays recorded. The timeout must come first: a lone missed
    // reply is retried and the silent wheel only faults at the feedback limit,
    // so an error already present from the start would be seen by the sweeps
    // before that limit and become the first cause instead.
    ready();
    simMotor[0].absent = true;
    runFor(400, true);
    assert(faultCode == MOTOR_TIMEOUT);
    assert(faultWheel == 1);
    simMotor[1].error = 1;
    runFor(200);
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
    cfg.encoderCountsPerRev = 65536;
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
  if (name == "encoder_wrap_hold") {
    // Documented 32768-count motors with the default configuration: a held
    // wheel creeping across the position wrap must not see a false half turn.
    for (auto &m : simMotor)
      m.position = simCountsPerRev - 60;
    ready();
    runFor(800, true);
    assert(state == HOLDING);
    simMotor[0].externalLoad = 2;
    const float degrees = simRotationDuring(0, 10000);
    std::cerr << "encoder_wrap_hold peak_deg=" << degrees << "\n";
    assert(degrees < 30);
    assert(state == HOLDING);
    assert(faultCode == NO_FAULT);
    assert(cfg.encoderCountsPerRev == uint32_t(simCountsPerRev));
    return;
  }
  if (name == "warm_command_recovery") {
    // Command and feedback faults are not thermal: motors that are warm but
    // below the warning threshold must not block recovery to readiness.
    ready();
    runFor(1000, true, 2000, 1);
    for (auto &m : simMotor)
      m.temp = 47;
    runFor(800);
    assert(faultCode == COMMAND_TIMEOUT);
    runFor(5000);
    assert(faultCode == NO_FAULT);
    assert(state == DISARMED);
    assert(!haveTargets);
    // A motor-reported overtemperature still waits for the release threshold.
    arm();
    runFor(500, true, 2000, 1);
    simMotor[1].temp = 66;
    runFor(1500, true, 2000, 1);
    assert(faultCode == OVERTEMPERATURE);
    assert(faultWheel == 2);
    simMotor[1].temp = 47;
    runFor(5000);
    assert(state == FAULT);
    for (auto &m : simMotor)
      m.temp = 44;
    runFor(5000);
    assert(faultCode == NO_FAULT);
    assert(state == DISARMED);
    return;
  }
  if (name == "disarmed_push") {
    // A brief push after a confirmed stop is verified again rather than
    // latched as a motor fault. Sustained motion past the verification window
    // still latches an inspection fault that names the moving wheel.
    boot();
    hello();
    configure();
    runFor(2000);
    assert(state == DISARMED);
    assert(stopState == 3);
    forceRpm(0, 8, 150);
    runFor(3000);
    assert(faultCode == NO_FAULT);
    assert(!inspectionRequired);
    assert(stopState == 3);
    arm();
    handleHostFrame(STOP, ++seq, nullptr, 0);
    runFor(2500);
    assert(stopState == 3);
    forceRpm(2, 8, 2500);
    assert(stopState == 4);
    assert(faultCode == MOTOR_FAULT);
    assert(faultWheel == 3);
    assert(inspectionRequired);
    return;
  }
  if (name == "slope_settle") {
    // Neutral on a slope must reach position holding instead of creeping
    // indefinitely just above the stationary threshold.
    boot();
    hello();
    configure();
    for (auto &m : simMotor)
      m.externalLoad = -30;
    arm();
    runFor(1500, true);
    assert(state == HOLDING);
    const float degrees = simRotationDuring(0, 10000);
    std::cerr << "slope_settle peak_deg=" << degrees << "\n";
    assert(degrees < 90);
    assert(state == HOLDING);
    assert(faultCode == NO_FAULT);
    return;
  }
  if (name == "config_result_scope") {
    // config_result reports CONFIG outcomes only. A stale motion frame must
    // not overwrite the applied result the Pi verifies on every cycle.
    ready();
    assert(configResult == 1);
    const uint16_t ack = configAckSequence;
    ProtocolV2::TargetsPayload stale{{bootId, hostSession + 1}, {0, 0, 0, 0}, 0};
    handleHostFrame(TARGETS, ++seq, reinterpret_cast<uint8_t *>(&stale),
                    sizeof(stale));
    assert(configResult == 1);
    assert(configAckSequence == ack);
    ProtocolV2::ConfigPayload wrong{};
    wrong.session = {bootId, hostSession + 1};
    const uint16_t configSequence = ++seq;
    handleHostFrame(CONFIG, configSequence, reinterpret_cast<uint8_t *>(&wrong),
                    sizeof(wrong));
    assert(configResult == 4);
    assert(configAckSequence == configSequence);
    ProtocolV2::ConfigPayload old{};
    old.session = {bootId, hostSession};
    handleHostFrame(CONFIG, configSequence - 10,
                    reinterpret_cast<uint8_t *>(&old), sizeof(old));
    assert(configAckSequence == configSequence);
    return;
  }
  if (name == "derate_disables_boost") {
    // A derating temperature reported over the motor bus disables Boost for
    // the robot without consuming the remaining allowance.
    ready();
    simMotor[1].temp = 56;
    runFor(1000, true, 2000, 1);
    assert(wheel[1].tempC == 56);
    assert(wheel[1].thermalDerating);
    boostRemainingMs = 10000;
    runFor(1000, true, 2000, 2);
    assert(requestedProfile == 2);
    assert(appliedProfile == 1);
    near(boostRemainingMs, 10000, 1);
    assert(wheel[1].effectiveCapMa <= cfg.gentleMa);
    for (int i : {0, 2, 3})
      assert(wheel[i].effectiveCapMa <= cfg.normalMa + 1);
    assert(reasonFlags & R_DERATE);
    return;
  }
  if (name == "sim_transport_faults") {
    // Self-check of the simulator's fault counters, which every retry scenario
    // relies on. It uses motorTransaction, which stays single-attempt.
    boot();
    SimMotor &m = simMotor[0];
    const unsigned queries = m.queries, commands = m.commands,
                   others = simMotor[1].queries + simMotor[2].queries + simMotor[3].queries;
    m.passRequests = m.ignoreRequests = m.dropReplies = m.garbleReplies = 1;
    sendMotorMode(1, 2);  // a mode write never consumes a counter
    assert(m.passRequests + m.ignoreRequests + m.dropReplies + m.garbleReplies == 4);
    // One counter per request, in order: pass, lose, drop, garble, then healthy.
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_OK);
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_NO_REPLY);
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_NO_REPLY);
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_BAD_REPLY);
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_OK);
    assert(m.passRequests + m.ignoreRequests + m.dropReplies + m.garbleReplies == 0);
    // A lost request still counts as sent; the other motors saw nothing.
    assert(m.queries == queries + 5);
    assert(simMotor[1].queries + simMotor[2].queries + simMotor[3].queries == others);
    // A lost command never reaches the motor; one whose reply is dropped does.
    m.mode = 1;
    m.currentMa = 0;
    m.ignoreRequests = 1;
    assert(motorTransaction(1, false, 4000, 0, false) == MOTOR_NO_REPLY);
    assert(m.currentMa == 0);
    m.dropReplies = 1;
    assert(motorTransaction(1, false, 4000, 0, false) == MOTOR_NO_REPLY);
    near(m.currentMa, 4000 * 8000.0f / 32767.0f, 0.01f);
    assert(m.commands == commands + 2);
    // garbleReplies is corrupt for exactly N replies.
    m.garbleReplies = 2;
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_BAD_REPLY);
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_BAD_REPLY);
    assert(motorTransaction(1, true, 0, 0, false) == MOTOR_OK);
    return;
  }
  if (name == "drive_single_drop") {
    driveThroughFailure(2, &SimMotor::dropReplies);
    return;
  }
  if (name == "drive_single_garble") {
    driveThroughFailure(3, &SimMotor::garbleReplies);
    return;
  }
  if (name == "drive_lost_command_redelivered") {
    // The first attempt never reaches the motor. Only the immediate retry can
    // deliver this sweep's current, which driveThroughFailure checks on the motor.
    driveThroughFailure(0, &SimMotor::ignoreRequests);
    return;
  }
  if (name == "drive_silence_limit") {
    driveToLimit(1, false);
    return;
  }
  if (name == "drive_garble_limit") {
    driveToLimit(2, true);
    return;
  }
  if (name == "drive_stale_skips_health_check") {
    driveToLimit(1, false, true);
    return;
  }
  if (name == "applied_needs_all_wheels") {
    ready();
    sendTargets(3000);
    const uint16_t prior = lastAppliedSequence, targetSeq = lastAcceptedSequence;
    assert(prior != targetSeq);
    simMotor[1].dropReplies = kAttempts;
    previousSweepUs = micros() - 15000;
    controlSweep();
    // Wheel 2 lost every attempt: no fault, but its command was never
    // acknowledged, so the new targets are not applied yet.
    assert(faultCode == NO_FAULT);
    assert(!stopPending);
    assert(simMotor[1].dropReplies == 0);
    assert(lastAppliedSequence == prior);
    // The wheel answers again. Whether the sweep that merely re-sends its last
    // current counts is left open; once its normal control has resumed, the
    // targets are applied.
    previousSweepUs = micros() - 15000;
    controlSweep();
    previousSweepUs = micros() - 15000;
    controlSweep();
    assert(faultCode == NO_FAULT);
    assert(lastAppliedSequence == targetSeq);
    return;
  }
  if (name == "drive_stale_holds_current") {
    ready();
    runFor(1500, true, 2000, 1);
    // No temperature poll may give wheel 1 a valid reply between its failed
    // sweep and the next one.
    for (auto &x : wheel)
      x.lastInfoMs = millis();
    // A target step makes every sweep command a clearly different current. At a
    // steady state the controller's output barely moves between sweeps, so
    // neither a re-sent nor a recomputed current could be told apart.
    sendTargets(3500, 1);
    runOneSweep(3500, 1);
    runOneSweep(3500, 1);
    const int16_t acknowledged = wheel[1].lastCommandedMa;
    assert(acknowledged > 100);
    // One sweep in which wheel 1 loses every attempt.
    unsigned commands[4];
    for (int i = 0; i < 4; i++)
      commands[i] = simMotor[i].commands;
    simMotor[1].dropReplies = kAttempts;
    runOneSweep(3500, 1);
    assert(faultCode == NO_FAULT);
    assert(simMotor[1].dropReplies == 0);
    assert(simMotor[1].commands == commands[1] + kAttempts);
    for (int i = 0; i < 4; i++)
      if (i != 1)
        assert(simMotor[i].commands == commands[i] + 1);
    assert(wheel[1].lastCommandedMa == acknowledged);
    // The motor applied that unacknowledged command, so a re-sent current is
    // distinguishable from it.
    assert(std::fabs(simMotor[1].currentMa - acknowledged) > 10);
    const float integral = wheel[1].integralRpmS, ramp = wheel[1].rampRpm;
    float ramps[4];
    for (int i = 0; i < 4; i++)
      ramps[i] = wheel[i].rampRpm;
    // The next sweep succeeds. Wheel 1 has no valid reply since its last
    // command, so its controller must not run on that stale feedback: its last
    // acknowledged current is sent again and its controller state stands still.
    runOneSweep(3500, 1);
    assert(faultCode == NO_FAULT);
    near(simMotor[1].currentMa, acknowledged, 0.5f);
    assert(wheel[1].integralRpmS == integral);
    assert(wheel[1].rampRpm == ramp);
    for (int i = 0; i < 4; i++)
      if (i != 1) {
        assert(wheel[i].rampRpm > ramps[i]);
        near(simMotor[i].currentMa, wheel[i].lastCommandedMa, 0.5f);
      }
    // A valid reply has arrived: normal control resumes on the sweep after.
    runOneSweep(3500, 1);
    assert(faultCode == NO_FAULT);
    assert(wheel[1].rampRpm > ramp);
    assert(std::fabs(simMotor[1].currentMa - acknowledged) > 10);
    near(simMotor[1].currentMa, wheel[1].lastCommandedMa, 0.5f);
    assert(motionState());
    return;
  }
  if (name == "drive_info_poll_retry") {
    // The temperature/info poll follows the same rules as a command: retried at
    // once, and when all attempts fail the same wheel is polled again next sweep.
    ready();
    runFor(1500, true, 2000, 1);
    const uint8_t w = 2, other = 3;
    const uint32_t overdue = cfg.tempPollMs + 20;
    // Only `w` and `other` are due for a poll, `w` first in the rotation.
    for (auto &x : wheel)
      x.lastInfoMs = millis();
    wheel[w].lastInfoMs = wheel[other].lastInfoMs = millis() - overdue;
    infoCursor = w;
    // Each wheel's command is its first request of a sweep and the poll its second.
    simMotor[w].passRequests = 1;
    simMotor[w].dropReplies = 1;
    unsigned queries = simMotor[w].queries, commands = simMotor[w].commands,
             others = simMotor[other].queries;
    runOneSweep(2000, 1);
    assert(faultCode == NO_FAULT);
    assert(simMotor[w].dropReplies == 0);
    assert(simMotor[w].queries == queries + 2);
    assert(simMotor[w].commands == commands + 1);
    assert(simMotor[other].queries == others);
    assert(millis() - wheel[w].lastInfoMs < 50);
    runFor(30, true, 2000, 1);
    expectReplyRetry(w);
    // All attempts of the poll fail: no fault, and `w` stays first in line.
    wheel[w].lastInfoMs = wheel[other].lastInfoMs = millis() - overdue;
    infoCursor = w;
    simMotor[w].passRequests = 1;
    simMotor[w].dropReplies = kAttempts;
    queries = simMotor[w].queries;
    runOneSweep(2000, 1);
    assert(faultCode == NO_FAULT);
    assert(simMotor[w].dropReplies == 0);
    assert(simMotor[w].queries == queries + kAttempts);
    // The next sweep polls `w` again, not the next wheel in line.
    others = simMotor[other].queries;
    runOneSweep(2000, 1);
    assert(faultCode == NO_FAULT);
    assert(simMotor[w].queries == queries + kAttempts + 1);
    assert(simMotor[other].queries == others);
    assert(millis() - wheel[w].lastInfoMs < 50);
    return;
  }
  if (name == "drive_limit_follows_last_failure") {
    // At the feedback limit the fault kind follows the wheel's most recent failed
    // attempt, not its first: garbled then silent is a timeout, silent then
    // garbled is a bad frame.
    ready();
    const uint8_t w = 1;
    for (bool garbledLast : {false, true}) {
      runFor(1500, true, 2000, 1);
      for (auto &x : wheel)
        x.lastInfoMs = millis();
      const uint32_t t0 = millis();
      simMotor[w].corrupt = !garbledLast;
      simMotor[w].absent = garbledLast;
      runFor(70, true, 2000, 1);
      assert(faultCode == NO_FAULT);
      simMotor[w].corrupt = garbledLast;
      simMotor[w].absent = !garbledLast;
      while (faultCode == NO_FAULT) {
        assert(millis() - t0 < 400);
        if (millis() - lastTargetMs >= 50)
          sendTargets(2000, 1);
        loop();
        advanceUs(1000);
      }
      assert(faultCode == (garbledLast ? BAD_MOTOR_FRAME : MOTOR_TIMEOUT));
      assert(faultWheel == w + 1);
      simMotor[w].corrupt = simMotor[w].absent = false;
      runFor(4500);
      assert(faultCode == NO_FAULT);
      assert(state == DISARMED);
      arm();
    }
    return;
  }
  if (name == "disarmed_single_drop") {
    // A disarmed poll makes up to three requests per wheel: the info query, a
    // re-query after repairing a motor that is not in speed mode, and the zero
    // command. One dropped reply at any of them is retried and goes unnoticed.
    boot();
    hello();
    configure();
    for (uint8_t startMode : {2, 1}) {
      unsigned at = 0;
      for (;; at++) {
        assert(at < 8);
        const uint8_t w = (at + startMode) % 4;
        simMotor[w].mode = startMode;
        simMotor[w].passRequests = at;
        simMotor[w].dropReplies = 1;
        nextDisarmedPollMs = 0;
        pollDisarmed();
        const bool reached = simMotor[w].dropReplies == 0;
        clearSimFaults(simMotor[w]);
        if (!reached)
          break;
        assert(state == DISARMED);
        assert(faultCode == NO_FAULT);
        assert(stopState == 3);
        assert(simMotor[w].mode == 2);
        runFor(30);
        expectReplyRetry(w);
        runFor(770);
        expectReplyRetry(w);
        runFor(300);
        expectReplyRetry(-1);
      }
      assert(at >= (startMode == 2 ? 2u : 3u));
    }
    // The warning is informational: with REPLY_RETRY raised, arming and driving
    // still go ahead.
    simMotor[1].dropReplies = 1;
    nextDisarmedPollMs = 0;
    pollDisarmed();
    runFor(30);
    expectReplyRetry(1);
    arm();
    runFor(300, true, 2000, 1);
    expectReplyRetry(1);
    assert(state == ARMED);
    assert(faultCode == NO_FAULT);
    return;
  }
  if (name == "disarmed_hold_single_drop") {
    // Powered holding while disarmed is a current-mode state as well: a missed
    // reply is retried, and a sweep that loses every attempt for one wheel
    // neither faults nor ends the hold.
    boot();
    hello();
    cfg.disarmedHoldEnabled = 1;
    configure();
    runFor(500);
    assert(disarmedHolding);
    assert(state == HOLDING);
    const uint8_t w = 1;
    unsigned commands = simMotor[w].commands;
    simMotor[w].dropReplies = 1;
    runOneSweep();
    assert(simMotor[w].dropReplies == 0);
    assert(faultCode == NO_FAULT);
    assert(disarmedHolding && state == HOLDING);
    assert(simMotor[w].commands == commands + 2);
    assert(stopState == 3);
    runFor(30);
    expectReplyRetry(w);
    commands = simMotor[w].commands;
    simMotor[w].dropReplies = kAttempts;
    runOneSweep();
    assert(simMotor[w].dropReplies == 0);
    assert(faultCode == NO_FAULT);
    assert(simMotor[w].commands == commands + kAttempts);
    runOneSweep();
    runOneSweep();
    assert(faultCode == NO_FAULT);
    assert(disarmedHolding && state == HOLDING);
    assert(stopState == 3);
    return;
  }
  if (name == "disarmed_retry_bounded") {
    // A wheel missing for exactly one poll gets three info queries, no more,
    // and the poll then latches the failure on that wheel as it always has.
    boot();
    hello();
    configure();
    const uint8_t w = 2;
    for (bool garbled : {false, true}) {
      const unsigned queries = simMotor[w].queries,
                     commands = simMotor[w].commands;
      (garbled ? simMotor[w].corrupt : simMotor[w].absent) = true;
      nextDisarmedPollMs = 0;
      pollDisarmed();
      (garbled ? simMotor[w].corrupt : simMotor[w].absent) = false;
      assert(simMotor[w].queries == queries + kAttempts);
      assert(simMotor[w].commands == commands);
      assert(faultCode == (garbled ? BAD_MOTOR_FRAME : MOTOR_TIMEOUT));
      assert(faultWheel == w + 1);
      assert(state == FAULT);
      if (!garbled) {
        runFor(4500);
        assert(faultCode == NO_FAULT);
        assert(state == DISARMED);
      }
    }
    return;
  }
  if (name == "arm_single_drop") {
    // Arming asks every motor for its mode, then commands zero current: one
    // dropped reply at either request is retried and arming still succeeds.
    boot();
    hello();
    configure();
    unsigned at = 0;
    for (;; at++) {
      assert(at < 8);
      const uint8_t w = at % 4;
      ProtocolV2::ArmPayload p{{bootId, hostSession}, configId};
      handleHostFrame(ARM, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
      assert(armPending);
      simMotor[w].passRequests = at;
      simMotor[w].dropReplies = 1;
      loop();
      const bool reached = simMotor[w].dropReplies == 0;
      clearSimFaults(simMotor[w]);
      assert(motionState());
      assert(faultCode == NO_FAULT);
      assert(!unsafePositionCommands);
      if (!reached)
        break;
      runFor(30, true);
      expectReplyRetry(w);
      runFor(770, true);
      expectReplyRetry(w);
      runFor(300, true);
      expectReplyRetry(-1);
      assert(faultCode == NO_FAULT);
      handleHostFrame(STOP, ++seq, nullptr, 0);
      runFor(500);
      assert(state == DISARMED);
      assert(stopState == 3);
    }
    assert(at >= 2);
    return;
  }
  if (name == "arm_retry_bounded") {
    // A wheel that never answers during arming gets three info queries, no
    // more. The arm then fails as it always has and no current is commanded.
    boot();
    hello();
    configure();
    const uint8_t w = 2;
    for (bool garbled : {false, true}) {
      const unsigned queries = simMotor[w].queries,
                     commands = simMotor[w].commands;
      (garbled ? simMotor[w].corrupt : simMotor[w].absent) = true;
      ProtocolV2::ArmPayload p{{bootId, hostSession}, configId};
      handleHostFrame(ARM, ++seq, reinterpret_cast<uint8_t *>(&p), sizeof(p));
      assert(armPending);
      processArm();
      (garbled ? simMotor[w].corrupt : simMotor[w].absent) = false;
      assert(simMotor[w].queries == queries + kAttempts);
      assert(simMotor[w].commands == commands);
      assert(faultCode == (garbled ? BAD_MOTOR_FRAME : MOTOR_TIMEOUT));
      assert(faultWheel == w + 1);
      assert(stopPending);
      assert(!motionState());
      assert(!unsafePositionCommands);
      if (!garbled) {
        runFor(4500);
        assert(faultCode == NO_FAULT);
        assert(state == DISARMED);
      }
    }
    return;
  }
  if (name == "stop_single_drop") {
    // The stop sequence asks each motor for its mode, zeroes its current,
    // switches it to speed mode and verifies that: four requests per wheel. One
    // dropped reply at any of them is retried; the stop still completes.
    ready();
    unsigned at = 0;
    for (;; at++) {
      assert(at < 8);
      const uint8_t w = at % 4;
      runFor(400, true, 2000, 1);
      handleHostFrame(STOP, ++seq, nullptr, 0);
      assert(stopPending);
      simMotor[w].passRequests = at;
      simMotor[w].dropReplies = 1;
      loop();
      const bool reached = simMotor[w].dropReplies == 0;
      clearSimFaults(simMotor[w]);
      assert(faultCode == NO_FAULT);
      if (reached) {
        runFor(30);
        expectReplyRetry(w);
        runFor(470);
      } else
        runFor(500);
      assert(faultCode == NO_FAULT);
      assert(stopState == 3);
      assert(state == DISARMED);
      assert(simMotor[w].mode == 2);
      assert(std::fabs(simMotor[w].rpm) < 1);
      assert(simMotor[w].currentMa == 0);
      if (!reached)
        break;
      runFor(300);
      expectReplyRetry(w);
      runFor(300);
      expectReplyRetry(-1);
      arm();
    }
    assert(at >= 4);
    return;
  }
  if (name == "stop_retry_bounded") {
    // A wheel that never answers during a stop is asked twice, once for its
    // mode and once to verify speed mode, and each time three attempts, no
    // more. It never gets a command; the failure latches on that wheel.
    ready();
    const uint8_t w = 1;
    for (bool garbled : {false, true}) {
      runFor(300, true, 2000, 1);
      const unsigned queries = simMotor[w].queries,
                     commands = simMotor[w].commands;
      (garbled ? simMotor[w].corrupt : simMotor[w].absent) = true;
      handleHostFrame(STOP, ++seq, nullptr, 0);
      loop();
      (garbled ? simMotor[w].corrupt : simMotor[w].absent) = false;
      assert(simMotor[w].queries == queries + 2 * kAttempts);
      assert(simMotor[w].commands == commands);
      assert(faultCode == (garbled ? BAD_MOTOR_FRAME : MOTOR_TIMEOUT));
      assert(faultWheel == w + 1);
      assert(state == FAULT);
      for (int i = 0; i < 4; i++)
        if (i != w)
          assert(simMotor[i].mode == 2);
      if (!garbled) {
        runFor(4500);
        assert(faultCode == NO_FAULT);
        assert(state == DISARMED);
        arm();
      }
    }
    return;
  }
  if (name == "no_retry_definite_results") {
    // Only transport failures are retried. A stop that interrupts a command and
    // a reply from a motor in the wrong mode are answers, not misses.
    ready();
    sendTargets(3000);
    injectStopAfterWheel = 2;
    unsigned commands[4];
    for (int i = 0; i < 4; i++)
      commands[i] = simMotor[i].commands;
    previousSweepUs = micros() - 15000;
    controlSweep();
    assert(stopPending);
    assert(faultCode == NO_FAULT);
    assert(simMotor[0].commands == commands[0] + 1);
    assert(simMotor[1].commands == commands[1] + 1);
    assert(simMotor[2].commands == commands[2]);
    assert(simMotor[3].commands == commands[3]);
    runFor(500);
    assert(state == DISARMED);
    assert(stopState == 3);
    arm();
    runFor(300, true, 2000, 1);
    simMotor[2].mode = 2;
    const unsigned before = simMotor[2].commands;
    runOneSweep(2000, 1);
    assert(faultCode == MOTOR_FAULT);
    assert(faultWheel == 3);
    assert(simMotor[2].commands == before + 1);
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

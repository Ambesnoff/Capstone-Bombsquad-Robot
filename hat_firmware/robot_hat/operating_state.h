#pragma once
static uint32_t recoveryCoolSinceMs = 0;
static bool stopObserved[4] = {};
static void resetDrive() {
  memset(targetCentiRpm, 0, sizeof(targetCentiRpm));
  memset(stagedTargetCentiRpm, 0, sizeof(stagedTargetCentiRpm));
  haveTargets = false;
  // Discard the previous operator request together with motion targets.
  // The HAT-owned allowance and cooldown retain their existing history.
  requestedProfile = stagedProfile = appliedProfile = 0;
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    Wheel &w = wheel[i];
    w.rampRpm = w.integralRpmS = 0;
    w.lastCommandedMa = 0;
    w.stallSinceMs = w.currentSinceMs = w.reverseSinceMs = w.saturationSinceMs =
        0;
  }
}
static void performStop() {
  stopping = true;
  stopPending = false;
  armPending = false;
  disarmedHolding = false;
  stopState = 2;
  state = faultCode == NO_FAULT ? STOPPING : FAULT;
  resetDrive();
  stopVerificationPending = true;
  stopVerifyStartedMs = millis();
  memset(stopObserved, 0, sizeof(stopObserved));
  // Query actual interpretation before any zero setpoint. Stop every known
  // current/speed wheel promptly, then verify the speed mode for each wheel.
  uint8_t modes[4] = {};
  for (uint8_t id = 1; id <= 4; id++) {
    const MotorResult r = motorTransaction(id, true, 0, 0, false);
    if (r == MOTOR_OK)
      modes[id - 1] = wheel[id - 1].mode;
    else
      latchFault(motorFailure(r), id);
  }
  for (uint8_t id = 1; id <= 4; id++)
    if (modes[id - 1] == 1 || modes[id - 1] == 2) {
      const MotorResult r =
          motorTransaction(id, false, 0, modes[id - 1], false);
      if (r != MOTOR_OK)
        latchFault(motorFailure(r), id);
    }
  for (uint8_t id = 1; id <= 4; id++)
    sendMotorMode(id, 2);
  for (uint8_t id = 1; id <= 4; id++) {
    MotorResult r = motorTransaction(id, true, 0, 2, false);
    if (r == MOTOR_OK)
      r = motorTransaction(id, false, 0, 2, false);
    if (r != MOTOR_OK)
      latchFault(motorFailure(r), id);
  }
  nextDisarmedPollMs = 0;
  stopping = false;
  state = faultCode == NO_FAULT ? STOPPING : FAULT;
  completedProgress();
  sendStatus();
}
static bool selectCurrentSafely() {
  for (uint8_t id = 1; id <= 4; id++) {
    if (stopPending)
      return false;
    if (!sendMotorMode(id, 1)) {
      trip(MOTOR_TIMEOUT, id);
      return false;
    }
  }
  // Mode-write acknowledgement does not exist. All modes must be observed
  // before a single 0x64 zero command is issued during arming/holding entry.
  for (uint8_t id = 1; id <= 4; id++) {
    if (stopPending)
      return false;
    const MotorResult r = motorTransaction(id, true, 0, 1, true);
    if (r == MOTOR_INTERRUPTED)
      return false;
    if (r != MOTOR_OK) {
      trip(motorFailure(r), id);
      return false;
    }
    if (wheel[id - 1].error ||
        abs(static_cast<int>(wheel[id - 1].rpm)) > STATIONARY_RPM) {
      trip(MOTOR_FAULT, id);
      return false;
    }
  }
  for (uint8_t id = 1; id <= 4; id++) {
    const MotorResult r = sendCurrentMa(id, 0, 1, true);
    if (r == MOTOR_INTERRUPTED)
      return false;
    if (r != MOTOR_OK) {
      trip(motorFailure(r), id);
      return false;
    }
  }
  return true;
}
static void establishHold() {
  for (uint8_t i = 0; i < 4; i++) {
    wheel[i].holdAnchor = wheel[i].positionUnwrapped;
    wheel[i].holdIntegral = 0;
    wheel[i].integralRpmS = wheel[i].rampRpm = 0;
  }
  state = HOLDING;
}
static void processArm() {
  const ProtocolV2::ArmPayload identity = pendingArmIdentity;
  const uint16_t sequence = armSequence;
  armPending = false;
  if (!armRequestFresh(identity, sequence))
    return;
  const bool preservingHold = state == HOLDING && disarmedHolding;
  if (!preservingHold) {
    if (state != DISARMED || !stationaryFresh(true) || stopState != 3)
      return;
    state = STOPPING;
    if (!selectCurrentSafely())
      return;
  }
  // Motor transactions poll the host. Recheck the original request after
  // that IO so a newer command or session cannot be overwritten by this ARM.
  if (!armRequestFresh(identity, sequence)) {
    if (!preservingHold)
      requestStop();
    return;
  }
  // Preserve the anchor/integral/current when arming powered holding.
  disarmedHolding = false;
  state = preservingHold ? HOLDING : SETTLING;
  settleStartedMs = millis();
  stopState = 0;
  acceptCommand(sequence);
  markApplied(sequence);
  lastTargetMs = millis();
  haveTargets = true;
  memset(stagedTargetCentiRpm, 0, sizeof(stagedTargetCentiRpm));
  memset(targetCentiRpm, 0, sizeof(targetCentiRpm));
  stagedSequence = sequence;
  previousSweepUs = nextSweepUs = micros();
  completedProgress();
  sendStatus();
}
static bool allNeutral() {
  for (uint8_t i = 0; i < 4; i++)
    if (targetCentiRpm[i])
      return false;
  return true;
}
static bool allCurrentBelow(uint8_t rpm) {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < 4; i++)
    if (!wheel[i].valid || now - wheel[i].lastFeedbackMs > cfg.feedbackMs ||
        abs(static_cast<int>(wheel[i].rpm)) > rpm)
      return false;
  return true;
}
static bool allCurrentStationary() { return allCurrentBelow(STATIONARY_RPM); }
static void controlSweep() {
  const uint16_t sweepSequence = stagedSequence;
  const uint32_t began = micros();
  const float dt =
      clampf(static_cast<uint32_t>(began - previousSweepUs) / 1000000.0f,
             0.001f, 0.1f);
  previousSweepUs = began;
  if (!disarmedHolding) {
    if (millis() - lastTargetMs >= cfg.watchdogMs) {
      trip(COMMAND_TIMEOUT);
      return;
    }
    memcpy(targetCentiRpm, stagedTargetCentiRpm, sizeof(targetCentiRpm));
    requestedProfile = stagedProfile;
    if (!allNeutral()) {
      if (state != ARMED) {
        for (uint8_t i = 0; i < 4; i++) {
          wheel[i].integralRpmS = wheel[i].holdIntegral = 0;
          wheel[i].holdLimited = false;
          wheel[i].rampRpm = wheel[i].rpm;
        }
        state = ARMED;
      }
      settleStartedMs = 0;
    } else if (state == ARMED) {
      state = SETTLING;
      settleStartedMs = 0;
    }
    if (state == SETTLING) {
      if (allCurrentBelow(SETTLE_ENTRY_RPM)) {
        if (!settleStartedMs)
          settleStartedMs = millis() ? millis() : 1;
        if (millis() - settleStartedMs >= cfg.settleMs && cfg.holdEnabled)
          establishHold();
      } else
        settleStartedMs = 0;
    }
  }
  for (uint8_t i = 0; i < 4; i++) {
    pollHost();
    if (stopPending)
      return;
    const int16_t current = calculateCurrent(i, dt);
    if (stopPending)
      return;
    const MotorResult r = sendCurrentMa(i + 1, current, 1, true);
    if (r == MOTOR_INTERRUPTED)
      return;
    if (r != MOTOR_OK) {
      trip(motorFailure(r), i + 1);
      return;
    }
    wheel[i].lastCommandedMa = current;
    checkWheelHealth(i, current);
    if (stopPending)
      return;
  }
  for (uint8_t offset = 0; offset < 4; offset++) {
    const uint8_t i = (infoCursor + offset) % 4;
    if (wheel[i].temperatureValid &&
        millis() - wheel[i].lastInfoMs < cfg.tempPollMs)
      continue;
    const MotorResult r = motorTransaction(i + 1, true, 0, 1, true);
    if (r == MOTOR_INTERRUPTED)
      return;
    if (r != MOTOR_OK) {
      trip(motorFailure(r), i + 1);
      return;
    }
    checkWheelHealth(i, wheel[i].lastCommandedMa);
    infoCursor = (i + 1) % 4;
    break;
  }
  if (disarmedHolding)
    stopState = allCurrentStationary() ? 3 : 4;
  if (!disarmedHolding)
    markApplied(sweepSequence);
  lastSweepUs = micros() - began;
  completedProgress();
  sendStatus();
}
// First wheel whose stop is unobserved, stale, in the wrong mode or moving.
static uint8_t unsettledWheel() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    const Wheel &w = wheel[i];
    if (!stopObserved[i] || !w.valid || now - w.lastFeedbackMs > cfg.feedbackMs ||
        w.mode != 2 || abs(static_cast<int>(w.rpm)) > STATIONARY_RPM)
      return i + 1;
  }
  return 0;
}
static bool recoverableFault() {
  return !inspectionRequired && recoverableFaultCode(faultCode);
}
static void pollDisarmed() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - nextDisarmedPollMs) < 0)
    return;
  nextDisarmedPollMs = now + 100;
  for (uint8_t id = 1; id <= 4; id++) {
    // Existing faults never terminate this loop. Every wheel gets bounded
    // stop polls, including mode repair, independent of fault severity.
    MotorResult r = motorTransaction(id, true, 0, 0, true);
    if (r == MOTOR_INTERRUPTED)
      return;
    if (r == MOTOR_OK && wheel[id - 1].mode != 2) {
      sendMotorMode(id, 2);
      r = motorTransaction(id, true, 0, 2, true);
    }
    if (r == MOTOR_OK)
      r = motorTransaction(id, false, 0, 2, true);
    if (r == MOTOR_OK)
      stopObserved[id - 1] = true;
    else {
      latchFault(motorFailure(r), id);
      state = FAULT;
    }
  }
  bool observed = true;
  for (uint8_t i = 0; i < 4; i++)
    observed = observed && stopObserved[i];
  if (observed && stationaryFresh()) {
    stopVerificationPending = false;
    stopState = 3;
    if (faultCode == NO_FAULT)
      state = DISARMED;
  } else if (!stopVerificationPending) {
    // A confirmed stop that is later disturbed, such as a pushed wheel, gets a
    // new bounded verification window instead of an immediate fault.
    stopVerificationPending = true;
    stopVerifyStartedMs = millis();
    stopState = 2;
  } else if (millis() - stopVerifyStartedMs >= cfg.stopVerifyMs) {
    stopState = 4;
    if (faultCode == NO_FAULT) {
      latchFault(stationaryFresh() ? MOTOR_TIMEOUT : MOTOR_FAULT,
                 unsettledWheel());
      state = FAULT;
    }
  } else
    stopState = 2;
  // Only an overtemperature stop waits for the thermal release threshold;
  // command, feedback and stale-temperature faults recover on healthy feedback.
  bool ready = stationaryFresh(true);
  if (faultCode == OVERTEMPERATURE)
    for (uint8_t i = 0; i < 4; i++)
      if (wheel[i].tempC > cfg.tempReleaseC)
        ready = false;
  if (recoverableFault() && stopState == 3 && ready) {
    if (!recoveryCoolSinceMs)
      recoveryCoolSinceMs = millis() ? millis() : 1;
    if (millis() - recoveryCoolSinceMs >= cfg.cooldownMs) {
      faultCode = NO_FAULT;
      faultWheel = 0;
      state = DISARMED;
      haveTargets = false;
      resetDrive();
      recoveryCoolSinceMs = 0;
    }
  } else
    recoveryCoolSinceMs = 0;
  // Optional powered hold is explicitly motion-inhibited. It enters only
  // after confirmed stop, configured policy and healthy fresh measurements.
  if (state == DISARMED && configured && cfg.disarmedHoldEnabled &&
      stopState == 3 && stationaryFresh(true)) {
    state = STOPPING;
    if (selectCurrentSafely()) {
      disarmedHolding = true;
      establishHold();
      previousSweepUs = nextSweepUs = micros();
    }
  }
  completedProgress();
  sendStatus();
}

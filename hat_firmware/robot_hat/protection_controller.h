#pragma once
// One protection envelope is used by driving, settling, holding and recovery.
static float profileCap(uint8_t profile) {
  return profile == 2 ? cfg.boostMa
                      : (profile == 1 ? cfg.normalMa : cfg.gentleMa);
}
static void protectionUpdate(float dt) {
  const uint32_t now = millis();
  bool cool = true, boostSafe = faultCode == NO_FAULT;
  reasonFlags = 0;
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    Wheel &w = wheel[i];
    w.reason = 0;
    if (w.stallSinceMs)
      w.reason |= R_STALL_WARN;
    if (w.saturationSinceMs &&
        now - w.saturationSinceMs >= cfg.saturationWarnMs)
      w.reason |= R_STALL_WARN | R_SPEED;
    if (w.holdLimited && state == HOLDING)
      w.reason |= R_HOLD_LIMITED;
    const bool fresh = w.valid && now - w.lastFeedbackMs <= cfg.feedbackMs;
    const bool tempFresh =
        w.temperatureValid && now - w.lastInfoMs <= cfg.tempFreshMs;
    if (!fresh) {
      w.reason |= R_FEEDBACK_STALE;
      cool = false;
      boostSafe = false;
      if (currentState())
        trip(MOTOR_TIMEOUT, i + 1);
    }
    if (!tempFresh) {
      w.reason |= R_TEMP_STALE;
      cool = false;
      boostSafe = false;
      if (currentState())
        trip(TEMPERATURE_STALE, i + 1);
    }
    if (!w.temperatureValid || now - w.lastInfoMs > cfg.boostFreshMs) {
      w.reason |= R_TEMP_STALE;
      boostSafe = false;
    }
    if (w.error) {
      w.reason |= R_MOTOR_ERROR;
      cool = false;
      boostSafe = false;
      if (currentState())
        trip(MOTOR_FAULT, i + 1);
    }
    if (w.temperatureValid) {
      if (w.tempC >= cfg.tempWarnC)
        w.thermalWarning = true;
      else if (w.tempC <= cfg.tempWarnC - cfg.tempHysteresisC)
        w.thermalWarning = false;
      if (w.tempC >= cfg.tempDerateC)
        w.thermalDerating = true;
      else if (w.tempC <= cfg.tempDerateC - cfg.tempHysteresisC)
        w.thermalDerating = false;
      if (w.thermalWarning)
        w.reason |= R_TEMP_WARN;
      if (w.thermalDerating) {
        w.reason |= R_DERATE;
        boostSafe = false;
      }
      if (w.tempC >= cfg.tempLimitC) {
        w.reason |= R_THERMAL_STOP;
        boostSafe = false;
        cool = false;
        if (currentState())
          trip(OVERTEMPERATURE, i + 1);
      }
      if (w.tempC > cfg.tempReleaseC)
        cool = false;
    } else
      cool = false;
    reasonFlags |= w.reason;
  }
  if (cool && faultCode == NO_FAULT) {
    if (!coolSinceMs)
      coolSinceMs = now ? now : 1;
  } else
    coolSinceMs = 0;
  const bool cooldownDone = coolSinceMs && now - coolSinceMs >= cfg.cooldownMs;
  const bool appliedBoost = motionState() && requestedProfile == 2 &&
                            boostSafe && boostRemainingMs > 0;
  appliedProfile =
      requestedProfile == 2 && !appliedBoost ? 1 : requestedProfile;
  if (appliedBoost)
    boostRemainingMs = max(0.0f, boostRemainingMs - dt * 1000.0f);
  else if (requestedProfile != 2 && cooldownDone && faultCode == NO_FAULT)
    boostRemainingMs =
        min(static_cast<float>(cfg.boostCapacityMs),
            boostRemainingMs +
                dt * 1000.0f * cfg.boostCapacityMs / cfg.boostRefillMs);
  if (requestedProfile == 2 && boostRemainingMs <= 0) {
    appliedProfile = 1;
    reasonFlags |= R_BOOST_EMPTY;
  }
  if (!cooldownDone)
    reasonFlags |= R_COOLDOWN;
  uint32_t faultReason = 0;
  if (faultCode == COMMAND_TIMEOUT)
    faultReason = R_COMMAND;
  else if (faultCode == MOTOR_TIMEOUT || faultCode == BAD_MOTOR_FRAME)
    faultReason = R_FEEDBACK_STALE;
  else if (faultCode == OVERTEMPERATURE)
    faultReason = R_THERMAL_STOP;
  else if (faultCode == TEMPERATURE_STALE)
    faultReason = R_TEMP_STALE;
  else if (faultCode == STALL)
    faultReason = R_STALL;
  else if (faultCode == ABNORMAL_CURRENT)
    faultReason = R_CURRENT;
  else if (faultCode == MOTOR_FAULT)
    faultReason = R_MOTOR_ERROR;
  else if (faultCode == OVERSPEED)
    faultReason = R_SPEED;
  else if (faultCode == CONFIGURATION_FAULT)
    faultReason = R_CONFIG;
  reasonFlags |= faultReason;
  if (faultWheel >= 1 && faultWheel <= 4)
    wheel[faultWheel - 1].reason |= faultReason;
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    Wheel &w = wheel[i];
    float desired =
        min(profileCap(appliedProfile), static_cast<float>(cfg.maxCurrentMa));
    if (profileCap(appliedProfile) > cfg.maxCurrentMa)
      w.reason |= R_CEILING;
    float protectionCeiling = cfg.maxCurrentMa;
    if (w.thermalDerating) {
      const float fraction =
          clampf(static_cast<float>(cfg.tempLimitC - w.tempC) /
                     (cfg.tempLimitC - cfg.tempDerateC),
                 0, 1);
      protectionCeiling =
          min(protectionCeiling, static_cast<float>(cfg.gentleMa) * fraction);
      desired = min(desired, protectionCeiling);
    }
    if (!w.temperatureValid || now - w.lastInfoMs > cfg.tempFreshMs || w.error)
      protectionCeiling = 0;
    const float step = cfg.capRampMaS * dt;
    w.effectiveCapMa = desired > w.effectiveCapMa
                           ? min(desired, w.effectiveCapMa + step)
                           : max(desired, w.effectiveCapMa - step);
    w.effectiveCapMa = min(w.effectiveCapMa, protectionCeiling);
    w.protectionCapMa = protectionCeiling;
    w.holdCapMa = min(static_cast<float>(cfg.holdMa), protectionCeiling);
    if (w.temperatureValid && w.tempC >= cfg.holdTempC) {
      w.holdCapMa = 0;
      w.reason |= R_HOLD_LIMITED;
    }
    if (w.holdCapMa < cfg.holdMa)
      w.reason |= R_HOLD_LIMITED;
    reasonFlags |= w.reason;
  }
}
static float advanceRampRpm(float prior, float desired, float dt) {
  if (prior * desired < 0) {
    const float toZero = fabsf(prior) / cfg.decelRpmS;
    if (dt <= toZero) {
      const float remain = max(0.0f, fabsf(prior) - cfg.decelRpmS * dt);
      return prior > 0 ? remain : -remain;
    }
    const float step = cfg.accelRpmS * (dt - toZero);
    return desired > 0 ? min(desired, step) : max(desired, -step);
  }
  const float rate =
                  fabsf(desired) < fabsf(prior) ? cfg.decelRpmS : cfg.accelRpmS,
              step = rate * dt;
  return desired > prior ? min(desired, prior + step)
                         : max(desired, prior - step);
}
static float antiWindup(float pff, float error, float dt, float cap, float gain,
                        float &integral) {
  const float candidate = integral + error * dt,
              output = pff + gain * candidate;
  if (!((output > cap && error > 0) || (output < -cap && error < 0)))
    integral = candidate;
  integral = gain > 0 ? clampf(integral, -cap / gain, cap / gain) : 0;
  return clampf(pff + gain * integral, -cap, cap);
}
static int16_t calculateCurrent(uint8_t i, float dt) {
  Wheel &w = wheel[i];
  const bool hold = state == HOLDING;
  if (hold) {
    w.controlCapMa = w.holdCapMa;
    if (!w.positionValid || millis() - w.lastPositionMs > cfg.feedbackMs) {
      trip(MOTOR_TIMEOUT, i + 1);
      return 0;
    }
    const float error = (w.holdAnchor - w.positionUnwrapped) *
                        (360.0f / cfg.encoderCountsPerRev);
    const float pff = cfg.holdKp * error - cfg.holdDamping * w.rpm;
    const float output =
        antiWindup(pff, error, dt, w.holdCapMa, cfg.holdKi, w.holdIntegral);
    w.holdLimited =
        w.holdCapMa < cfg.holdMa ||
        (fabsf(output) >= max(1.0f, w.holdCapMa - 1) && fabsf(error) > 1);
    if (w.holdLimited) {
      w.reason |= R_HOLD_LIMITED;
      reasonFlags |= R_HOLD_LIMITED;
    }
    return static_cast<int16_t>(lroundf(output));
  }
  const float desired = targetCentiRpm[i] / 100.0f;
  float cap = w.effectiveCapMa;
  if (desired == 0)
    cap = min(static_cast<float>(cfg.neutralBrakeMa), w.protectionCapMa);
  cap = min(cap, w.protectionCapMa);
  w.controlCapMa = cap;
  // A zero inner wheel remains a driving wheel. Stop its stale speed ramp;
  // whole-chassis neutral is decided by the operating state, never here.
  if (desired == 0 && abs(static_cast<int>(w.rpm)) <= STATIONARY_RPM) {
    w.rampRpm = 0;
    w.integralRpmS = 0;
    return 0;
  }
  const float prior = w.rampRpm;
  w.rampRpm = advanceRampRpm(prior, desired, dt);
  const float error = w.rampRpm - w.rpm, accel = (w.rampRpm - prior) / dt;
  return static_cast<int16_t>(
      lroundf(antiWindup(cfg.kpMaPerRpm * error + cfg.ffMaPerRpmS * accel,
                         error, dt, cap, cfg.kiMaPerRpmS, w.integralRpmS)));
}
static void checkWheelHealth(uint8_t i, int16_t commandedMa) {
  Wheel &w = wheel[i];
  const uint32_t now = millis();
  if (w.error) {
    trip(MOTOR_FAULT, i + 1);
    return;
  }
  if (w.temperatureValid && w.tempC >= cfg.tempLimitC) {
    trip(OVERTEMPERATURE, i + 1);
    return;
  }
  if (abs(static_cast<int>(w.rpm)) >
      cfg.maxRpm + max(10, static_cast<int>(cfg.maxRpm / 4))) {
    trip(OVERSPEED, i + 1);
    return;
  }
  const bool reversing = static_cast<float>(targetCentiRpm[i]) * w.rampRpm < 0;
  const bool stall = cfg.stallEnabled && state != HOLDING && !reversing &&
                     fabsf(w.rampRpm) * 100 >= cfg.stallTargetCenti &&
                     abs(static_cast<int>(commandedMa)) >= cfg.stallCurrentMa &&
                     abs(static_cast<int>(w.rpm)) * 100 <= cfg.stallSpeedCenti;
  if (stall) {
    if (!w.stallSinceMs)
      w.stallSinceMs = now ? now : 1;
    w.reason |= R_STALL_WARN;
    if (now - w.stallSinceMs >= cfg.stallMs) {
      w.reason |= R_STALL;
      trip(STALL, i + 1);
      return;
    }
  } else
    w.stallSinceMs = 0;
  const bool reverse = abs(static_cast<int>(w.rpm)) >= 5 &&
                       fabsf(w.rampRpm) >= 5 && w.rpm * w.rampRpm < 0 &&
                       !reversing;
  if (reverse) {
    if (!w.reverseSinceMs)
      w.reverseSinceMs = now ? now : 1;
    if (now - w.reverseSinceMs >= cfg.stallMs) {
      trip(STALL, i + 1);
      return;
    }
  } else
    w.reverseSinceMs = 0;
  const int ceiling =
      min(static_cast<int>(cfg.abnormalCurrentMa),
          static_cast<int>(w.controlCapMa) + cfg.abnormalMarginMa);
  if (abs(static_cast<int>(w.currentMa)) > ceiling) {
    if (!w.currentSinceMs)
      w.currentSinceMs = now ? now : 1;
    if (now - w.currentSinceMs >= cfg.abnormalMs) {
      w.reason |= R_CURRENT;
      trip(ABNORMAL_CURRENT, i + 1);
      return;
    }
  } else
    w.currentSinceMs = 0;
  const float cap = w.controlCapMa;
  const bool saturated =
      cap > 0 && fabsf(static_cast<float>(commandedMa)) >= max(1.0f, cap - 2) &&
      fabsf(w.rampRpm - w.rpm) >= 5;
  if (saturated) {
    if (!w.saturationSinceMs)
      w.saturationSinceMs = now ? now : 1;
    if (now - w.saturationSinceMs >= cfg.saturationWarnMs)
      w.reason |= R_STALL_WARN | R_SPEED;
  } else
    w.saturationSinceMs = 0;
  reasonFlags |= w.reason;
}

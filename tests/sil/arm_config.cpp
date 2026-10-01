// Reuse the actual firmware, serial parser, and deterministic motor simulator.
#define main prior_native_main
#include "main.cpp"
#undef main

static void queueHostFrame(uint8_t type, uint16_t sequence, const void *payload,
                           uint8_t length) {
  std::vector<uint8_t> frame(9 + length);
  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = 2;
  frame[3] = type;
  put16(frame.data() + 4, sequence);
  frame[6] = length;
  if (length)
    memcpy(frame.data() + 7, payload, length);
  put16(frame.data() + 7 + length,
        crc16(frame.data() + 2, 5 + length));
  Serial.incoming.insert(Serial.incoming.end(), frame.begin(), frame.end());
}
static uint16_t queueArm(uint32_t id) {
  const ProtocolV2::ArmPayload payload{{bootId, hostSession}, id};
  const uint16_t sequence = ++seq;
  queueHostFrame(ARM, sequence, &payload, sizeof(payload));
  return sequence;
}
static uint16_t queueConfig(uint16_t maxRpm = 40) {
  ProtocolV2::ConfigPayload payload{};
  payload.session = {bootId, hostSession};
  payload.config = ProtocolV2::defaultConfig();
  payload.config.max_rpm = maxRpm;
  payload.config_id = configCrc32(
      reinterpret_cast<const uint8_t *>(&payload.config), sizeof(payload.config));
  const uint16_t sequence = ++seq;
  queueHostFrame(CONFIG, sequence, &payload, sizeof(payload));
  return sequence;
}
static uint16_t queueHello() {
  uint8_t nonce[8];
  put64(nonce, 0xABCDEF);
  const uint16_t sequence = ++seq;
  queueHostFrame(HELLO, sequence, nonce, sizeof(nonce));
  return sequence;
}
static void disarmedReady(bool wrap = false) {
  boot();
  hello();
  if (wrap) {
    seq = 65532;
    queueHello();
    loop();
  }
  configure();
  assert(state == DISARMED);
  assert(!armPending);
}
static void parseDuringDisarmedPolling() {
  // The first loop poll sees no bytes. The frames become readable during
  // motor IO, after the loop has already passed its pending-ARM check.
  Serial.ready = simTimeUs + 50;
  nextDisarmedPollMs = millis();
  loop();
  assert(Serial.incoming.empty());
  assert(hostRxLength == 0);
}
static void armThroughParser() {
  const uint16_t sequence = queueArm(configId);
  loop();
  assert(motionState());
  assert(lastAcceptedSequence == sequence);
  assert(lastAppliedSequence == sequence);
  assert(!freshCommand(sequence));
  assert(!unsafePositionCommands);
}
static void executeArmConfigCase(const std::string &name) {
  if (name == "arm_config" || name == "arm_config_wrap") {
    disarmedReady(name == "arm_config_wrap");
    const uint32_t originalConfig = configId;
    const uint16_t pendingSequence = queueArm(originalConfig);
    const uint16_t configSequence = queueConfig(100);
    parseDuringDisarmedPolling();
    assert(configResult == 1);
    assert(cfg.maxRpm == 100);
    assert(configId != originalConfig);
    assert(lastAcceptedSequence == configSequence);
    assert(lastAppliedSequence == configSequence);
    assert(!armPending);
    loop();
    assert(state == DISARMED);
    assert(!haveTargets);
    assert(lastAcceptedSequence == configSequence);
    assert(lastAppliedSequence == configSequence);
    assert(!freshCommand(configSequence));
    assert(!freshCommand(pendingSequence));
    // Even a fresh sequence cannot arm with the superseded configuration.
    queueArm(originalConfig);
    loop();
    assert(state == DISARMED);
    assert(!armPending);
    armThroughParser();
    return;
  }
  if (name == "arm_stop" || name == "arm_old_stop") {
    disarmedReady();
    const uint16_t originalBoundary = lastAcceptedSequence;
    queueArm(configId);
    const uint16_t stopSequence = name == "arm_old_stop" ? 0 : ++seq;
    queueHostFrame(STOP, stopSequence, nullptr, 0);
    parseDuringDisarmedPolling();
    assert(!armPending);
    loop();
    assert(state == DISARMED);
    assert(!haveTargets);
    assert(lastAcceptedSequence ==
           (name == "arm_old_stop" ? originalBoundary : stopSequence));
    armThroughParser();
    return;
  }
  if (name == "arm_session") {
    disarmedReady();
    const ProtocolV2::ArmPayload original{{bootId, hostSession}, configId};
    queueArm(configId);
    const uint16_t helloSequence = queueHello();
    parseDuringDisarmedPolling();
    assert(hostSession != original.session.host_session);
    assert(!configured);
    assert(!armPending);
    loop();
    assert(state == DISARMED);
    assert(lastAcceptedSequence == helloSequence);
    queueHostFrame(ARM, ++seq, &original, sizeof(original));
    loop();
    assert(!motionState());
    assert(!armPending);
    assert(lastAcceptedSequence == helloSequence);
    const uint16_t configSequence = queueConfig();
    loop();
    assert(configured);
    assert(lastAcceptedSequence == configSequence);
    armThroughParser();
    return;
  }
  if (name == "arm_io_stop" || name == "arm_io_session" ||
      name == "arm_io_config") {
    disarmedReady();
    const uint64_t originalSession = hostSession;
    const uint32_t originalConfig = configId;
    const uint16_t pendingSequence = queueArm(configId);
    pollHost();
    assert(armPending);
    const uint16_t appliedBefore = lastAppliedSequence;
    uint16_t interruptSequence;
    if (name == "arm_io_session")
      interruptSequence = queueHello();
    else if (name == "arm_io_config")
      interruptSequence = queueConfig(100);
    else {
      interruptSequence = ++seq;
      queueHostFrame(STOP, interruptSequence, nullptr, 0);
    }
    // Four 4 ms mode writes, four 1 ms queries, then four zero writes.
    // Arrive in the last zero write to exercise cancellation late in arming.
    Serial.ready = simTimeUs + 23500;
    loop();
    assert(Serial.incoming.empty());
    assert(!armPending);
    assert(!unsafePositionCommands);
    if (name == "arm_io_config") {
      // CONFIG is unavailable once mode selection has begun. The original
      // ARM remains bound to the original configuration and sequence.
      assert(configResult == 2);
      assert(configId == originalConfig);
      assert(cfg.maxRpm == 40);
      assert(motionState());
      assert(lastAcceptedSequence == pendingSequence);
      assert(lastAppliedSequence == pendingSequence);
      return;
    }
    assert(!motionState());
    assert(!haveTargets);
    assert(lastAcceptedSequence == interruptSequence);
    assert(lastAppliedSequence == appliedBefore);
    assert(!freshCommand(pendingSequence));
    runFor(300);
    assert(state == DISARMED);
    assert(stopState == 3);
    assert(lastAcceptedSequence == interruptSequence);
    if (name == "arm_io_session") {
      assert(hostSession != originalSession);
      assert(!configured);
      const uint16_t configSequence = queueConfig();
      loop();
      assert(configured);
      assert(lastAcceptedSequence == configSequence);
    }
    armThroughParser();
    return;
  }
  throw std::runtime_error("unknown ARM/config simulator case " + name);
}
int main(int argc, char **argv) {
  if (argc != 2)
    return 2;
  executeArmConfigCase(argv[1]);
  std::cout << "PASS " << argv[1] << " time_ms=" << millis() << "\n";
  return 0;
}

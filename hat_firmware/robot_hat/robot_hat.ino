// Four-wheel DDSM115 controller for Waveshare DDSM Driver HAT (A).
// ESP32 Arduino core 3.3.12; UART0 Pi, GPIO18/19 UART1 motor RS485.
#include "controller_types.h"
#include "host_protocol.h"
#include "motor_io.h"
#include "protection_controller.h"
#ifdef ARDUINO_ARCH_ESP32
#include <esp_system.h>
#include <esp_task_wdt.h>
static esp_task_wdt_user_handle_t progressWatchdog = nullptr;
static void completedProgress() {
  if (progressWatchdog)
    esp_task_wdt_reset_user(progressWatchdog);
}
static void startProgressWatchdog() {
  const esp_task_wdt_config_t settings = {
      .timeout_ms = 1000, .idle_core_mask = 0, .trigger_panic = true};
  const esp_err_t initialized = esp_task_wdt_init(&settings);
  if (initialized == ESP_ERR_INVALID_STATE)
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&settings));
  else
    ESP_ERROR_CHECK(initialized);
  ESP_ERROR_CHECK(
      esp_task_wdt_add_user("completed motor control", &progressWatchdog));
}
#else
static uint32_t nativeProgressMs = 0;
static void completedProgress() { nativeProgressMs = millis(); }
static void startProgressWatchdog() {}
#endif
#include "operating_state.h"
void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(PI_BAUD);
  Serial1.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  Serial.setRxFIFOFull(1);
  Serial1.setRxFIFOFull(1);
  delay(20);
  pollHost();
  drainMotorInput();
  hostRxLength = 0;
#ifdef ARDUINO_ARCH_ESP32
  bootId = (static_cast<uint64_t>(esp_random()) << 32) | esp_random();
  if (!bootId)
    bootId = 1;
#else
  bootId = 0x123456789abcdef0ULL;
#endif
  boostRemainingMs = 0;
  startProgressWatchdog();
  performStop();
  protectionPreviousMs = millis();
}
void loop() {
  pollHost();
  const uint32_t now = millis();
  const float elapsed =
      static_cast<uint32_t>(now - protectionPreviousMs) / 1000.0f;
  protectionPreviousMs = now;
  protectionUpdate(elapsed);
  if (stopPending) {
    performStop();
    return;
  }
  if (armPending) {
    processArm();
    return;
  }
  if (currentState()) {
    if (!disarmedHolding && millis() - lastTargetMs >= cfg.watchdogMs) {
      trip(COMMAND_TIMEOUT);
      return;
    }
    if (static_cast<int32_t>(micros() - nextSweepUs) >= 0) {
      const uint32_t began = micros();
      controlSweep();
      nextSweepUs = began + static_cast<uint32_t>(cfg.periodMs) * 1000;
    }
  } else
    pollDisarmed();
  if (millis() - lastStatusMs >= 100)
    sendStatus();
  delay(
      1); // yield to ESP32 system tasks; watchdog feed is completed work only.
}

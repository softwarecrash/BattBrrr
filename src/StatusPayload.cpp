#include "StatusPayload.h"

#include <ArduinoJson.h>
#include <WiFi.h>

#include "HeaterController.h"
#include "HeaterTypes.h"
#include "MqttBridge.h"
#include "SettingsPrefs.h"
#include "TempManager.h"
#include "WiFiManager.h"
#include "PidAutotune.h"

String buildStatusJson(const StatusContext& ctx) {
  JsonDocument doc;

  const uint32_t nowMs = millis();

  if (ctx.settings) {
    doc["deviceName"] = ctx.settings->get.deviceName();
  }
  doc["uptime_s"] = nowMs / 1000UL;

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  bool apMode = ctx.wifi ? ctx.wifi->isApMode() : false;
  wifi["mode"] = apMode ? "AP" : "STA";
  wifi["connected"] = (WiFi.status() == WL_CONNECTED);
  wifi["ip"] = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  wifi["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;

  JsonObject mqtt = doc["mqtt"].to<JsonObject>();
  if (ctx.mqtt) {
    mqtt["enabled"] = ctx.mqtt->isEnabled();
    mqtt["connected"] = ctx.mqtt->isConnected();
    mqtt["timed_out"] = ctx.mqtt->isTimedOut(nowMs);
    mqtt["last_rx_ms"] = ctx.mqtt->lastRxMs();
  }

  JsonObject temps = doc["temps"].to<JsonObject>();
  if (ctx.temps) {
    float t = NAN;
    bool valid = false;
    if (ctx.temps->getRoleTemp(SensorRole::BATTERY_PRIMARY, &t, &valid)) {
      if (valid) temps["primary_c"] = t;
      else temps["primary_c"] = nullptr;
      temps["primary_valid"] = valid;
    }
    if (ctx.temps->getRoleTemp(SensorRole::BATTERY_SECONDARY, &t, &valid)) {
      if (valid) temps["secondary_c"] = t;
      else temps["secondary_c"] = nullptr;
      temps["secondary_valid"] = valid;
    }
    if (ctx.temps->getRoleTemp(SensorRole::AMBIENT, &t, &valid)) {
      if (valid) temps["ambient_c"] = t;
      else temps["ambient_c"] = nullptr;
      temps["ambient_valid"] = valid;
    }

    JsonArray list = temps["sensors"].to<JsonArray>();
    for (const auto& sensor : ctx.temps->sensors()) {
      JsonObject o = list.add<JsonObject>();
      o["id"] = sensor.id;
      o["name"] = sensor.name;
      o["role"] = sensorRoleToString(sensor.role);
      o["offset_c"] = sensor.offsetC;
      o["present"] = sensor.present;
      o["valid"] = sensor.valid;
      if (sensor.valid) o["temp_c"] = sensor.tempC;
      else o["temp_c"] = nullptr;
      o["errors"] = sensor.errorTotal;
    }

    temps["last_update_ms"] = ctx.temps->lastUpdateMs();
    temps["last_scan_ms"] = ctx.temps->lastScanMs();
  }

  JsonObject controller = doc["controller"].to<JsonObject>();
  if (ctx.heater) {
    controller["enabled"] = ctx.heater->enabledEffective();
    controller["requested_mode"] = modeToString(ctx.heater->requestedMode());
    controller["mode"] = modeToString(ctx.heater->effectiveMode());
    controller["target_c"] = ctx.heater->targetC();
    controller["output_pct"] = ctx.heater->outputPct();
    controller["heater_on"] = ctx.heater->heaterOn();
    if (ctx.heater->controlTempValid()) {
      controller["control_temp_c"] = ctx.heater->controlTempC();
    } else {
      controller["control_temp_c"] = nullptr;
    }
    controller["control_temp_stale"] = ctx.heater->controlTempStale();
    controller["using_bms"] = ctx.heater->usingBmsFallback();
    const HeaterController::InputState inputs = ctx.heater->inputState();
    JsonObject input = controller["inputs"].to<JsonObject>();
    input["enable"] = inputs.enableActive;
    input["mode"] = inputs.modeActive;
    input["manual"] = inputs.manualActive;
  }

  JsonObject faults = doc["faults"].to<JsonObject>();
  if (ctx.heater) {
    const uint32_t latched = ctx.heater->faultMaskLatched();
    const uint32_t active = ctx.heater->faultMaskActive();
    JsonArray latchedArr = faults["latched"].to<JsonArray>();
    JsonArray activeArr = faults["active"].to<JsonArray>();

    for (uint8_t i = 0; i <= static_cast<uint8_t>(FaultCode::CONFIG_INVALID); ++i) {
      const FaultCode code = static_cast<FaultCode>(i);
      const uint32_t bit = faultBit(code);
      if (latched & bit) latchedArr.add(faultCodeToString(code));
      if (active & bit) activeArr.add(faultCodeToString(code));
    }

    faults["last_code"] = faultCodeToString(ctx.heater->lastFault());
    faults["last_ms"] = ctx.heater->lastFaultMs();
  }

  if (ctx.mqtt) {
    doc["last_bms_update_ms"] = ctx.mqtt->lastBmsUpdateMs();
  }

  if (ctx.autotune) {
    JsonDocument tuneDoc;
    const String tuneJson = ctx.autotune->buildStatusJson();
    if (!deserializeJson(tuneDoc, tuneJson)) {
      doc["autotune"] = tuneDoc.as<JsonVariant>();
    }
  }

  String out;
  serializeJson(doc, out);
  return out;
}

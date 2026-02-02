#include "MqttBridge.h"

#include <ArduinoJson.h>

#include "HeaterController.h"
#include "PidAutotune.h"
#include "StatusPayload.h"
#include "TempManager.h"

namespace {
constexpr uint32_t kReconnectIntervalMs = 3000;

void publishJsonFlat(PubSubClient& client, const String& rootTopic, JsonVariant v, bool retain, const String& path = String()) {
  if (v.is<JsonObject>()) {
    JsonObject obj = v.as<JsonObject>();
    for (JsonPair kv : obj) {
      String next = path.length() ? (path + "/" + kv.key().c_str()) : String(kv.key().c_str());
      publishJsonFlat(client, rootTopic, kv.value(), retain, next);
    }
    return;
  }
  if (v.is<JsonArray>()) {
    JsonArray arr = v.as<JsonArray>();
    for (size_t i = 0; i < arr.size(); ++i) {
      String next = path.length() ? (path + "/" + String(i)) : String(i);
      publishJsonFlat(client, rootTopic, arr[i], retain, next);
    }
    return;
  }

  String payload;
  if (v.is<const char*>()) {
    payload = v.as<const char*>();
  } else {
    serializeJson(v, payload);
  }

  String topic = rootTopic;
  if (path.length()) {
    topic += "/";
    topic += path;
  }
  client.publish(topic.c_str(), payload.c_str(), retain);
}
}  // namespace

MqttBridge::MqttBridge()
  : _settings(nullptr),
    _controller(nullptr),
    _temps(nullptr),
    _autotune(nullptr),
    _client(_net),
    _enabled(false),
    _port(1883),
    _keepaliveS(30),
    _publishIntervalS(5),
    _retain(false),
    _bmsTimeoutS(60),
    _lastConnectAttemptMs(0),
    _lastConnectedMs(0),
    _lastDisconnectMs(0),
    _lastPublishMs(0),
    _lastRxMs(0),
    _bmsModeValid(false),
    _bmsMode(ControlMode::IDLE),
    _bmsTempValid(false),
    _bmsTempC(NAN),
    _lastBmsStateUpdateMs(0),
    _lastBmsTempUpdateMs(0),
    _lastFaultReportedMs(0),
    _lastAutotuneResultId(0) {}

void MqttBridge::begin(Settings& settings, HeaterController& controller, TempManager& temps) {
  _settings = &settings;
  _controller = &controller;
  _temps = &temps;
  applySettings(settings);
  _client.setBufferSize(1024);
  _client.setCallback([this](char* topic, uint8_t* payload, unsigned int length) {
    handleMessage(topic, payload, length);
  });
}

void MqttBridge::setAutotune(PidAutotune* autotune) {
  _autotune = autotune;
}

void MqttBridge::applySettings(Settings& settings) {
  _enabled = settings.get.mqttEnable();
  _host = settings.get.mqttHost();
  _port = settings.get.mqttPort();
  _user = settings.get.mqttUser();
  _pass = settings.get.mqttPass();
  _clientId = settings.get.mqttClientId();
  _baseTopic = normalizeBaseTopic(settings.get.mqttBaseTopic());
  _keepaliveS = settings.get.mqttKeepaliveS();
  _publishIntervalS = settings.get.mqttPublishS();
  _retain = settings.get.mqttRetain();

  _bmsStateTopic = settings.get.bmsStateTopic();
  _bmsTempTopic = settings.get.bmsTempTopic();
  _bmsStatePath = settings.get.bmsStatePath();
  _bmsTempPath = settings.get.bmsTempPath();
  _bmsTimeoutS = settings.get.bmsTimeoutS();

  if (_enabled && _host.length()) {
    _client.setServer(_host.c_str(), _port);
    _client.setKeepAlive(_keepaliveS);
  }
  if (_client.connected()) {
    _client.disconnect();
  }
}

void MqttBridge::loop(uint32_t nowMs) {
  if (!_enabled || !_host.length()) {
    if (_client.connected()) {
      _client.disconnect();
    }
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!_client.connected()) {
    if (_lastConnectedMs != 0 && _lastDisconnectMs == 0) {
      _lastDisconnectMs = nowMs;
    }
    connectIfNeeded(nowMs);
  } else {
    _lastDisconnectMs = 0;
    _client.loop();
  }

  publishState(nowMs);
}

void MqttBridge::connectIfNeeded(uint32_t nowMs) {
  if (_client.connected()) return;
  if (_lastConnectAttemptMs != 0 && (nowMs - _lastConnectAttemptMs) < kReconnectIntervalMs) return;
  _lastConnectAttemptMs = nowMs;

  String cid = _clientId;
  if (!cid.length()) {
    cid = "battbrrr-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  }

  bool ok = false;
  if (_user.length()) {
    ok = _client.connect(cid.c_str(), _user.c_str(), _pass.c_str());
  } else {
    ok = _client.connect(cid.c_str());
  }

  if (ok) {
    _lastConnectedMs = nowMs;
    _lastDisconnectMs = 0;
    subscribeTopics();
  } else {
    if (_lastDisconnectMs == 0) _lastDisconnectMs = nowMs;
  }
}

void MqttBridge::subscribeTopics() {
  if (!_client.connected()) return;
  _client.subscribe(buildTopic("heater/cmd/enable").c_str());
  _client.subscribe(buildTopic("heater/cmd/mode").c_str());
  _client.subscribe(buildTopic("heater/cmd/target_idle").c_str());
  _client.subscribe(buildTopic("heater/cmd/target_charge").c_str());
  _client.subscribe(buildTopic("heater/cmd/target_discharge").c_str());
  _client.subscribe(buildTopic("heater/cmd/target_frost").c_str());
  _client.subscribe(buildTopic("heater/cmd/max_temp").c_str());
  _client.subscribe(buildTopic("heater/cmd/max_output").c_str());
  _client.subscribe(buildTopic("heater/cmd/reset_fault").c_str());
  _client.subscribe(buildTopic("heater/cmd/output_test").c_str());
  _client.subscribe(buildTopic("heater/cmd/autotune_start").c_str());
  _client.subscribe(buildTopic("heater/cmd/autotune_abort").c_str());
  _client.subscribe(buildTopic("heater/cmd/autotune_commit").c_str());

  if (_bmsStateTopic.length()) {
    _client.subscribe(_bmsStateTopic.c_str());
  }
  if (_bmsTempTopic.length()) {
    _client.subscribe(_bmsTempTopic.c_str());
  }
}

void MqttBridge::publishState(uint32_t nowMs) {
  if (!_client.connected()) return;
  if (_publishIntervalS == 0) return;
  if (_lastPublishMs != 0 && (nowMs - _lastPublishMs) < (_publishIntervalS * 1000UL)) return;

  StatusContext ctx = { _settings, _temps, _controller, this, nullptr, _autotune };
  String payload = buildStatusJson(ctx);
  _client.publish(buildTopic("heater/state").c_str(), payload.c_str(), _retain);
  JsonDocument doc;
  if (!deserializeJson(doc, payload)) {
    publishJsonFlat(_client, buildTopic("heater/state"), doc.as<JsonVariant>(), _retain);
  }
  _lastPublishMs = nowMs;

  if (_controller) {
    const uint32_t lastFaultMs = _controller->lastFaultMs();
    if (lastFaultMs != 0 && lastFaultMs != _lastFaultReportedMs) {
      _lastFaultReportedMs = lastFaultMs;
      String detail = faultCodeToString(_controller->lastFault());
      publishEvent("fault", detail);
    }
  }

  if (_autotune) {
    const String atState = _autotune->buildMqttStateJson();
    const String atProgress = _autotune->buildMqttProgressJson();
    _client.publish(buildTopic("heater/autotune/state").c_str(), atState.c_str(), _retain);
    _client.publish(buildTopic("heater/autotune/progress").c_str(), atProgress.c_str(), _retain);
    JsonDocument atDoc;
    if (!deserializeJson(atDoc, atState)) {
      publishJsonFlat(_client, buildTopic("heater/autotune/state"), atDoc.as<JsonVariant>(), _retain);
    }
    atDoc.clear();
    if (!deserializeJson(atDoc, atProgress)) {
      publishJsonFlat(_client, buildTopic("heater/autotune/progress"), atDoc.as<JsonVariant>(), _retain);
    }
    const uint32_t rid = _autotune->resultId();
    if (rid != _lastAutotuneResultId) {
      _lastAutotuneResultId = rid;
      const String atResult = _autotune->buildMqttResultJson();
      _client.publish(buildTopic("heater/autotune/result").c_str(), atResult.c_str(), _retain);
      atDoc.clear();
      if (!deserializeJson(atDoc, atResult)) {
        publishJsonFlat(_client, buildTopic("heater/autotune/result"), atDoc.as<JsonVariant>(), _retain);
      }
    }
  }
}

String MqttBridge::buildTopic(const char* suffix) const {
  if (!_baseTopic.length()) return String(suffix);
  return _baseTopic + "/" + suffix;
}

String MqttBridge::normalizeBaseTopic(const String& base) const {
  String out = base;
  out.trim();
  while (out.endsWith("/")) {
    out.remove(out.length() - 1);
  }
  while (out.startsWith("/")) {
    out.remove(0, 1);
  }
  return out;
}

void MqttBridge::handleMessage(char* topic, uint8_t* payload, unsigned int length) {
  _lastRxMs = millis();
  String topicStr = topic;
  String payloadStr;
  payloadStr.reserve(length + 1);
  for (unsigned int i = 0; i < length; ++i) {
    payloadStr += static_cast<char>(payload[i]);
  }
  payloadStr.trim();

  if (topicStr == _bmsStateTopic) {
    String extracted;
    if (extractJsonPath(payloadStr, _bmsStatePath, &extracted)) {
      ControlMode mode = modeFromPayload(extracted);
      if (mode != ControlMode::FAULT) {
        _bmsMode = mode;
        _bmsModeValid = true;
        _lastBmsStateUpdateMs = millis();
      } else {
        _bmsModeValid = false;
      }
    }
    return;
  }

  if (topicStr == _bmsTempTopic) {
    String extracted;
    if (extractJsonPath(payloadStr, _bmsTempPath, &extracted)) {
      float temp = NAN;
      if (parseFloat(extracted, &temp)) {
        _bmsTempC = temp;
        _bmsTempValid = true;
        _lastBmsTempUpdateMs = millis();
      } else {
        _bmsTempValid = false;
      }
    }
    return;
  }

  if (!_controller || !_settings) return;

  if (topicStr == buildTopic("heater/cmd/enable")) {
    bool val = false;
    if (parseBool(payloadStr, &val)) {
      _settings->set.enabled(val);
      _settings->save();
      _controller->applySettings(*_settings);
      publishEvent("enable", val ? "true" : "false");
    }
    return;
  }

  if (topicStr == buildTopic("heater/cmd/mode")) {
    ControlMode mode = modeFromPayload(payloadStr);
    if (mode != ControlMode::FAULT) {
      _settings->set.mode(static_cast<int32_t>(mode));
      _settings->save();
      _controller->applySettings(*_settings);
      publishEvent("mode", modeToString(mode));
    }
    return;
  }

  float fval = NAN;
  if (topicStr == buildTopic("heater/cmd/target_idle")) {
    if (parseFloat(payloadStr, &fval)) {
      _settings->set.targetIdleC(fval);
    }
  } else if (topicStr == buildTopic("heater/cmd/target_charge")) {
    if (parseFloat(payloadStr, &fval)) {
      _settings->set.targetChargeC(fval);
    }
  } else if (topicStr == buildTopic("heater/cmd/target_discharge")) {
    if (parseFloat(payloadStr, &fval)) {
      _settings->set.targetDischargeC(fval);
    }
  } else if (topicStr == buildTopic("heater/cmd/target_frost")) {
    if (parseFloat(payloadStr, &fval)) {
      _settings->set.targetFrostC(fval);
    }
  } else if (topicStr == buildTopic("heater/cmd/max_temp")) {
    if (parseFloat(payloadStr, &fval)) {
      _settings->set.maxTempC(fval);
    }
  } else if (topicStr == buildTopic("heater/cmd/max_output")) {
    if (parseFloat(payloadStr, &fval)) {
      _settings->set.maxOutputPct(fval);
    }
  } else if (topicStr == buildTopic("heater/cmd/reset_fault")) {
    _controller->requestFaultReset();
    publishEvent("fault_reset", "requested");
    return;
  } else if (topicStr == buildTopic("heater/cmd/output_test")) {
    JsonDocument doc;
    if (!deserializeJson(doc, payloadStr)) {
      float pct = doc["pct"] | 0.0f;
      uint32_t durationS = doc["duration_s"] | 0;
      if (durationS > 0) {
        _controller->startOutputTest(pct, durationS * 1000UL);
      }
    }
    return;
  } else if (_autotune && topicStr == buildTopic("heater/cmd/autotune_start")) {
    JsonDocument doc;
    bool autoSave = false;
    String aggr = "conservative";
    uint32_t maxDur = 0;
    if (!deserializeJson(doc, payloadStr)) {
      autoSave = doc["auto_save"] | false;
      aggr = doc["aggressiveness"] | "conservative";
      maxDur = doc["max_duration_s"] | 0;
    }
    _autotune->start(autoSave, PidAutotune::aggressivenessFromString(aggr), maxDur);
    publishEvent("autotune", "start");
    return;
  } else if (_autotune && topicStr == buildTopic("heater/cmd/autotune_abort")) {
    _autotune->abort();
    publishEvent("autotune", "abort");
    return;
  } else if (_autotune && topicStr == buildTopic("heater/cmd/autotune_commit")) {
    _autotune->commit();
    publishEvent("autotune", "commit");
    return;
  }

  if (!isnan(fval)) {
    _settings->save();
    _controller->applySettings(*_settings);
    return;
  }
}

bool MqttBridge::parseBool(const String& payload, bool* out) const {
  if (payload.equalsIgnoreCase("true") || payload == "1" || payload.equalsIgnoreCase("on")) {
    if (out) *out = true;
    return true;
  }
  if (payload.equalsIgnoreCase("false") || payload == "0" || payload.equalsIgnoreCase("off")) {
    if (out) *out = false;
    return true;
  }
  return false;
}

bool MqttBridge::parseFloat(const String& payload, float* out) const {
  char* endPtr = nullptr;
  float v = strtof(payload.c_str(), &endPtr);
  if (endPtr == payload.c_str()) return false;
  if (out) *out = v;
  return true;
}

bool MqttBridge::parseInt(const String& payload, int32_t* out) const {
  char* endPtr = nullptr;
  long v = strtol(payload.c_str(), &endPtr, 10);
  if (endPtr == payload.c_str()) return false;
  if (out) *out = static_cast<int32_t>(v);
  return true;
}

bool MqttBridge::extractJsonPath(const String& payload, const String& path, String* out) const {
  if (!path.length()) {
    if (out) *out = payload;
    return true;
  }
  JsonDocument doc;
  if (deserializeJson(doc, payload)) return false;

  JsonVariant current = doc.as<JsonVariant>();
  int start = 0;
  while (start < path.length()) {
    int dot = path.indexOf('.', start);
    String key = (dot >= 0) ? path.substring(start, dot) : path.substring(start);
    if (!current.is<JsonObject>()) return false;
    current = current[key];
    if (dot < 0) break;
    start = dot + 1;
  }

  if (current.isNull()) return false;
  if (current.is<const char*>()) {
    if (out) *out = String(current.as<const char*>());
    return true;
  }
  if (current.is<float>() || current.is<double>()) {
    if (out) *out = String(current.as<float>(), 3);
    return true;
  }
  if (current.is<int>() || current.is<long>() || current.is<uint32_t>()) {
    if (out) *out = String(current.as<long>());
    return true;
  }
  if (current.is<bool>()) {
    if (out) *out = current.as<bool>() ? "true" : "false";
    return true;
  }
  return false;
}

ControlMode MqttBridge::modeFromPayload(const String& payload) const {
  String v = payload;
  v.trim();
  v.toLowerCase();
  if (v == "charge" || v == "charging") return ControlMode::CHARGE;
  if (v == "discharge" || v == "discharging") return ControlMode::DISCHARGE;
  if (v == "idle" || v == "standby" || v == "stationary") return ControlMode::IDLE;
  if (v == "frost" || v == "frost_protect") return ControlMode::FROST_PROTECT;
  if (v == "manual") return ControlMode::MANUAL;
  if (v == "0") return ControlMode::IDLE;
  if (v == "1") return ControlMode::CHARGE;
  if (v == "2") return ControlMode::DISCHARGE;
  if (v == "3") return ControlMode::FROST_PROTECT;
  if (v == "4") return ControlMode::MANUAL;
  return ControlMode::FAULT;
}

bool MqttBridge::isEnabled() const {
  return _enabled;
}

bool MqttBridge::isConnected() {
  return _client.connected();
}

bool MqttBridge::isTimedOut(uint32_t nowMs) {
  if (!_enabled || !_host.length()) return false;
  if (_client.connected()) return false;
  if (_lastDisconnectMs == 0) return false;
  const uint32_t timeoutMs = static_cast<uint32_t>(_settings->get.mqttTimeoutS()) * 1000UL;
  return (nowMs - _lastDisconnectMs) > timeoutMs;
}

uint32_t MqttBridge::lastRxMs() const {
  return _lastRxMs;
}

uint32_t MqttBridge::lastConnectMs() const {
  return _lastConnectedMs;
}

bool MqttBridge::bmsTempValid(uint32_t nowMs) const {
  if (!_bmsTempTopic.length()) return false;
  if (!_bmsTempValid) return false;
  const uint32_t timeoutMs = static_cast<uint32_t>(_bmsTimeoutS) * 1000UL;
  return (nowMs - _lastBmsTempUpdateMs) <= timeoutMs;
}

float MqttBridge::bmsTempC() const {
  return _bmsTempC;
}

bool MqttBridge::bmsModeValid(uint32_t nowMs) const {
  if (!_bmsStateTopic.length()) return false;
  if (!_bmsModeValid) return false;
  const uint32_t timeoutMs = static_cast<uint32_t>(_bmsTimeoutS) * 1000UL;
  return (nowMs - _lastBmsStateUpdateMs) <= timeoutMs;
}

ControlMode MqttBridge::bmsMode() const {
  return _bmsMode;
}

uint32_t MqttBridge::lastBmsUpdateMs() const {
  return (_lastBmsStateUpdateMs > _lastBmsTempUpdateMs) ? _lastBmsStateUpdateMs : _lastBmsTempUpdateMs;
}

void MqttBridge::publishEvent(const String& type, const String& detail) {
  if (!_client.connected()) return;
  JsonDocument doc;
  doc["type"] = type;
  doc["detail"] = detail;
  doc["ts_ms"] = millis();
  String out;
  serializeJson(doc, out);
  _client.publish(buildTopic("heater/event").c_str(), out.c_str(), _retain);
  publishJsonFlat(_client, buildTopic("heater/event"), doc.as<JsonVariant>(), _retain);
}

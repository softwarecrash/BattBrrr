#include "TempManager.h"

#include <ArduinoJson.h>

#include "GpioValidator.h"
#include "WebSerial.h"

namespace {
constexpr uint32_t kConversionMs12bit = 750;
uint16_t conversionMsForResolution(uint8_t resBits) {
  switch (resBits) {
    case 9: return 94;
    case 10: return 188;
    case 11: return 375;
    case 12:
    default:
      return 750;
  }
}
}  // namespace

#if TEMP_DEBUG
#define TEMP_LOG(msg) do { webSerial.println(msg); } while (0)
#else
#define TEMP_LOG(msg) do { } while (0)
#endif

TempManager::TempManager()
  : _oneWirePin(-1),
    _pollIntervalMs(2000),
    _errorLimit(3),
    _rescanIntervalMin(10),
    _lastConversionStartMs(0),
    _lastUpdateMs(0),
    _lastScanMs(0),
    _conversionInFlight(false),
    _rescanPending(true),
    _conversionWaitMs(kConversionMs12bit) {}

void TempManager::begin(Settings& settings) {
  loadConfigFromJson(settings.get.sensorsJson());
  applySettings(settings);
}

void TempManager::applySettings(Settings& settings) {
  _pollIntervalMs = settings.get.sensorPollMs();
  _errorLimit = settings.get.sensorFailCount();
  _rescanIntervalMin = settings.get.sensorRescanMin();

  ensureBus(settings);
}

void TempManager::ensureBus(Settings& settings) {
  const int32_t pin = settings.get.oneWirePin();
  if (pin == _oneWirePin) return;

  _oneWirePin = pin;
  _dallas.reset();
  _oneWire.reset();

  if (_oneWirePin < 0) {
    TEMP_LOG("[TEMP] OneWire pin disabled");
    return;
  }

  if (!GpioValidator::isValidOutputPin(_oneWirePin)) {
    TEMP_LOG(String("[TEMP] Invalid OneWire pin: ") + String(_oneWirePin));
    return;
  }

  TEMP_LOG(String("[TEMP] Init OneWire on GPIO ") + String(_oneWirePin));
  _oneWire.reset(new OneWire(_oneWirePin));
  _dallas.reset(new DallasTemperature(_oneWire.get()));
  _dallas->begin();
  _dallas->setResolution(12);
  _conversionWaitMs = conversionMsForResolution(12);
  _dallas->setWaitForConversion(false);
  _dallas->setCheckForConversion(true);

  _rescanPending = true;
}

void TempManager::loop(uint32_t nowMs) {
  if (!_dallas) return;

  if (_rescanIntervalMin > 0) {
    const uint32_t intervalMs = static_cast<uint32_t>(_rescanIntervalMin) * 60000UL;
    if (_lastScanMs == 0 || (nowMs - _lastScanMs) >= intervalMs) {
      _rescanPending = true;
    }
  }

  if (_rescanPending) {
    _rescanPending = false;
    _lastScanMs = nowMs;
    std::vector<String> presentIds;
    const uint8_t count = _dallas->getDeviceCount();
    TEMP_LOG(String("[TEMP] Rescan -> found devices: ") + String(count));
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t addr[8] = {};
      if (_dallas->getAddress(addr, i)) {
        presentIds.push_back(addressToString(addr));
        TEMP_LOG(String("[TEMP] Device ") + String(i) + ": " + presentIds.back());
        bool known = false;
        for (auto& sensor : _sensors) {
          if (sensor.id == presentIds.back()) {
            known = true;
            sensor.present = true;
            break;
          }
        }
        if (!known) {
          Sensor sensor = {};
          memcpy(sensor.address, addr, sizeof(sensor.address));
          sensor.id = presentIds.back();
          sensor.name = sensor.id;
          sensor.role = SensorRole::UNUSED;
          sensor.offsetC = 0.0f;
          sensor.present = true;
          sensor.valid = false;
          sensor.tempC = NAN;
          sensor.errorStreak = 0;
          sensor.errorTotal = 0;
          sensor.lastReadMs = 0;
          _sensors.push_back(sensor);
        }
      }
    }
    updatePresence(presentIds);
  }

  if (_conversionInFlight) {
    const uint32_t elapsed = nowMs - _lastConversionStartMs;
    const bool done = _dallas->isConversionComplete();
    if (done || elapsed >= (_conversionWaitMs + 200)) {
      readSensors(nowMs);
      _conversionInFlight = false;
      _lastUpdateMs = nowMs;
    }
    return;
  }

  if (_lastUpdateMs == 0 || (nowMs - _lastUpdateMs) >= _pollIntervalMs) {
    startConversion(nowMs);
  }
}

void TempManager::startConversion(uint32_t nowMs) {
  if (!_dallas) return;
  _dallas->requestTemperatures();
  _lastConversionStartMs = nowMs;
  _conversionInFlight = true;
}

void TempManager::readSensors(uint32_t nowMs) {
  for (auto& sensor : _sensors) {
    if (!sensor.present) {
      sensor.valid = false;
      sensor.tempC = NAN;
      continue;
    }

    float temp = _dallas->getTempC(sensor.address);
    const bool ok = (temp > -126.0f) && (temp < 125.0f) && (temp != 85.0f);
    if (!ok) {
      sensor.errorStreak++;
      sensor.errorTotal++;
      if (sensor.errorStreak >= _errorLimit) {
        sensor.valid = false;
        sensor.tempC = NAN;
      }
    } else {
      sensor.errorStreak = 0;
      sensor.valid = true;
      sensor.tempC = temp + sensor.offsetC;
    }
    sensor.lastReadMs = nowMs;
  }
}

void TempManager::requestRescan() {
  _rescanPending = true;
}

bool TempManager::rescanNow(Settings& settings) {
  if (!_dallas) return false;
  _rescanPending = false;
  _lastScanMs = millis();

  std::vector<String> presentIds;
  const uint8_t count = _dallas->getDeviceCount();
  TEMP_LOG(String("[TEMP] Manual rescan -> found devices: ") + String(count));
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t addr[8] = {};
    if (_dallas->getAddress(addr, i)) {
      presentIds.push_back(addressToString(addr));
      TEMP_LOG(String("[TEMP] Device ") + String(i) + ": " + presentIds.back());
      bool known = false;
      for (auto& sensor : _sensors) {
        if (sensor.id == presentIds.back()) {
          known = true;
          sensor.present = true;
          break;
        }
      }
      if (!known) {
        Sensor sensor = {};
        memcpy(sensor.address, addr, sizeof(sensor.address));
        sensor.id = presentIds.back();
        sensor.name = sensor.id;
        sensor.role = SensorRole::UNUSED;
        sensor.offsetC = 0.0f;
        sensor.present = true;
        sensor.valid = false;
        sensor.tempC = NAN;
        sensor.errorStreak = 0;
        sensor.errorTotal = 0;
        sensor.lastReadMs = 0;
        _sensors.push_back(sensor);
      }
    }
  }

  updatePresence(presentIds);
  autoAssignPrimaryIfNeeded(settings);
  return true;
}

void TempManager::updatePresence(const std::vector<String>& presentIds) {
  for (auto& sensor : _sensors) {
    bool present = false;
    for (const auto& id : presentIds) {
      if (sensor.id == id) {
        present = true;
        break;
      }
    }
    sensor.present = present;
    if (!present) {
      sensor.valid = false;
      sensor.tempC = NAN;
    }
  }
}

void TempManager::autoAssignPrimaryIfNeeded(Settings& settings) {
  bool hasPrimary = false;
  for (const auto& sensor : _sensors) {
    if (sensor.role == SensorRole::BATTERY_PRIMARY) {
      hasPrimary = true;
      break;
    }
  }
  if (hasPrimary) return;
  for (auto& sensor : _sensors) {
    if (sensor.present) {
      sensor.role = SensorRole::BATTERY_PRIMARY;
      settings.set.sensorsJson(buildSensorsJson());
      settings.save();
      break;
    }
  }
}

uint32_t TempManager::lastUpdateMs() const {
  return _lastUpdateMs;
}

uint32_t TempManager::lastScanMs() const {
  return _lastScanMs;
}

const std::vector<TempManager::Sensor>& TempManager::sensors() const {
  return _sensors;
}

bool TempManager::getRoleTemp(SensorRole role, float* outTemp, bool* outValid, const Sensor** outSensor) const {
  for (const auto& sensor : _sensors) {
    if (sensor.role != role) continue;
    if (outTemp) *outTemp = sensor.tempC;
    if (outValid) *outValid = sensor.valid && sensor.present;
    if (outSensor) *outSensor = &sensor;
    return true;
  }
  if (outValid) *outValid = false;
  return false;
}

bool TempManager::hasRole(SensorRole role) const {
  for (const auto& sensor : _sensors) {
    if (sensor.role == role) return true;
  }
  return false;
}

void TempManager::applySensorOverrides(const String& json, Settings& settings) {
  const auto oldSensors = _sensors;
  loadConfigFromJson(json);
  for (auto& sensor : _sensors) {
    for (const auto& old : oldSensors) {
      if (sensor.id == old.id) {
        sensor.present = old.present;
        sensor.valid = old.valid;
        sensor.tempC = old.tempC;
        sensor.errorStreak = old.errorStreak;
        sensor.errorTotal = old.errorTotal;
        sensor.lastReadMs = old.lastReadMs;
        break;
      }
    }
  }
  settings.set.sensorsJson(json);
  requestRescan();
}

String TempManager::buildSensorsJson() const {
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (const auto& sensor : _sensors) {
    JsonObject obj = arr.add<JsonObject>();
    obj["id"] = sensor.id;
    obj["name"] = sensor.name;
    obj["role"] = sensorRoleToString(sensor.role);
    obj["offset_c"] = sensor.offsetC;
  }
  String out;
  serializeJson(doc, out);
  return out;
}

void TempManager::loadConfigFromJson(const String& json) {
  if (!json.length()) {
    _sensors.clear();
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) return;

  _sensors.clear();
  if (!doc.is<JsonArray>()) return;
  for (JsonObject obj : doc.as<JsonArray>()) {
    const char* id = obj["id"] | "";
    if (!id || !*id) continue;
    Sensor sensor = {};
    memset(sensor.address, 0, sizeof(sensor.address));
    sensor.id = id;
    sensor.name = obj["name"] | sensor.id;
    sensor.role = sensorRoleFromString(String(obj["role"] | ""));
    sensor.offsetC = obj["offset_c"] | 0.0f;
    sensor.present = false;
    sensor.valid = false;
    sensor.tempC = NAN;
    sensor.errorStreak = 0;
    sensor.errorTotal = 0;
    sensor.lastReadMs = 0;
    _sensors.push_back(sensor);
  }

  for (auto& sensor : _sensors) {
    parseAddress(sensor.id, sensor.address);
  }
}

bool TempManager::parseAddress(const String& id, uint8_t out[8]) const {
  if (id.length() != 16) return false;
  for (int i = 0; i < 8; ++i) {
    char buf[3] = {id[i * 2], id[i * 2 + 1], 0};
    char* endptr = nullptr;
    long v = strtol(buf, &endptr, 16);
    if (endptr == buf) return false;
    out[i] = static_cast<uint8_t>(v);
  }
  return true;
}

String TempManager::addressToString(const uint8_t address[8]) const {
  char buf[17] = {};
  for (int i = 0; i < 8; ++i) {
    snprintf(buf + (i * 2), sizeof(buf) - (i * 2), "%02X", address[i]);
  }
  return String(buf);
}

#pragma once

#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <memory>
#include <vector>

#include "HeaterTypes.h"
#include "SettingsPrefs.h"

class TempManager {
public:
  struct Sensor {
    uint8_t address[8];
    String id;
    String name;
    SensorRole role;
    float offsetC;
    bool present;
    bool valid;
    float tempC;
    uint32_t errorStreak;
    uint32_t errorTotal;
    uint32_t lastReadMs;
  };

  TempManager();

  void begin(Settings& settings);
  void applySettings(Settings& settings);
  void loop(uint32_t nowMs);

  void requestRescan();
  bool rescanNow(Settings& settings);

  uint32_t lastUpdateMs() const;
  uint32_t lastScanMs() const;

  const std::vector<Sensor>& sensors() const;

  bool getRoleTemp(SensorRole role, float* outTemp, bool* outValid, const Sensor** outSensor = nullptr) const;
  bool hasRole(SensorRole role) const;

  void applySensorOverrides(const String& json, Settings& settings);
  String buildSensorsJson() const;

private:
  void ensureBus(Settings& settings);
  void loadConfigFromJson(const String& json);
  bool parseAddress(const String& id, uint8_t out[8]) const;
  String addressToString(const uint8_t address[8]) const;
  void updatePresence(const std::vector<String>& presentIds);
  void autoAssignPrimaryIfNeeded(Settings& settings);
  void startConversion(uint32_t nowMs);
  void readSensors(uint32_t nowMs);

  int32_t _oneWirePin;
  uint32_t _pollIntervalMs;
  uint16_t _errorLimit;
  uint16_t _rescanIntervalMin;
  uint32_t _lastConversionStartMs;
  uint32_t _lastUpdateMs;
  uint32_t _lastScanMs;
  bool _conversionInFlight;
  bool _rescanPending;

  std::unique_ptr<OneWire> _oneWire;
  std::unique_ptr<DallasTemperature> _dallas;
  uint16_t _conversionWaitMs;
  std::vector<Sensor> _sensors;
};

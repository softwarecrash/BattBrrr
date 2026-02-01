#pragma once

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "HeaterTypes.h"
#include "SettingsPrefs.h"

class HeaterController;
class TempManager;
class PidAutotune;

class MqttBridge {
public:
  MqttBridge();

  void begin(Settings& settings, HeaterController& controller, TempManager& temps);
  void setAutotune(PidAutotune* autotune);
  void applySettings(Settings& settings);
  void loop(uint32_t nowMs);

  bool isEnabled() const;
  bool isConnected();
  bool isTimedOut(uint32_t nowMs);

  uint32_t lastRxMs() const;
  uint32_t lastConnectMs() const;

  bool bmsTempValid(uint32_t nowMs) const;
  float bmsTempC() const;
  bool bmsModeValid(uint32_t nowMs) const;
  ControlMode bmsMode() const;
  uint32_t lastBmsUpdateMs() const;

  void publishEvent(const String& type, const String& detail);

private:
  void connectIfNeeded(uint32_t nowMs);
  void handleMessage(char* topic, uint8_t* payload, unsigned int length);
  void subscribeTopics();
  void publishState(uint32_t nowMs);
  String buildTopic(const char* suffix) const;
  String normalizeBaseTopic(const String& base) const;

  bool parseBool(const String& payload, bool* out) const;
  bool parseFloat(const String& payload, float* out) const;
  bool parseInt(const String& payload, int32_t* out) const;
  bool extractJsonPath(const String& payload, const String& path, String* out) const;
  ControlMode modeFromPayload(const String& payload) const;

  Settings* _settings;
  HeaterController* _controller;
  TempManager* _temps;
  PidAutotune* _autotune;

  WiFiClient _net;
  PubSubClient _client;

  bool _enabled;
  String _host;
  uint16_t _port;
  String _user;
  String _pass;
  String _clientId;
  String _baseTopic;
  uint16_t _keepaliveS;
  uint16_t _publishIntervalS;
  bool _retain;

  String _bmsStateTopic;
  String _bmsTempTopic;
  String _bmsStatePath;
  String _bmsTempPath;
  uint16_t _bmsTimeoutS;

  uint32_t _lastConnectAttemptMs;
  uint32_t _lastConnectedMs;
  uint32_t _lastDisconnectMs;
  uint32_t _lastPublishMs;
  uint32_t _lastRxMs;

  bool _bmsModeValid;
  ControlMode _bmsMode;
  bool _bmsTempValid;
  float _bmsTempC;
  uint32_t _lastBmsStateUpdateMs;
  uint32_t _lastBmsTempUpdateMs;
  uint32_t _lastFaultReportedMs;
  uint32_t _lastAutotuneResultId;
};

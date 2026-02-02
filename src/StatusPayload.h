#pragma once

#include <Arduino.h>

class Settings;
class TempManager;
class HeaterController;
class MqttBridge;
class WiFiManager;
class PidAutotune;

struct StatusContext {
  Settings* settings;
  TempManager* temps;
  HeaterController* heater;
  MqttBridge* mqtt;
  WiFiManager* wifi;
  PidAutotune* autotune;
};

String buildStatusJson(const StatusContext& ctx);

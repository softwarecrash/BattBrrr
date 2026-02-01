#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

#include "HeaterController.h"
#include "MqttBridge.h"
#include "OtaManager.h"
#include "PidAutotune.h"
#include "SettingsPrefs.h"
#include "TempManager.h"
#include "WebSerial.h"
#include "WebServerHandler.h"
#include "WiFiManager.h"

Settings settings;
WiFiManager wifiManager;
AsyncWebServer server(80);
WebServerHandler web(server);
TempManager tempManager;
HeaterController heater;
MqttBridge mqtt;
OtaManager otaManager;
PidAutotune autotune;

void setup() {
#ifdef WSL_CUSTOM_PAGE
  webSerial.setCustomHtmlPage(webserialHtml(), webserialHtmlLen(), "gzip");
#endif
  webSerial.begin(&server, 115200, 200);

  settings.begin();
  webSerial.setAuthentication(settings.get.webUIuser(), settings.get.webUIPass());

  wifiManager.begin();
  tempManager.begin(settings);
  heater.begin(settings);
  mqtt.begin(settings, heater, tempManager);
  mqtt.setAutotune(&autotune);
  otaManager.begin();
  autotune.begin(settings, heater);
  web.begin();

  webSerial.println("[BOOT] BattBrrr Controller started");
}

void loop() {
  const uint32_t nowMs = millis();
  wifiManager.loop();
  tempManager.loop(nowMs);
  mqtt.loop(nowMs);
  autotune.loop(nowMs, tempManager);
  heater.loop(nowMs, tempManager, mqtt);
  otaManager.loop(nowMs);
}

#include "WebServerHandler.h"

#include <ArduinoJson.h>
#include <esp_timer.h>
#include <Update.h>

#include "HeaterController.h"
#include "MqttBridge.h"
#include "OtaManager.h"
#include "PidAutotune.h"
#include "SettingsPrefs.h"
#include "StatusPayload.h"
#include "TempManager.h"
#include "WiFiManager.h"
#include "WebSerial.h"
#include "www.h"

extern Settings settings;
extern WiFiManager wifiManager;
extern TempManager tempManager;
extern HeaterController heater;
extern MqttBridge mqtt;
extern OtaManager otaManager;
extern PidAutotune autotune;

static void scheduleRestart(uint32_t delayMs);

const uint8_t* webserialHtml() {
  return WebSerial_html_gz;
}

size_t webserialHtmlLen() {
  return WebSerial_html_gz_len;
}

// -------------------- Non-blocking WiFi scan cache --------------------
namespace NetScanCache {
  static bool scanRunning = false;
  static uint32_t cacheTs = 0;
  static String cacheJson;
  static const uint32_t CACHE_MS = 10000;

  static bool cacheValid() {
    if (cacheTs == 0) return false;
    return (millis() - cacheTs) < CACHE_MS && cacheJson.length() > 0;
  }

  static void startAsyncScanIfNeeded(bool force) {
    if (!force && cacheValid()) return;

    int sc = WiFi.scanComplete();
    if (sc == WIFI_SCAN_RUNNING) {
      scanRunning = true;
      return;
    }

    if (sc < 0) {
      int rc = WiFi.scanNetworks(true, true);
      scanRunning = (rc == WIFI_SCAN_RUNNING);
    } else {
      scanRunning = false;
    }
  }

  static void collectIfFinished() {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
      scanRunning = true;
      return;
    }
    if (n < 0) {
      scanRunning = false;
      return;
    }

    scanRunning = false;

    JsonDocument doc;
    JsonArray arr = doc["networks"].to<JsonArray>();

    for (int i = 0; i < n; i++) {
      JsonObject o = arr.add<JsonObject>();
      o["ssid"] = WiFi.SSID(i);
      o["rssi"] = WiFi.RSSI(i);
      o["enc"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
      o["bssid"] = WiFi.BSSIDstr(i);
    }

    WiFi.scanDelete();

    cacheJson = "";
    serializeJson(doc, cacheJson);
    cacheTs = millis();
  }

  static const String& json() {
    return cacheJson;
  }
}  // namespace NetScanCache

// -------------------- Restart scheduling (no delay in handlers) --------------------
static void bh_restart_cb(void* arg) {
  (void)arg;
  ESP.restart();
}

static void scheduleRestart(uint32_t delayMs) {
  esp_timer_handle_t t = nullptr;
  esp_timer_create_args_t args = {};
  args.callback = &bh_restart_cb;
  args.arg = nullptr;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "bh_restart";

  if (esp_timer_create(&args, &t) == ESP_OK && t) {
    esp_timer_start_once(t, static_cast<uint64_t>(delayMs) * 1000ULL);
  } else {
    ESP.restart();
  }
}

WebServerHandler::WebServerHandler(AsyncWebServer& s) : server(s) {}

bool WebServerHandler::isAuthorized(AsyncWebServerRequest* req) {
  if (!settings.get.webUIuser() || !*settings.get.webUIuser()) return true;
  return req->authenticate(settings.get.webUIuser(), settings.get.webUIPass());
}

void WebServerHandler::sendGz(AsyncWebServerRequest* req, const uint8_t* data, size_t len, const char* mime) {
  AsyncWebServerResponse* r = req->beginResponse(200, mime, data, len);
  r->addHeader("Content-Encoding", "gzip");
  r->addHeader("Cache-Control", "no-store");
  req->send(r);
}

void WebServerHandler::handleNetlist(AsyncWebServerRequest* req) {
  NetScanCache::startAsyncScanIfNeeded(false);
  NetScanCache::collectIfFinished();

  if (NetScanCache::cacheValid()) {
    req->send(200, "application/json", NetScanCache::json());
    return;
  }
  req->send(200, "application/json", "{\"networks\":[]}");
}

void WebServerHandler::handleConfigGet(AsyncWebServerRequest* req) {
  JsonDocument doc;

  doc["deviceName"] = settings.get.deviceName();
  doc["enabled"] = settings.get.enabled();
  doc["mode"] = settings.get.mode();
  doc["frostEnable"] = settings.get.frostEnable();

  doc["targetIdleC"] = settings.get.targetIdleC();
  doc["targetChargeC"] = settings.get.targetChargeC();
  doc["targetDischargeC"] = settings.get.targetDischargeC();
  doc["targetFrostC"] = settings.get.targetFrostC();

  doc["algorithm"] = settings.get.algorithm();
  doc["pidKp"] = settings.get.pidKp();
  doc["pidKi"] = settings.get.pidKi();
  doc["pidKd"] = settings.get.pidKd();
  doc["pidIntegralLimit"] = settings.get.pidIntegralLimit();
  doc["pidDerivFilter"] = settings.get.pidDerivFilter();
  doc["hystOnDelta"] = settings.get.hystOnDelta();
  doc["hystOffDelta"] = settings.get.hystOffDelta();
  doc["manualOutputPct"] = settings.get.manualOutputPct();

  doc["maxOutputPct"] = settings.get.maxOutputPct();
  doc["minOnMs"] = settings.get.minOnMs();
  doc["minOffMs"] = settings.get.minOffMs();
  doc["sensorPollMs"] = settings.get.sensorPollMs();
  doc["sensorFailCount"] = settings.get.sensorFailCount();
  doc["sensorRescanMin"] = settings.get.sensorRescanMin();

  doc["maxTempC"] = settings.get.maxTempC();
  doc["maxDeltaC"] = settings.get.maxDeltaC();
  doc["stuckOnPct"] = settings.get.stuckOnPct();
  doc["stuckOnS"] = settings.get.stuckOnS();
  doc["minRiseC"] = settings.get.minRiseC();
  doc["riseWindowS"] = settings.get.riseWindowS();
  doc["runawayEnable"] = settings.get.runawayEnable();
  doc["runawayRateCPerMin"] = settings.get.runawayRateCPerMin();
  doc["runawayWindowS"] = settings.get.runawayWindowS();
  doc["runawayMarginC"] = settings.get.runawayMarginC();
  doc["runawayLatch"] = settings.get.runawayLatch();

  doc["mqttLossMode"] = settings.get.mqttLossMode();
  doc["mqttTimeoutS"] = settings.get.mqttTimeoutS();

  doc["oneWirePin"] = settings.get.oneWirePin();
  doc["heaterOutPin"] = settings.get.heaterOutPin();
  doc["heaterOutInvert"] = settings.get.heaterOutInvert();
  doc["heaterOutType"] = settings.get.heaterOutType();
  doc["pwmFreq"] = settings.get.pwmFreq();
  doc["pwmResolution"] = settings.get.pwmResolution();
  doc["windowMs"] = settings.get.windowMs();

  doc["enableInPin"] = settings.get.enableInPin();
  doc["enableInPull"] = settings.get.enableInPull();
  doc["enableInActive"] = settings.get.enableInActive();
  doc["enableInDebounce"] = settings.get.enableInDebounce();

  doc["modeInPin"] = settings.get.modeInPin();
  doc["modeInPull"] = settings.get.modeInPull();
  doc["modeInActive"] = settings.get.modeInActive();
  doc["modeInDebounce"] = settings.get.modeInDebounce();

  doc["manualInPin"] = settings.get.manualInPin();
  doc["manualInPull"] = settings.get.manualInPull();
  doc["manualInActive"] = settings.get.manualInActive();
  doc["manualInDebounce"] = settings.get.manualInDebounce();

  doc["mqttEnable"] = settings.get.mqttEnable();
  doc["mqttHost"] = settings.get.mqttHost();
  doc["mqttPort"] = settings.get.mqttPort();
  doc["mqttUser"] = settings.get.mqttUser();
  doc["mqttPass"] = settings.get.mqttPass();
  doc["mqttClientId"] = settings.get.mqttClientId();
  doc["mqttBaseTopic"] = settings.get.mqttBaseTopic();
  doc["mqttKeepaliveS"] = settings.get.mqttKeepaliveS();
  doc["mqttPublishS"] = settings.get.mqttPublishS();
  doc["mqttRetain"] = settings.get.mqttRetain();

  doc["bmsStateTopic"] = settings.get.bmsStateTopic();
  doc["bmsTempTopic"] = settings.get.bmsTempTopic();
  doc["bmsStatePath"] = settings.get.bmsStatePath();
  doc["bmsTempPath"] = settings.get.bmsTempPath();
  doc["bmsTimeoutS"] = settings.get.bmsTimeoutS();
  doc["bmsFallback"] = settings.get.bmsFallback();

  JsonArray sensorsArr = doc["sensors"].to<JsonArray>();
  for (const auto& sensor : tempManager.sensors()) {
    JsonObject obj = sensorsArr.add<JsonObject>();
    obj["id"] = sensor.id;
    obj["name"] = sensor.name;
    obj["role"] = sensorRoleToString(sensor.role);
    obj["offset_c"] = sensor.offsetC;
    obj["present"] = sensor.present;
    obj["valid"] = sensor.valid;
    obj["temp_c"] = sensor.tempC;
  }

  String out;
  serializeJson(doc, out);
  req->send(200, "application/json", out);
}

void WebServerHandler::handleConfigPost(AsyncWebServerRequest* req, const String& body) {
  JsonDocument doc;
  if (deserializeJson(doc, body)) {
    req->send(400, "application/json", "{\"success\":false}");
    return;
  }

  auto oldDevice = String(settings.get.deviceName());
  auto oldSsid = String(settings.get.wifiSsid0());
  auto oldPass = String(settings.get.wifiPass0());

  #define APPLY_IF(KEY, STMT) \
    do { \
      JsonVariant v = doc[KEY]; \
      if (!v.isNull()) { \
        STMT; \
      } \
    } while (0)

  APPLY_IF("deviceName", settings.set.deviceName(v.as<String>()));
  APPLY_IF("enabled", settings.set.enabled(v.as<bool>()));
  APPLY_IF("mode", settings.set.mode(v.as<int32_t>()));
  APPLY_IF("frostEnable", settings.set.frostEnable(v.as<bool>()));

  APPLY_IF("targetIdleC", settings.set.targetIdleC(v.as<float>()));
  APPLY_IF("targetChargeC", settings.set.targetChargeC(v.as<float>()));
  APPLY_IF("targetDischargeC", settings.set.targetDischargeC(v.as<float>()));
  APPLY_IF("targetFrostC", settings.set.targetFrostC(v.as<float>()));

  APPLY_IF("algorithm", settings.set.algorithm(v.as<int32_t>()));
  APPLY_IF("pidKp", settings.set.pidKp(v.as<float>()));
  APPLY_IF("pidKi", settings.set.pidKi(v.as<float>()));
  APPLY_IF("pidKd", settings.set.pidKd(v.as<float>()));
  APPLY_IF("pidIntegralLimit", settings.set.pidIntegralLimit(v.as<float>()));
  APPLY_IF("pidDerivFilter", settings.set.pidDerivFilter(v.as<float>()));
  APPLY_IF("hystOnDelta", settings.set.hystOnDelta(v.as<float>()));
  APPLY_IF("hystOffDelta", settings.set.hystOffDelta(v.as<float>()));
  APPLY_IF("manualOutputPct", settings.set.manualOutputPct(v.as<float>()));

  APPLY_IF("maxOutputPct", settings.set.maxOutputPct(v.as<float>()));
  APPLY_IF("minOnMs", settings.set.minOnMs(v.as<uint32_t>()));
  APPLY_IF("minOffMs", settings.set.minOffMs(v.as<uint32_t>()));
  APPLY_IF("sensorPollMs", settings.set.sensorPollMs(v.as<uint32_t>()));
  APPLY_IF("sensorFailCount", settings.set.sensorFailCount(v.as<uint16_t>()));
  APPLY_IF("sensorRescanMin", settings.set.sensorRescanMin(v.as<uint16_t>()));

  APPLY_IF("maxTempC", settings.set.maxTempC(v.as<float>()));
  APPLY_IF("maxDeltaC", settings.set.maxDeltaC(v.as<float>()));
  APPLY_IF("stuckOnPct", settings.set.stuckOnPct(v.as<float>()));
  APPLY_IF("stuckOnS", settings.set.stuckOnS(v.as<uint32_t>()));
  APPLY_IF("minRiseC", settings.set.minRiseC(v.as<float>()));
  APPLY_IF("riseWindowS", settings.set.riseWindowS(v.as<uint32_t>()));
  APPLY_IF("runawayEnable", settings.set.runawayEnable(v.as<bool>()));
  APPLY_IF("runawayRateCPerMin", settings.set.runawayRateCPerMin(v.as<float>()));
  APPLY_IF("runawayWindowS", settings.set.runawayWindowS(v.as<uint32_t>()));
  APPLY_IF("runawayMarginC", settings.set.runawayMarginC(v.as<float>()));
  APPLY_IF("runawayLatch", settings.set.runawayLatch(v.as<bool>()));

  APPLY_IF("mqttLossMode", settings.set.mqttLossMode(v.as<int32_t>()));
  APPLY_IF("mqttTimeoutS", settings.set.mqttTimeoutS(v.as<uint16_t>()));

  APPLY_IF("oneWirePin", settings.set.oneWirePin(v.as<int32_t>()));
  APPLY_IF("heaterOutPin", settings.set.heaterOutPin(v.as<int32_t>()));
  APPLY_IF("heaterOutInvert", settings.set.heaterOutInvert(v.as<bool>()));
  APPLY_IF("heaterOutType", settings.set.heaterOutType(v.as<int32_t>()));
  APPLY_IF("pwmFreq", settings.set.pwmFreq(v.as<uint32_t>()));
  APPLY_IF("pwmResolution", settings.set.pwmResolution(v.as<uint16_t>()));
  APPLY_IF("windowMs", settings.set.windowMs(v.as<uint32_t>()));

  APPLY_IF("enableInPin", settings.set.enableInPin(v.as<int32_t>()));
  APPLY_IF("enableInPull", settings.set.enableInPull(v.as<int32_t>()));
  APPLY_IF("enableInActive", settings.set.enableInActive(v.as<int32_t>()));
  APPLY_IF("enableInDebounce", settings.set.enableInDebounce(v.as<uint16_t>()));

  APPLY_IF("modeInPin", settings.set.modeInPin(v.as<int32_t>()));
  APPLY_IF("modeInPull", settings.set.modeInPull(v.as<int32_t>()));
  APPLY_IF("modeInActive", settings.set.modeInActive(v.as<int32_t>()));
  APPLY_IF("modeInDebounce", settings.set.modeInDebounce(v.as<uint16_t>()));

  APPLY_IF("manualInPin", settings.set.manualInPin(v.as<int32_t>()));
  APPLY_IF("manualInPull", settings.set.manualInPull(v.as<int32_t>()));
  APPLY_IF("manualInActive", settings.set.manualInActive(v.as<int32_t>()));
  APPLY_IF("manualInDebounce", settings.set.manualInDebounce(v.as<uint16_t>()));

  APPLY_IF("mqttEnable", settings.set.mqttEnable(v.as<bool>()));
  APPLY_IF("mqttHost", settings.set.mqttHost(v.as<String>()));
  APPLY_IF("mqttPort", settings.set.mqttPort(v.as<uint16_t>()));
  APPLY_IF("mqttUser", settings.set.mqttUser(v.as<String>()));
  APPLY_IF("mqttPass", settings.set.mqttPass(v.as<String>()));
  APPLY_IF("mqttClientId", settings.set.mqttClientId(v.as<String>()));
  APPLY_IF("mqttBaseTopic", settings.set.mqttBaseTopic(v.as<String>()));
  APPLY_IF("mqttKeepaliveS", settings.set.mqttKeepaliveS(v.as<uint16_t>()));
  APPLY_IF("mqttPublishS", settings.set.mqttPublishS(v.as<uint16_t>()));
  APPLY_IF("mqttRetain", settings.set.mqttRetain(v.as<bool>()));

  APPLY_IF("bmsStateTopic", settings.set.bmsStateTopic(v.as<String>()));
  APPLY_IF("bmsTempTopic", settings.set.bmsTempTopic(v.as<String>()));
  APPLY_IF("bmsStatePath", settings.set.bmsStatePath(v.as<String>()));
  APPLY_IF("bmsTempPath", settings.set.bmsTempPath(v.as<String>()));
  APPLY_IF("bmsTimeoutS", settings.set.bmsTimeoutS(v.as<uint16_t>()));
  APPLY_IF("bmsFallback", settings.set.bmsFallback(v.as<bool>()));

  const JsonVariant sensorsVar = doc["sensors"];
  const bool hasSensors = !sensorsVar.isNull();
  if (hasSensors) {
    String sensorsOut;
    serializeJson(sensorsVar, sensorsOut);
    settings.set.sensorsJson(sensorsOut);
  }

  #undef APPLY_IF

  settings.save();
  if (hasSensors) {
    tempManager.applySensorOverrides(settings.get.sensorsJson(), settings);
  } else {
    tempManager.applySettings(settings);
  }
  heater.applySettings(settings);
  mqtt.applySettings(settings);

  req->send(200, "application/json", "{\"success\":true}");

  const bool networkChanged = (oldDevice != settings.get.deviceName()) ||
                              (oldSsid != settings.get.wifiSsid0()) ||
                              (oldPass != settings.get.wifiPass0());
  if (networkChanged) {
    scheduleRestart(1000);
  }
}

void WebServerHandler::handleSubmitNetConfig(AsyncWebServerRequest* req) {
  auto getP = [&](const char* name) -> String {
    if (!req->hasParam(name, true)) return "";
    return req->getParam(name, true)->value();
  };

  settings.set.deviceName(getP("devicename"));
  settings.set.wifiSsid0(getP("ssid0"));
  settings.set.wifiPass0(getP("password0"));
  settings.set.wifiBssid0(getP("bssid0"));
  const String bssidLock = getP("bssidLock");
  if (bssidLock.length()) {
    const bool lock = (bssidLock == "1" || bssidLock == "true" || bssidLock == "on");
    settings.set.wifiBssidLock(lock);
  }

  settings.set.wifiSsid1(getP("ssid1"));
  settings.set.wifiPass1(getP("password1"));

  settings.set.staticIP(getP("ip"));
  settings.set.staticSN(getP("subnet"));
  settings.set.staticGW(getP("gateway"));
  settings.set.staticDNS(getP("dns"));

  settings.set.webUIuser(getP("webUser"));
  settings.set.webUIPass(getP("webPass"));

  settings.save();

  req->send(200, "application/json", "{\"success\":true}");
  scheduleRestart(600);
}

void WebServerHandler::handleNetconfJson(AsyncWebServerRequest* req) {
  JsonDocument doc;
  doc["deviceName"] = settings.get.deviceName();
  doc["ssid0"] = settings.get.wifiSsid0();
  doc["pass0"] = settings.get.wifiPass0();
  doc["bssid0"] = settings.get.wifiBssid0();
  doc["bssidLock"] = settings.get.wifiBssidLock();
  doc["ssid1"] = settings.get.wifiSsid1();
  doc["pass1"] = settings.get.wifiPass1();
  doc["ip"] = settings.get.staticIP();
  doc["subnet"] = settings.get.staticSN();
  doc["gateway"] = settings.get.staticGW();
  doc["dns"] = settings.get.staticDNS();
  doc["webUser"] = settings.get.webUIuser();
  doc["webPass"] = settings.get.webUIPass();

  String out;
  serializeJson(doc, out);
  req->send(200, "application/json", out);
}

void WebServerHandler::handleStatusJson(AsyncWebServerRequest* req) {
  StatusContext ctx = { &settings, &tempManager, &heater, &mqtt, &wifiManager, &autotune };
  const String out = buildStatusJson(ctx);
  AsyncWebServerResponse* r = req->beginResponse(200, "application/json", out);
  r->addHeader("Cache-Control", "no-store");
  req->send(r);
}

void WebServerHandler::begin() {
  auto captivePortalResponse = [&](AsyncWebServerRequest* req) {
    if (wifiManager.isApMode()) {
      sendGz(req, WiFiSetup_html_gz, WiFiSetup_html_gz_len, WiFiSetup_html_gz_mime);
      return;
    }
    req->send(404, "text/plain", "Not found");
  };

  server.on("/", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (wifiManager.isApMode()) {
      req->redirect("/wifisetup");
      return;
    }
    if (!isAuthorized(req)) return req->requestAuthentication();
    sendGz(req, Status_html_gz, Status_html_gz_len, Status_html_gz_mime);
  });

  server.on("/config", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    sendGz(req, Config_html_gz, Config_html_gz_len, Config_html_gz_mime);
  });

  server.on("/wifisetup", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    NetScanCache::startAsyncScanIfNeeded(true);
    sendGz(req, WiFiSetup_html_gz, WiFiSetup_html_gz_len, WiFiSetup_html_gz_mime);
  });

  server.on("/ota", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    sendGz(req, Ota_html_gz, Ota_html_gz_len, Ota_html_gz_mime);
  });

  server.on("/autotune", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    sendGz(req, Autotune_html_gz, Autotune_html_gz_len, Autotune_html_gz_mime);
  });

  server.on("/generate_204", HTTP_GET, captivePortalResponse);
  server.on("/gen_204", HTTP_GET, captivePortalResponse);
  server.on("/hotspot-detect.html", HTTP_GET, captivePortalResponse);
  server.on("/library/test/success.html", HTTP_GET, captivePortalResponse);
  server.on("/ncsi.txt", HTTP_GET, captivePortalResponse);
  server.on("/connecttest.txt", HTTP_GET, captivePortalResponse);
  server.on("/fwlink", HTTP_GET, captivePortalResponse);

  server.on("/style.css", HTTP_GET, [&](AsyncWebServerRequest* req) {
    sendGz(req, Style_css_gz, Style_css_gz_len, Style_css_gz_mime);
  });
  server.on("/logo.svg", HTTP_GET, [&](AsyncWebServerRequest* req) {
    sendGz(req, logo_svg_gz, logo_svg_gz_len, logo_svg_gz_mime);
  });
  server.on("/favicon.ico", HTTP_GET, [&](AsyncWebServerRequest* req) {
    sendGz(req, logo_ico_gz, logo_ico_gz_len, logo_ico_gz_mime);
  });
  server.on("/backgroundCanvas.js", HTTP_GET, [&](AsyncWebServerRequest* req) {
    sendGz(req, backgroundCanvas_js_gz, backgroundCanvas_js_gz_len, backgroundCanvas_js_gz_mime);
  });
  server.on("/footer.js", HTTP_GET, [&](AsyncWebServerRequest* req) {
    sendGz(req, footer_js_gz, footer_js_gz_len, footer_js_gz_mime);
  });

  server.on("/netlist", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    handleNetlist(req);
  });

  server.on("/submitConfig", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    handleSubmitNetConfig(req);
  });

  server.on("/netconf.json", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    handleNetconfJson(req);
  });

  server.on("/status.json", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    handleStatusJson(req);
  });

  server.on("/info.json", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!isAuthorized(req)) return req->requestAuthentication();

    JsonDocument doc;
    doc["deviceName"] = settings.get.deviceName();
    doc["mode"] = wifiManager.isApMode() ? "AP" : "STA";
    doc["ip"] = wifiManager.isApMode() ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    doc["rssi"] = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
    doc["version"] = STRVERSION;

    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  server.on("/config.json", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    handleConfigGet(req);
  });

  server.on("/config", HTTP_POST,
    [&](AsyncWebServerRequest* req) {
      if (!wifiManager.isApMode()) {
        if (!isAuthorized(req)) {
          if (req->_tempObject) {
            delete (String*)req->_tempObject;
            req->_tempObject = nullptr;
          }
          return req->requestAuthentication();
        }
      }
      String* body = (String*)req->_tempObject;
      const String payload = body ? *body : "";
      if (body) {
        delete body;
        req->_tempObject = nullptr;
      }
      handleConfigPost(req, payload);
    },
    nullptr,
    [&](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (!wifiManager.isApMode() && !isAuthorized(req)) return;
      String* body = (String*)req->_tempObject;
      if (!body) {
        body = new String();
        body->reserve(total);
        req->_tempObject = body;
      }
      body->concat((const char*)data, len);
    }
  );

  server.on("/action/enable", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!isAuthorized(req)) return req->requestAuthentication();
    const bool enabled = req->hasParam("enabled", true) && req->getParam("enabled", true)->value() == "1";
    settings.set.enabled(enabled);
    settings.save();
    heater.applySettings(settings);
    req->send(200, "application/json", "{\"success\":true}");
  });

  server.on("/action/mode", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!isAuthorized(req)) return req->requestAuthentication();
    const String mode = req->hasParam("mode", true) ? req->getParam("mode", true)->value() : "";
    ControlMode m = modeFromString(mode, ControlMode::IDLE);
    settings.set.mode(static_cast<int32_t>(m));
    settings.save();
    heater.applySettings(settings);
    req->send(200, "application/json", "{\"success\":true}");
  });

  server.on("/action/reset_fault", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!isAuthorized(req)) return req->requestAuthentication();
    heater.requestFaultReset();
    req->send(200, "application/json", "{\"success\":true}");
  });

  server.on("/action/rescan", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!isAuthorized(req)) return req->requestAuthentication();
    tempManager.requestRescan();
    req->send(200, "application/json", "{\"success\":true}");
  });

  server.on("/action/output_test", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!isAuthorized(req)) return req->requestAuthentication();
    const float pct = req->hasParam("pct", true) ? req->getParam("pct", true)->value().toFloat() : 0.0f;
    const uint32_t durationS = req->hasParam("duration_s", true) ? req->getParam("duration_s", true)->value().toInt() : 0;
    bool ok = heater.startOutputTest(pct, durationS * 1000UL);
    req->send(ok ? 200 : 400, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
  });

  server.on("/config/backup", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    const bool pretty = req->hasParam("pretty");
    const String out = settings.backup(pretty);
    AsyncWebServerResponse* r = req->beginResponse(200, "application/json", out);
    r->addHeader("Content-Disposition", "attachment; filename=battbrrr-backup.json");
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
  });

  server.on("/config/restore", HTTP_POST,
    [&](AsyncWebServerRequest* req) {
      if (!wifiManager.isApMode()) {
        if (!isAuthorized(req)) {
          if (req->_tempObject) {
            delete (String*)req->_tempObject;
            req->_tempObject = nullptr;
          }
          return req->requestAuthentication();
        }
      }

      String* body = (String*)req->_tempObject;
      const bool ok = body && settings.restore(*body, true, true);
      if (body) {
        delete body;
        req->_tempObject = nullptr;
      }

      if (ok) {
        tempManager.applySettings(settings);
        heater.applySettings(settings);
        mqtt.applySettings(settings);
        req->send(200, "application/json", "{\"success\":true}");
        scheduleRestart(600);
      } else {
        req->send(400, "application/json", "{\"success\":false}");
      }
    },
    nullptr,
    [&](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (!wifiManager.isApMode() && !isAuthorized(req)) return;
      String* body = (String*)req->_tempObject;
      if (!body) {
        body = new String();
        body->reserve(total);
        req->_tempObject = body;
      }
      body->concat((const char*)data, len);
    }
  );

  server.on("/api/ota/upload", HTTP_POST,
    [&](AsyncWebServerRequest* req) {
      if (!wifiManager.isApMode()) {
        if (!isAuthorized(req)) return req->requestAuthentication();
      }
      const bool ok = !Update.hasError();
      req->send(ok ? 200 : 500, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
      if (ok) scheduleRestart(1200);
    },
    [&](AsyncWebServerRequest* req, String filename, size_t index, uint8_t* data, size_t len, bool final) {
      if (!wifiManager.isApMode() && !isAuthorized(req)) return;
      if (!index) {
        const uint32_t total = req->contentLength();
        if (!Update.begin(total ? total : UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
        }
      }
      if (Update.write(data, len) != len) {
        Update.printError(Serial);
      }
      if (final) {
        if (!Update.end(true)) {
          Update.printError(Serial);
        }
      }
    }
  );

  server.on("/api/ota/github/check", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    String error;
    const bool ok = otaManager.startGithubCheck(&error);
    req->send(ok ? 200 : 400, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
  });

  server.on("/api/ota/github/update", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    String error;
    const bool ok = otaManager.startGithubUpdate(&error);
    req->send(ok ? 200 : 400, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
  });

  server.on("/api/ota/github/status", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    req->send(200, "application/json", otaManager.buildGithubStatusJson());
  });

  server.on("/api/heater/autotune/status", HTTP_GET, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    req->send(200, "application/json", autotune.buildStatusJson());
  });

  server.on("/api/heater/autotune/start", HTTP_POST,
    [&](AsyncWebServerRequest* req) {
      if (!wifiManager.isApMode()) {
        if (!isAuthorized(req)) {
          if (req->_tempObject) {
            delete (String*)req->_tempObject;
            req->_tempObject = nullptr;
          }
          return req->requestAuthentication();
        }
      }
      String* body = (String*)req->_tempObject;
      const String payload = body ? *body : "";
      if (body) {
        delete body;
        req->_tempObject = nullptr;
      }
      JsonDocument doc;
      if (deserializeJson(doc, payload)) {
        req->send(400, "application/json", "{\"success\":false}");
        return;
      }
      const bool autoSave = doc["auto_save"] | false;
      const String aggr = doc["aggressiveness"] | "conservative";
      const uint32_t maxDur = doc["max_duration_s"] | 0;
      const bool ok = autotune.start(autoSave, PidAutotune::aggressivenessFromString(aggr), maxDur);
      req->send(ok ? 200 : 400, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
    },
    nullptr,
    [&](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (!wifiManager.isApMode() && !isAuthorized(req)) return;
      String* body = (String*)req->_tempObject;
      if (!body) {
        body = new String();
        body->reserve(total);
        req->_tempObject = body;
      }
      body->concat((const char*)data, len);
    }
  );

  server.on("/api/heater/autotune/abort", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    const bool ok = autotune.abort();
    req->send(ok ? 200 : 400, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
  });

  server.on("/api/heater/autotune/commit", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    const bool ok = autotune.commit();
    req->send(ok ? 200 : 400, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
  });

  server.on("/api/heater/autotune/discard", HTTP_POST, [&](AsyncWebServerRequest* req) {
    if (!wifiManager.isApMode()) {
      if (!isAuthorized(req)) return req->requestAuthentication();
    }
    const bool ok = autotune.discard();
    req->send(ok ? 200 : 400, "application/json", ok ? "{\"success\":true}" : "{\"success\":false}");
  });

  server.onNotFound([&](AsyncWebServerRequest* req) {
    if (wifiManager.isApMode()) {
      req->redirect("/wifisetup");
      return;
    }
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  webSerial.println("[WEB] Server started");
}

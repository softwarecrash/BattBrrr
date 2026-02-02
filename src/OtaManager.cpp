#include "OtaManager.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifndef OTA_GH_RELEASE_URL
#define OTA_GH_RELEASE_URL ""
#endif
#ifndef OTA_GH_ASSET_PATTERN
#define OTA_GH_ASSET_PATTERN "*.bin"
#endif

namespace {
constexpr uint32_t kHttpTimeoutMs = 20000;
constexpr uint32_t kCheckTaskStack = 8192;
constexpr uint32_t kUpdateTaskStack = 10240;
constexpr uint32_t kTaskPrio = 1;
}  // namespace

OtaManager::OtaManager()
  : _state(State::IDLE),
    _updateAvailable(false),
    _bytesTotal(0),
    _bytesDone(0),
    _progressPct(0),
    _lastUpdateMs(0),
    _lastCheckMs(0),
    _taskHandle(nullptr),
    _mux(portMUX_INITIALIZER_UNLOCKED) {}

void OtaManager::begin() {
  _cfg.releaseUrl = String(OTA_GH_RELEASE_URL);
  _cfg.assetPattern = String(OTA_GH_ASSET_PATTERN);
  _cfg.releaseUrl.trim();
  _cfg.assetPattern.trim();
}

void OtaManager::loop(uint32_t nowMs) {
  (void)nowMs;
}

bool OtaManager::isBusy() const {
  return _state == State::CHECKING || _state == State::DOWNLOADING || _state == State::APPLYING;
}

bool OtaManager::startGithubCheck(String* error) {
  if (isBusy()) {
    if (error) *error = "Busy";
    return false;
  }
  if (!_cfg.releaseUrl.length()) {
    if (error) *error = "Release URL missing";
    return false;
  }
  setState(State::CHECKING);
  _taskHandle = nullptr;
  BaseType_t ok = xTaskCreatePinnedToCore(
    &OtaManager::checkTaskThunk, "ota_check", kCheckTaskStack, this, kTaskPrio,
    reinterpret_cast<TaskHandle_t*>(&_taskHandle), 1);
  if (ok != pdPASS) {
    setError("Failed to start check task");
    setState(State::FAILED);
    return false;
  }
  return true;
}

bool OtaManager::startGithubUpdate(String* error) {
  if (isBusy()) {
    if (error) *error = "Busy";
    return false;
  }
  if (!_cfg.releaseUrl.length()) {
    if (error) *error = "Release URL missing";
    return false;
  }
  setState(State::DOWNLOADING);
  _taskHandle = nullptr;
  BaseType_t ok = xTaskCreatePinnedToCore(
    &OtaManager::updateTaskThunk, "ota_update", kUpdateTaskStack, this, kTaskPrio,
    reinterpret_cast<TaskHandle_t*>(&_taskHandle), 1);
  if (ok != pdPASS) {
    setError("Failed to start update task");
    setState(State::FAILED);
    return false;
  }
  return true;
}

String OtaManager::buildGithubStatusJson() const {
  ReleaseInfo rel;
  State state;
  String error;
  bool updateAvailable;
  uint32_t bytesTotal;
  uint32_t bytesDone;
  uint32_t progressPct;
  uint32_t lastUpdate;
  uint32_t lastCheck;

  portENTER_CRITICAL(&_mux);
  rel = _lastRelease;
  state = _state;
  error = _lastError;
  updateAvailable = _updateAvailable;
  bytesTotal = _bytesTotal;
  bytesDone = _bytesDone;
  progressPct = _progressPct;
  lastUpdate = _lastUpdateMs;
  lastCheck = _lastCheckMs;
  portEXIT_CRITICAL(&_mux);

  JsonDocument doc;
  doc["state"] = static_cast<int>(state);
  doc["state_str"] = (state == State::IDLE) ? "IDLE" :
                     (state == State::CHECKING) ? "CHECKING" :
                     (state == State::READY) ? "READY" :
                     (state == State::DOWNLOADING) ? "DOWNLOADING" :
                     (state == State::APPLYING) ? "APPLYING" :
                     (state == State::SUCCESS) ? "SUCCESS" :
                     "FAILED";
  doc["error"] = error;
  doc["update_available"] = updateAvailable;
  doc["bytes_total"] = bytesTotal;
  doc["bytes_done"] = bytesDone;
  doc["progress_pct"] = progressPct;
  doc["last_update_ms"] = lastUpdate;
  doc["last_check_ms"] = lastCheck;
  doc["current_version"] = STRVERSION;

  JsonObject release = doc["release"].to<JsonObject>();
  release["tag"] = rel.tag;
  release["name"] = rel.name;
  release["asset_name"] = rel.assetName;
  release["asset_size"] = rel.assetSize;
  release["prerelease"] = rel.prerelease;
  release["draft"] = rel.draft;
  if (rel.body.length()) {
    String snippet = rel.body;
    if (snippet.length() > 400) {
      snippet.remove(400);
      snippet += "...";
    }
    release["notes"] = snippet;
  } else {
    release["notes"] = "";
  }

  String out;
  serializeJson(doc, out);
  return out;
}

void OtaManager::checkTaskThunk(void* arg) {
  OtaManager* self = reinterpret_cast<OtaManager*>(arg);
  if (self) self->runCheckTask();
  vTaskDelete(nullptr);
}

void OtaManager::updateTaskThunk(void* arg) {
  OtaManager* self = reinterpret_cast<OtaManager*>(arg);
  if (self) self->runUpdateTask();
  vTaskDelete(nullptr);
}

void OtaManager::runCheckTask() {
  ReleaseInfo rel;
  String error;
  if (!fetchReleaseInfo(&rel, &error)) {
    setError(error.length() ? error : "Check failed");
    setState(State::FAILED);
    return;
  }

  portENTER_CRITICAL(&_mux);
  _lastRelease = rel;
  _updateAvailable = (rel.tag.length() && String(STRVERSION) != rel.tag);
  _lastCheckMs = millis();
  portEXIT_CRITICAL(&_mux);

  setState(State::READY);
}

void OtaManager::runUpdateTask() {
  ReleaseInfo rel;
  String error;
  if (_lastRelease.tag.length() == 0) {
    if (!fetchReleaseInfo(&rel, &error)) {
      setError(error.length() ? error : "Update check failed");
      setState(State::FAILED);
      return;
    }
  } else {
    portENTER_CRITICAL(&_mux);
    rel = _lastRelease;
    portEXIT_CRITICAL(&_mux);
  }

  if (!downloadAndUpdate(rel, &error)) {
    setError(error.length() ? error : "Update failed");
    setState(State::FAILED);
    return;
  }

  setState(State::SUCCESS);
  scheduleRestart(1200);
}

bool OtaManager::fetchReleaseInfo(ReleaseInfo* out, String* error) {
  if (!_cfg.releaseUrl.length()) {
    if (error) *error = "Release URL missing";
    return false;
  }

  const String url = _cfg.releaseUrl;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(kHttpTimeoutMs);
  http.begin(client, url);
  http.addHeader("Accept", "application/vnd.github+json");
  http.addHeader("User-Agent", "BattBrrr");

  const int code = http.GET();
  if (code != 200) {
    if (error) *error = "HTTP " + String(code);
    http.end();
    return false;
  }

  const String payload = http.getString();
  http.end();

  JsonDocument doc;
  if (deserializeJson(doc, payload)) {
    if (error) *error = "JSON parse error";
    return false;
  }

  JsonObject relObj;
  if (doc.is<JsonArray>()) {
    JsonArray arr = doc.as<JsonArray>();
    for (JsonVariant v : arr) {
      if (!v.is<JsonObject>()) continue;
      JsonObject o = v.as<JsonObject>();
      bool draft = o["draft"] | false;
      if (draft) continue;
      relObj = o;
      break;
    }
  } else if (doc.is<JsonObject>()) {
    relObj = doc.as<JsonObject>();
  } else {
    if (error) *error = "Release payload invalid";
    return false;
  }

  if (relObj.isNull()) {
    if (error) *error = "No release found";
    return false;
  }

  ReleaseInfo info;
  info.tag = relObj["tag_name"] | "";
  info.name = relObj["name"] | "";
  info.body = relObj["body"] | "";
  info.prerelease = relObj["prerelease"] | false;
  info.draft = relObj["draft"] | false;

  JsonArray assets = relObj["assets"].as<JsonArray>();
  if (assets.isNull() || assets.size() == 0) {
    if (error) *error = "No assets in release";
    return false;
  }

  String pattern = _cfg.assetPattern;
  pattern.trim();
  if (!pattern.length()) pattern = "*.bin";
  bool found = false;
  for (JsonVariant a : assets) {
    if (!a.is<JsonObject>()) continue;
    String name = a["name"] | "";
    if (!pattern.length()) {
      if (!name.endsWith(".bin")) continue;
    } else if (!matchPattern(name, pattern)) {
      continue;
    }
    info.assetName = name;
    info.assetUrl = a["browser_download_url"] | "";
    info.assetSize = a["size"] | 0;
    found = info.assetUrl.length() > 0;
    if (found) break;
  }

  if (!found) {
    if (error) *error = "No asset matched";
    return false;
  }

  if (out) *out = info;
  return true;
}

bool OtaManager::downloadAndUpdate(const ReleaseInfo& release, String* error) {
  if (!release.assetUrl.length()) {
    if (error) *error = "Missing asset URL";
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(kHttpTimeoutMs / 1000);

  HTTPClient http;
  http.setTimeout(kHttpTimeoutMs);
  http.begin(client, release.assetUrl);
  http.addHeader("User-Agent", "BattBrrr");
  int code = http.GET();
  if (code != 200) {
    if (error) *error = "HTTP " + String(code);
    http.end();
    return false;
  }

  const int total = http.getSize();
  if (release.assetSize && total > 0 && release.assetSize != (uint32_t)total) {
    if (error) *error = "Size mismatch";
    http.end();
    return false;
  }

  const uint32_t totalSize = (total > 0) ? (uint32_t)total : 0;
  portENTER_CRITICAL(&_mux);
  _bytesTotal = totalSize;
  _bytesDone = 0;
  _progressPct = 0;
  _lastUpdateMs = millis();
  portEXIT_CRITICAL(&_mux);

  if (!Update.begin(total > 0 ? total : UPDATE_SIZE_UNKNOWN)) {
    if (error) *error = "Update begin failed";
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[2048];
  uint32_t written = 0;
  setState(State::DOWNLOADING);

  while (http.connected() && (total > 0 ? (written < (uint32_t)total) : true)) {
    size_t avail = stream->available();
    if (avail) {
      const size_t toRead = (avail > sizeof(buf)) ? sizeof(buf) : avail;
      const size_t len = stream->readBytes(buf, toRead);
      if (len == 0) {
        vTaskDelay(1);
        continue;
      }
      if (Update.write(buf, len) != len) {
        if (error) *error = "Update write failed";
        Update.abort();
        http.end();
        return false;
      }
      written += len;
      portENTER_CRITICAL(&_mux);
      _bytesDone = written;
      if (_bytesTotal > 0) {
        _progressPct = (uint32_t)((written * 100UL) / _bytesTotal);
      }
      _lastUpdateMs = millis();
      portEXIT_CRITICAL(&_mux);
    } else {
      vTaskDelay(1);
    }
  }

  setState(State::APPLYING);
  const bool ok = Update.end(true);
  http.end();
  if (!ok) {
    if (error) *error = "Update end failed";
    return false;
  }

  return true;
}

static bool matchPatternInternal(const char* s, const char* p) {
  if (!p || !s) return false;
  if (*p == '\0') return *s == '\0';
  if (*p == '*') {
    return matchPatternInternal(s, p + 1) || (*s && matchPatternInternal(s + 1, p));
  }
  if (*p == '?') {
    return *s && matchPatternInternal(s + 1, p + 1);
  }
  char c1 = *s;
  char c2 = *p;
  if (c1 >= 'A' && c1 <= 'Z') c1 = c1 - 'A' + 'a';
  if (c2 >= 'A' && c2 <= 'Z') c2 = c2 - 'A' + 'a';
  return (c1 == c2) && matchPatternInternal(s + 1, p + 1);
}

bool OtaManager::matchPattern(const String& name, const String& pattern) const {
  const char* str = name.c_str();
  const char* pat = pattern.c_str();
  return matchPatternInternal(str, pat);
}

void OtaManager::setState(State state) {
  portENTER_CRITICAL(&_mux);
  _state = state;
  _lastUpdateMs = millis();
  portEXIT_CRITICAL(&_mux);
}

void OtaManager::setError(const String& error) {
  portENTER_CRITICAL(&_mux);
  _lastError = error;
  _lastUpdateMs = millis();
  portEXIT_CRITICAL(&_mux);
}

static void ota_restart_cb(void* arg) {
  (void)arg;
  ESP.restart();
}

void OtaManager::scheduleRestart(uint32_t delayMs) {
  esp_timer_handle_t t = nullptr;
  esp_timer_create_args_t args = {};
  args.callback = &ota_restart_cb;
  args.arg = nullptr;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "ota_restart";

  if (esp_timer_create(&args, &t) == ESP_OK && t) {
    esp_timer_start_once(t, static_cast<uint64_t>(delayMs) * 1000ULL);
  } else {
    ESP.restart();
  }
}

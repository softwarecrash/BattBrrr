#pragma once

#include <Arduino.h>

class OtaManager {
public:
  enum class State : uint8_t {
    IDLE = 0,
    CHECKING,
    READY,
    DOWNLOADING,
    APPLYING,
    SUCCESS,
    FAILED
  };

  struct GithubConfig {
    String releaseUrl;
    String assetPattern;
  };

  OtaManager();

  void begin();
  void loop(uint32_t nowMs);

  bool startGithubCheck(String* error);
  bool startGithubUpdate(String* error);
  bool isBusy() const;

  String buildGithubStatusJson() const;

private:
  struct ReleaseInfo {
    String tag;
    String name;
    String body;
    String assetName;
    String assetUrl;
    uint32_t assetSize;
    bool prerelease;
    bool draft;
  };

  static void checkTaskThunk(void* arg);
  static void updateTaskThunk(void* arg);
  void runCheckTask();
  void runUpdateTask();

  bool fetchReleaseInfo(ReleaseInfo* out, String* error);
  bool downloadAndUpdate(const ReleaseInfo& release, String* error);

  bool matchPattern(const String& name, const String& pattern) const;
  void setState(State state);
  void setError(const String& error);
  void scheduleRestart(uint32_t delayMs);

  GithubConfig _cfg;
  ReleaseInfo _lastRelease;

  State _state;
  String _lastError;
  bool _updateAvailable;
  uint32_t _bytesTotal;
  uint32_t _bytesDone;
  uint32_t _progressPct;
  uint32_t _lastUpdateMs;
  uint32_t _lastCheckMs;

  void* _taskHandle;
  mutable portMUX_TYPE _mux;
};

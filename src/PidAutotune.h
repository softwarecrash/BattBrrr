#pragma once

#include <Arduino.h>

class Settings;
class HeaterController;
class TempManager;

class PidAutotune {
public:
  enum class Phase : uint8_t {
    IDLE = 0,
    PROBE_RUNNING,
    TUNE_RUNNING,
    FINISHED,
    ABORTED,
    FAILED
  };

  enum class Aggressiveness : uint8_t {
    CONSERVATIVE = 0,
    NORMAL,
    AGGRESSIVE
  };

  enum class DetectedClass : uint8_t {
    UNKNOWN = 0,
    FAST,
    MEDIUM,
    SLOW
  };

  struct Result {
    float kp;
    float ki;
    float kd;
    float ku;
    float pu;
    float quality;
    String rule;
    bool valid;
  };

  PidAutotune();

  void begin(Settings& settings, HeaterController& heater);
  void loop(uint32_t nowMs, TempManager& temps);

  bool start(bool autoSave, Aggressiveness aggr, uint32_t maxDurationS);
  bool abort();
  bool commit();
  bool discard();

  Phase phase() const;
  bool isRunning() const;
  uint32_t lastUpdateMs() const;
  uint32_t resultId() const;
  const Result& result() const;

  String buildStatusJson() const;
  String buildMqttStateJson() const;
  String buildMqttProgressJson() const;
  String buildMqttResultJson() const;

  static Aggressiveness aggressivenessFromString(const String& value);
  static String aggressivenessToString(Aggressiveness aggr);
  static String phaseToString(Phase phase);
  static String classToString(DetectedClass cls);

private:
  struct Sample {
    uint32_t ms;
    float tempC;
  };

  struct Peak {
    uint32_t ms;
    float tempC;
  };

  void reset();
  void fail(const String& reason);
  void finish();

  void handleProbe(uint32_t nowMs, float tempC);
  void handleTune(uint32_t nowMs, float tempC);
  void pushSample(uint32_t nowMs, float tempC);
  bool computeRate(uint32_t windowMs, float* rate, float* delta, float* dtMin) const;

  void addMaxPeak(uint32_t ms, float tempC);
  void addMinPeak(uint32_t ms, float tempC);
  bool computeKuPu(float* ku, float* pu, float* amplitude, float* period, float* quality) const;

  void deriveTuneParameters(float rateCPerMin);
  void applyAggressiveness();
  void computePidFromKuPu(float ku, float pu);

  void setOverride(float targetC, float outputPct);
  void clearOverride();

  Settings* _settings;
  HeaterController* _heater;

  Phase _phase;
  Aggressiveness _aggr;
  DetectedClass _detected;

  bool _autoSave;
  bool _autoSaved;
  uint32_t _startMs;
  uint32_t _phaseStartMs;
  uint32_t _lastUpdateMs;
  uint32_t _maxDurationS;

  float _targetC;
  float _outputPct;
  float _noiseBandC;
  uint32_t _samplePeriodMs;
  uint8_t _requiredCycles;
  float _measuredRateCPerMin;
  String _lastError;

  // Probe parameters
  float _probeOutputPct;
  float _probeMaxOutputPct;
  uint32_t _probeWindowMs;
  uint32_t _probeWindowMaxMs;
  uint32_t _probeWindowStepMs;
  uint32_t _probeLastAdjustMs;
  uint32_t _probeSampleMs;
  float _probeMinRiseC;

  // Sampling
  static constexpr uint8_t kMaxSamples = 64;
  Sample _samples[kMaxSamples];
  uint8_t _sampleCount;
  uint8_t _sampleHead;
  uint32_t _lastSampleMs;

  // Peaks
  static constexpr uint8_t kMaxPeaks = 10;
  Peak _maxPeaks[kMaxPeaks];
  Peak _minPeaks[kMaxPeaks];
  uint8_t _maxCount;
  uint8_t _minCount;
  uint32_t _lastPeakMs;

  float _prevTemp;
  float _prevSlope;
  uint32_t _prevMs;
  bool _hasPrev;

  Result _result;
  uint32_t _resultId;
};

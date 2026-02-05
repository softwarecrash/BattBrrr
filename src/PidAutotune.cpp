#include "PidAutotune.h"

#include <ArduinoJson.h>
#include <algorithm>
#include <math.h>

#include "ControlProfile.h"
#include "HeaterController.h"
#include "SettingsPrefs.h"
#include "TempManager.h"

namespace {
constexpr float kProbeStepPct = 10.0f;
constexpr uint32_t kProbeWindowStartMs = 120000;
constexpr uint32_t kProbeWindowStepMs = 120000;
constexpr uint32_t kProbeWindowMaxMs = 600000;
constexpr uint32_t kProbeSampleMs = 10000;
constexpr float kProbeMinRiseC = 0.2f;

constexpr float kFastRate = 0.20f;
constexpr float kMediumRate = 0.05f;
constexpr float kFastRateCap = 0.60f;
constexpr float kSlowRateMin = 0.01f;

constexpr float kQualityThreshold = 0.55f;
}  // namespace

PidAutotune::PidAutotune()
  : _settings(nullptr),
    _heater(nullptr),
    _phase(Phase::IDLE),
    _aggr(Aggressiveness::CONSERVATIVE),
    _detected(DetectedClass::UNKNOWN),
    _autoSave(false),
    _autoSaved(false),
    _startMs(0),
    _phaseStartMs(0),
    _lastUpdateMs(0),
    _maxDurationS(0),
    _targetC(NAN),
    _outputPct(0.0f),
    _noiseBandC(0.25f),
    _samplePeriodMs(2000),
    _requiredCycles(6),
    _measuredRateCPerMin(NAN),
    _probeOutputPct(0.0f),
    _probeMaxOutputPct(0.0f),
    _probeWindowMs(kProbeWindowStartMs),
    _probeWindowMaxMs(kProbeWindowMaxMs),
    _probeWindowStepMs(kProbeWindowStepMs),
    _probeLastAdjustMs(0),
    _probeSampleMs(kProbeSampleMs),
    _probeMinRiseC(kProbeMinRiseC),
    _sampleCount(0),
    _sampleHead(0),
    _lastSampleMs(0),
    _maxCount(0),
    _minCount(0),
    _lastPeakMs(0),
    _prevTemp(NAN),
    _prevSlope(0.0f),
    _prevMs(0),
    _hasPrev(false),
    _resultId(0) {
  _result = {0, 0, 0, 0, 0, 0, "", false};
}

void PidAutotune::begin(Settings& settings, HeaterController& heater) {
  _settings = &settings;
  _heater = &heater;
  reset();
}

void PidAutotune::reset() {
  _phase = Phase::IDLE;
  _autoSave = false;
  _autoSaved = false;
  _detected = DetectedClass::UNKNOWN;
  _lastError = "";
  _result = {0, 0, 0, 0, 0, 0, "", false};
  _sampleCount = 0;
  _sampleHead = 0;
  _lastSampleMs = 0;
  _maxCount = 0;
  _minCount = 0;
  _lastPeakMs = 0;
  _prevTemp = NAN;
  _prevSlope = 0.0f;
  _prevMs = 0;
  _hasPrev = false;
  clearOverride();
}

PidAutotune::Phase PidAutotune::phase() const {
  return _phase;
}

bool PidAutotune::isRunning() const {
  return _phase == Phase::PROBE_RUNNING || _phase == Phase::TUNE_RUNNING;
}

uint32_t PidAutotune::lastUpdateMs() const {
  return _lastUpdateMs;
}

uint32_t PidAutotune::resultId() const {
  return _resultId;
}

const PidAutotune::Result& PidAutotune::result() const {
  return _result;
}

bool PidAutotune::start(bool autoSave, Aggressiveness aggr, uint32_t maxDurationS) {
  if (!_settings || !_heater) return false;
  if (isRunning()) return false;
  if (!_heater->enabledEffective()) return false;
  if (_heater->faultMaskActive() || _heater->faultMaskLatched()) return false;
  if (!_heater->controlTempValid()) return false;

  reset();

  _autoSave = autoSave;
  _aggr = aggr;
  _maxDurationS = maxDurationS;
  _phase = Phase::PROBE_RUNNING;
  _startMs = millis();
  _phaseStartMs = _startMs;
  _lastUpdateMs = _startMs;

  float tempC = _heater->controlTempC();
  float target = _heater->targetC();
  if (!isfinite(target)) target = tempC;
  if (tempC > target - 0.5f) target = tempC + 0.5f;
  float maxTarget = _settings->get.maxTempC() - 1.0f;
  if (target > maxTarget) target = maxTarget;
  _targetC = target;

  const float maxOut = _settings->get.maxOutputPct();
  _probeOutputPct = min(maxOut, ControlProfile::kHeatStartPct);
  _probeMaxOutputPct = maxOut;
  if (_probeMaxOutputPct < _probeOutputPct) {
    _probeMaxOutputPct = _probeOutputPct;
  }

  _probeWindowMs = kProbeWindowStartMs;
  _probeWindowMaxMs = kProbeWindowMaxMs;
  _probeWindowStepMs = kProbeWindowStepMs;
  _probeSampleMs = kProbeSampleMs;
  _probeMinRiseC = kProbeMinRiseC;
  _probeLastAdjustMs = _startMs;

  setOverride(_targetC, _probeOutputPct);
  return true;
}

bool PidAutotune::abort() {
  if (!isRunning()) return false;
  _phase = Phase::ABORTED;
  _lastError = "ABORTED";
  clearOverride();
  return true;
}

bool PidAutotune::commit() {
  if (!_settings || !_heater) return false;
  if (_phase != Phase::FINISHED || !_result.valid) return false;
  _settings->set.pidKp(_result.kp);
  _settings->set.pidKi(_result.ki);
  _settings->set.pidKd(_result.kd);
  _settings->set.algorithm(0);
  _settings->save();
  _heater->applySettings(*_settings);
  _autoSaved = true;
  return true;
}

bool PidAutotune::discard() {
  if (_phase != Phase::FINISHED && _phase != Phase::FAILED && _phase != Phase::ABORTED) return false;
  reset();
  return true;
}

void PidAutotune::loop(uint32_t nowMs, TempManager& temps) {
  (void)temps;
  if (_phase == Phase::IDLE || _phase == Phase::FINISHED || _phase == Phase::FAILED || _phase == Phase::ABORTED) {
    return;
  }
  _lastUpdateMs = nowMs;

  if (!_heater || !_settings) return;

  if (_heater->faultMaskActive() || _heater->faultMaskLatched()) {
    fail("SAFETY_FAULT");
    return;
  }
  if (!_heater->enabledEffective()) {
    fail("DISABLED");
    return;
  }
  if (!_heater->controlTempValid()) {
    fail("SENSOR_INVALID");
    return;
  }

  const float tempC = _heater->controlTempC();

  if (_phase == Phase::PROBE_RUNNING) {
    handleProbe(nowMs, tempC);
    return;
  }
  if (_phase == Phase::TUNE_RUNNING) {
    handleTune(nowMs, tempC);
  }
}

void PidAutotune::handleProbe(uint32_t nowMs, float tempC) {
  setOverride(_targetC, _probeOutputPct);

  if (_lastSampleMs != 0 && (nowMs - _lastSampleMs) < _probeSampleMs) return;
  _lastSampleMs = nowMs;
  pushSample(nowMs, tempC);

  float rate = NAN;
  float delta = 0.0f;
  float dtMin = 0.0f;
  const bool haveRate = computeRate(_probeWindowMs, &rate, &delta, &dtMin);

  if (haveRate && dtMin >= (_probeWindowMs / 60000.0f) * 0.7f) {
    if (delta >= _probeMinRiseC) {
      _measuredRateCPerMin = rate;
      deriveTuneParameters(rate);
      applyAggressiveness();
      _phase = Phase::TUNE_RUNNING;
      _phaseStartMs = nowMs;
      _sampleCount = 0;
      _sampleHead = 0;
      _maxCount = 0;
      _minCount = 0;
      _lastPeakMs = 0;
      _lastSampleMs = 0;
      _prevSlope = 0.0f;
      _hasPrev = false;
      return;
    }
  }

  if ((nowMs - _probeLastAdjustMs) >= _probeWindowMs) {
    if (_probeWindowMs < _probeWindowMaxMs) {
      _probeWindowMs = min(_probeWindowMs + _probeWindowStepMs, _probeWindowMaxMs);
      _probeLastAdjustMs = nowMs;
      _sampleCount = 0;
      _sampleHead = 0;
      return;
    }
    if ((_probeOutputPct + kProbeStepPct) <= _probeMaxOutputPct) {
      _probeOutputPct += kProbeStepPct;
      _probeLastAdjustMs = nowMs;
      _sampleCount = 0;
      _sampleHead = 0;
      return;
    }
    fail("INSUFFICIENT_RESPONSE");
  }
}

void PidAutotune::handleTune(uint32_t nowMs, float tempC) {
  const float high = _outputPct;
  const float low = 0.0f;

  float desired = _outputPct;
  if (tempC > (_targetC + _noiseBandC)) {
    desired = low;
  } else if (tempC < (_targetC - _noiseBandC)) {
    desired = high;
  }
  setOverride(_targetC, desired);

  if (_lastSampleMs != 0 && (nowMs - _lastSampleMs) < _samplePeriodMs) return;
  _lastSampleMs = nowMs;

  if (!_hasPrev) {
    _prevTemp = tempC;
    _prevMs = nowMs;
    _prevSlope = 0.0f;
    _hasPrev = true;
    return;
  }

  float slope = tempC - _prevTemp;
  const uint32_t minPeakDistance = std::max<uint32_t>(_samplePeriodMs * 3, 10000);
  if (_prevSlope > 0.0f && slope <= 0.0f) {
    if ((_prevMs - _lastPeakMs) >= minPeakDistance) {
      addMaxPeak(_prevMs, _prevTemp);
      _lastPeakMs = _prevMs;
    }
  } else if (_prevSlope < 0.0f && slope >= 0.0f) {
    if ((_prevMs - _lastPeakMs) >= minPeakDistance) {
      addMinPeak(_prevMs, _prevTemp);
      _lastPeakMs = _prevMs;
    }
  }

  if (fabsf(slope) > 0.0001f) {
    _prevSlope = slope;
  }
  _prevTemp = tempC;
  _prevMs = nowMs;

  float ku = NAN;
  float pu = NAN;
  float amp = NAN;
  float period = NAN;
  float quality = 0.0f;
  bool ok = computeKuPu(&ku, &pu, &amp, &period, &quality);

  const uint32_t elapsedS = (nowMs - _startMs) / 1000UL;
  if (_maxDurationS > 0 && elapsedS >= _maxDurationS) {
    if (ok) {
      _result.ku = ku;
      _result.pu = pu;
      _result.quality = quality * 100.0f;
      computePidFromKuPu(ku, pu);
    }
    fail("TIMEOUT");
    return;
  }

  uint8_t cycles = 0;
  if (_maxCount >= 2 && _minCount >= 2) {
    cycles = std::min<uint8_t>(_maxCount - 1, _minCount - 1);
  }

  if (ok && cycles >= _requiredCycles && quality >= kQualityThreshold) {
    _result.ku = ku;
    _result.pu = pu;
    _result.quality = quality * 100.0f;
    computePidFromKuPu(ku, pu);
    finish();
  }
}

void PidAutotune::pushSample(uint32_t nowMs, float tempC) {
  if (_sampleCount < kMaxSamples) {
    uint8_t idx = (_sampleHead + _sampleCount) % kMaxSamples;
    _samples[idx] = {nowMs, tempC};
    _sampleCount++;
  } else {
    _samples[_sampleHead] = {nowMs, tempC};
    _sampleHead = (_sampleHead + 1) % kMaxSamples;
  }

  while (_sampleCount > 1) {
    const Sample& oldest = _samples[_sampleHead];
    if ((nowMs - oldest.ms) <= _probeWindowMaxMs) break;
    _sampleHead = (_sampleHead + 1) % kMaxSamples;
    _sampleCount--;
  }
}

bool PidAutotune::computeRate(uint32_t windowMs, float* rate, float* delta, float* dtMin) const {
  if (_sampleCount < 2) return false;
  const uint8_t newestIdx = (_sampleHead + _sampleCount - 1) % kMaxSamples;
  const Sample& newest = _samples[newestIdx];

  int oldestIdx = -1;
  for (uint8_t i = 0; i < _sampleCount; ++i) {
    uint8_t idx = (_sampleHead + i) % kMaxSamples;
    if ((newest.ms - _samples[idx].ms) <= windowMs) {
      oldestIdx = idx;
      break;
    }
  }
  if (oldestIdx < 0) oldestIdx = _sampleHead;
  const Sample& oldest = _samples[oldestIdx];

  const float dt = (newest.ms - oldest.ms) / 60000.0f;
  if (dt <= 0.0f) return false;
  const float dT = newest.tempC - oldest.tempC;

  if (rate) *rate = dT / dt;
  if (delta) *delta = dT;
  if (dtMin) *dtMin = dt;
  return true;
}

void PidAutotune::addMaxPeak(uint32_t ms, float tempC) {
  if (_maxCount < kMaxPeaks) {
    _maxPeaks[_maxCount++] = {ms, tempC};
  } else {
    for (uint8_t i = 1; i < kMaxPeaks; ++i) _maxPeaks[i - 1] = _maxPeaks[i];
    _maxPeaks[kMaxPeaks - 1] = {ms, tempC};
  }
}

void PidAutotune::addMinPeak(uint32_t ms, float tempC) {
  if (_minCount < kMaxPeaks) {
    _minPeaks[_minCount++] = {ms, tempC};
  } else {
    for (uint8_t i = 1; i < kMaxPeaks; ++i) _minPeaks[i - 1] = _minPeaks[i];
    _minPeaks[kMaxPeaks - 1] = {ms, tempC};
  }
}

bool PidAutotune::computeKuPu(float* ku, float* pu, float* amplitude, float* period, float* quality) const {
  if (_maxCount < 2 || _minCount < 2) return false;
  const uint8_t cycles = std::min<uint8_t>(_maxCount - 1, _minCount - 1);
  const uint8_t use = std::min<uint8_t>(cycles, 3);
  if (use == 0) return false;

  float ampSum = 0.0f;
  float ampSq = 0.0f;
  float perSum = 0.0f;
  float perSq = 0.0f;

  const uint8_t pairCount = std::min<uint8_t>(_maxCount, _minCount);
  for (uint8_t i = 0; i < use; ++i) {
    const uint8_t idx = pairCount - use + i;
    const float a = fabsf(_maxPeaks[idx].tempC - _minPeaks[idx].tempC) * 0.5f;
    ampSum += a;
    ampSq += a * a;
  }

  for (uint8_t i = 0; i < use; ++i) {
    const uint8_t idx = _maxCount - use + i;
    const float p = (_maxPeaks[idx + 1].ms - _maxPeaks[idx].ms) / 1000.0f;
    perSum += p;
    perSq += p * p;
  }

  const float ampMean = ampSum / use;
  const float perMean = perSum / use;
  if (ampMean <= 0.0f || perMean <= 0.0f) return false;

  const float ampVar = (ampSq / use) - (ampMean * ampMean);
  const float perVar = (perSq / use) - (perMean * perMean);
  const float ampStd = ampVar > 0.0f ? sqrtf(ampVar) : 0.0f;
  const float perStd = perVar > 0.0f ? sqrtf(perVar) : 0.0f;
  const float ampRel = ampStd / ampMean;
  const float perRel = perStd / perMean;

  const float d = _outputPct * 0.5f;
  const float kuVal = (4.0f * d) / (3.1415926f * ampMean);

  if (ku) *ku = kuVal;
  if (pu) *pu = perMean;
  if (amplitude) *amplitude = ampMean;
  if (period) *period = perMean;
  if (quality) {
    const float worst = max(ampRel, perRel);
    float q = 1.0f - worst;
    if (q < 0.0f) q = 0.0f;
    if (q > 1.0f) q = 1.0f;
    *quality = q;
  }
  return true;
}

void PidAutotune::deriveTuneParameters(float rateCPerMin) {
  _measuredRateCPerMin = rateCPerMin;

  if (rateCPerMin >= kFastRate) {
    _detected = DetectedClass::FAST;
    float r = min(rateCPerMin, kFastRateCap);
    float t = (r - kFastRate) / (kFastRateCap - kFastRate);
    _samplePeriodMs = static_cast<uint32_t>(2000 - t * 1000);
    _noiseBandC = 0.25f - t * 0.10f;
    _outputPct = 25.0f - t * 15.0f;
    _requiredCycles = 7;
    if (_maxDurationS == 0) _maxDurationS = 3600;
  } else if (rateCPerMin >= kMediumRate) {
    _detected = DetectedClass::MEDIUM;
    float r = min(rateCPerMin, kFastRate);
    float t = (r - kMediumRate) / (kFastRate - kMediumRate);
    _samplePeriodMs = static_cast<uint32_t>(5000 - t * 3000);
    _noiseBandC = 0.40f - t * 0.15f;
    _outputPct = 40.0f - t * 20.0f;
    _requiredCycles = 6;
    if (_maxDurationS == 0) _maxDurationS = 5400;
  } else {
    _detected = DetectedClass::SLOW;
    float r = max(rateCPerMin, kSlowRateMin);
    float t = (r - kSlowRateMin) / (kMediumRate - kSlowRateMin);
    _samplePeriodMs = static_cast<uint32_t>(10000 - t * 5000);
    _noiseBandC = 0.60f - t * 0.20f;
    _outputPct = 60.0f - t * 30.0f;
    _requiredCycles = 5;
    if (_maxDurationS == 0) _maxDurationS = 9000;
  }
}

void PidAutotune::applyAggressiveness() {
  float clampPct = _settings ? _settings->get.maxOutputPct() : 100.0f;
  if (_aggr == Aggressiveness::CONSERVATIVE) {
    clampPct = min(clampPct, 40.0f);
    _noiseBandC *= 1.25f;
  } else if (_aggr == Aggressiveness::NORMAL) {
    clampPct = min(clampPct, 60.0f);
  } else {
    clampPct = min(clampPct, 80.0f);
    _noiseBandC *= 0.85f;
  }

  if (_measuredRateCPerMin >= 0.50f) {
    _outputPct = min(_outputPct, 15.0f);
  } else if (_measuredRateCPerMin >= 0.30f) {
    _outputPct = min(_outputPct, 20.0f);
  }

  if (_outputPct > clampPct) _outputPct = clampPct;
  if (_outputPct < 5.0f) _outputPct = 5.0f;
  if (_noiseBandC < 0.1f) _noiseBandC = 0.1f;
}

void PidAutotune::computePidFromKuPu(float ku, float pu) {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  String rule = "ZN";

  if (_aggr == Aggressiveness::CONSERVATIVE) {
    rule = "Tyreus-Luyben";
    kp = ku / 2.2f;
    const float ti = 2.2f * pu;
    const float td = pu / 6.3f;
    ki = kp / ti;
    kd = kp * td * 0.7f;
  } else if (_aggr == Aggressiveness::NORMAL) {
    rule = "Ziegler-Nichols";
    kp = 0.6f * ku;
    const float ti = 0.5f * pu;
    const float td = 0.125f * pu;
    ki = kp / ti;
    kd = kp * td;
  } else {
    rule = "Ziegler-Nichols (aggressive)";
    kp = 0.8f * ku;
    const float ti = 0.4f * pu;
    const float td = 0.15f * pu;
    ki = kp / ti;
    kd = kp * td;
  }

  _result.kp = kp;
  _result.ki = ki;
  _result.kd = kd;
  _result.rule = rule;
  _result.valid = true;
}

void PidAutotune::finish() {
  _phase = Phase::FINISHED;
  _resultId++;
  clearOverride();
  if (_autoSave && !_autoSaved) {
    commit();
  }
}

void PidAutotune::fail(const String& reason) {
  _phase = Phase::FAILED;
  _lastError = reason;
  clearOverride();
  _resultId++;
}

void PidAutotune::setOverride(float targetC, float outputPct) {
  if (_heater) {
    _heater->setExternalOverride(true, targetC, outputPct);
  }
}

void PidAutotune::clearOverride() {
  if (_heater) {
    _heater->setExternalOverride(false, 0.0f, 0.0f);
  }
}

String PidAutotune::phaseToString(Phase phase) {
  switch (phase) {
    case Phase::PROBE_RUNNING: return "PROBE_RUNNING";
    case Phase::TUNE_RUNNING: return "TUNE_RUNNING";
    case Phase::FINISHED: return "FINISHED";
    case Phase::ABORTED: return "ABORTED";
    case Phase::FAILED: return "FAILED";
    default: return "IDLE";
  }
}

String PidAutotune::classToString(DetectedClass cls) {
  switch (cls) {
    case DetectedClass::FAST: return "FAST";
    case DetectedClass::MEDIUM: return "MEDIUM";
    case DetectedClass::SLOW: return "SLOW";
    default: return "UNKNOWN";
  }
}

PidAutotune::Aggressiveness PidAutotune::aggressivenessFromString(const String& value) {
  String v = value;
  v.toLowerCase();
  if (v == "aggressive") return Aggressiveness::AGGRESSIVE;
  if (v == "normal") return Aggressiveness::NORMAL;
  return Aggressiveness::CONSERVATIVE;
}

String PidAutotune::aggressivenessToString(Aggressiveness aggr) {
  switch (aggr) {
    case Aggressiveness::AGGRESSIVE: return "aggressive";
    case Aggressiveness::NORMAL: return "normal";
    default: return "conservative";
  }
}

String PidAutotune::buildStatusJson() const {
  JsonDocument doc;
  doc["phase"] = phaseToString(_phase);
  doc["auto_save"] = _autoSave;
  doc["aggressiveness"] = aggressivenessToString(_aggr);
  doc["elapsed_s"] = _startMs ? (millis() - _startMs) / 1000UL : 0;
  doc["last_update_ms"] = _lastUpdateMs;
  doc["detected_class"] = classToString(_detected);
  doc["measured_rate_c_per_min"] = _measuredRateCPerMin;
  doc["target_c"] = _targetC;
  // During probe phase the active output comes from _probeOutputPct.
  doc["output_pct"] = (_phase == Phase::PROBE_RUNNING) ? _probeOutputPct : _outputPct;
  doc["noise_band_c"] = _noiseBandC;
  doc["sample_period_ms"] = _samplePeriodMs;
  doc["required_cycles"] = _requiredCycles;
  doc["max_duration_s"] = _maxDurationS;
  doc["last_error"] = _lastError;

  float temp = NAN;
  if (_heater && _heater->controlTempValid()) temp = _heater->controlTempC();
  if (isfinite(temp)) {
    doc["current_temp_c"] = temp;
  } else {
    doc["current_temp_c"] = nullptr;
  }

  JsonObject result = doc["result"].to<JsonObject>();
  if (_result.valid) {
    result["kp"] = _result.kp;
    result["ki"] = _result.ki;
    result["kd"] = _result.kd;
    result["ku"] = _result.ku;
    result["pu"] = _result.pu;
    result["quality"] = _result.quality;
    result["rule"] = _result.rule;
  }

  uint8_t cycles = 0;
  if (_maxCount >= 2 && _minCount >= 2) {
    cycles = std::min<uint8_t>(_maxCount - 1, _minCount - 1);
  }
  doc["cycles"] = cycles;

  uint32_t progress = 0;
  if (_phase == Phase::PROBE_RUNNING) {
    const uint32_t elapsed = millis() - _phaseStartMs;
    const uint32_t denom = _probeWindowMaxMs + _probeWindowStepMs;
    progress = std::min<uint32_t>(20, (denom > 0) ? (elapsed * 20UL) / denom : 0);
  } else if (_phase == Phase::TUNE_RUNNING) {
    if (_requiredCycles > 0) {
      progress = 20 + std::min<uint32_t>(80, (uint32_t)(cycles * 80UL) / _requiredCycles);
    } else {
      progress = 20;
    }
  } else if (_phase == Phase::FINISHED) {
    progress = 100;
  }
  doc["progress_pct"] = progress;

  String out;
  serializeJson(doc, out);
  return out;
}

String PidAutotune::buildMqttStateJson() const {
  JsonDocument doc;
  doc["phase"] = phaseToString(_phase);
  doc["auto_save"] = _autoSave;
  doc["aggressiveness"] = aggressivenessToString(_aggr);
  doc["detected_class"] = classToString(_detected);
  doc["last_update_ms"] = _lastUpdateMs;
  String out;
  serializeJson(doc, out);
  return out;
}

String PidAutotune::buildMqttProgressJson() const {
  JsonDocument doc;
  doc["phase"] = phaseToString(_phase);
  doc["elapsed_s"] = _startMs ? (millis() - _startMs) / 1000UL : 0;
  doc["target_c"] = _targetC;
  // During probe phase the active output comes from _probeOutputPct.
  doc["output_pct"] = (_phase == Phase::PROBE_RUNNING) ? _probeOutputPct : _outputPct;
  doc["noise_band_c"] = _noiseBandC;
  doc["sample_period_ms"] = _samplePeriodMs;
  doc["measured_rate_c_per_min"] = _measuredRateCPerMin;
  float temp = NAN;
  if (_heater && _heater->controlTempValid()) temp = _heater->controlTempC();
  if (isfinite(temp)) {
    doc["current_temp_c"] = temp;
  } else {
    doc["current_temp_c"] = nullptr;
  }
  String out;
  serializeJson(doc, out);
  return out;
}

String PidAutotune::buildMqttResultJson() const {
  JsonDocument doc;
  doc["phase"] = phaseToString(_phase);
  doc["rule"] = _result.rule;
  doc["kp"] = _result.kp;
  doc["ki"] = _result.ki;
  doc["kd"] = _result.kd;
  doc["ku"] = _result.ku;
  doc["pu"] = _result.pu;
  doc["quality"] = _result.quality;
  doc["valid"] = _result.valid;
  doc["last_error"] = _lastError;
  String out;
  serializeJson(doc, out);
  return out;
}

#include "HeaterController.h"

#include <algorithm>

#include "ControlProfile.h"
#include "GpioValidator.h"
#include "MqttBridge.h"
#include "TempManager.h"
#include "WebSerial.h"

namespace {
constexpr uint8_t kPwmChannel = 0;
constexpr uint8_t kRunawayMaxSamples = 12;
constexpr uint32_t kRunawayModeChangeGraceMs = 60000;
constexpr uint32_t kPidControlIntervalMs = 250;
constexpr uint32_t kRunawayOvershootHoldMs = 15000;
constexpr float kPidLookaheadS = 20.0f;
constexpr float kPidLookaheadMaxDeltaC = 2.0f;
constexpr float kPidSlopeFilter = 0.85f;
}  // namespace

HeaterController::HeaterController()
  : _requestedMode(ControlMode::IDLE),
    _effectiveMode(ControlMode::IDLE),
    _targetC(NAN),
    _outputPct(0.0f),
    _appliedPct(0.0f),
    _heaterOn(false),
    _outputEnabled(false),
    _enabledEffective(false),
    _usingBmsFallback(false),
    _controlTempC(NAN),
    _controlTempValid(false),
    _controlTempStale(false),
    _lastGoodControlTempC(NAN),
    _lastGoodControlTempMs(0),
    _lastModeInputActive(false),
    _pidIntegral(0.0f),
    _pidLastError(0.0f),
    _pidLastDeriv(0.0f),
    _pidLastTempC(NAN),
    _pidTempSlopeCps(0.0f),
    _pidTempSlopeValid(false),
    _lastControlMs(0),
    _hystState(false),
    _lastModeChangeMs(0),
    _runawayWaitForCooling(false),
    _outputLastChangeMs(0),
    _windowStartMs(0),
    _pwmChannel(kPwmChannel),
    _outputConfigured(false),
    _testActive(false),
    _testUntilMs(0),
    _testPct(0.0f),
    _overrideActive(false),
    _overrideTargetC(NAN),
    _overrideOutputPct(0.0f),
    _stuckActive(false),
    _stuckStartMs(0),
    _stuckStartTemp(0.0f),
    _runawayOvershootStartMs(0),
    _runawayCount(0),
    _runawayHead(0),
    _lastRunawaySampleMs(0),
    _faultLatchedMask(0),
    _faultActiveMask(0),
    _lastFault(FaultCode::CONFIG_INVALID),
    _lastFaultMs(0),
    _bootMs(0),
    _hadValidPrimary(false),
    _primaryInvalidSinceMs(0),
    _resetFaultsRequested(false) {
  memset(&_cfg, 0, sizeof(_cfg));
}

void HeaterController::begin(Settings& settings) {
  _bootMs = millis();
  _hadValidPrimary = false;
  _primaryInvalidSinceMs = 0;
  _lastGoodControlTempC = NAN;
  _lastGoodControlTempMs = 0;
  _controlTempStale = false;
  applySettings(settings);
  configureOutput();
}

void HeaterController::applySettings(Settings& settings) {
  _cfg.enabled = settings.get.enabled();
  _cfg.mode = static_cast<ControlMode>(settings.get.mode());
  _cfg.frostEnable = settings.get.frostEnable();
  _cfg.targetIdleC = settings.get.targetIdleC();
  _cfg.targetChargeC = settings.get.targetChargeC();
  _cfg.targetDischargeC = settings.get.targetDischargeC();
  _cfg.targetFrostC = settings.get.targetFrostC();
  _cfg.algorithm = algorithmFromInt(settings.get.algorithm());
  _cfg.pidKp = settings.get.pidKp();
  _cfg.pidKi = settings.get.pidKi();
  _cfg.pidKd = settings.get.pidKd();
  _cfg.pidIntegralLimit = settings.get.pidIntegralLimit();
  _cfg.pidDerivFilter = settings.get.pidDerivFilter();
  _cfg.hystOnDelta = settings.get.hystOnDelta();
  _cfg.hystOffDelta = settings.get.hystOffDelta();
  _cfg.manualOutputPct = settings.get.manualOutputPct();
  _cfg.maxOutputPct = settings.get.maxOutputPct();
  _cfg.minOnMs = settings.get.minOnMs();
  _cfg.minOffMs = settings.get.minOffMs();
  _cfg.maxTempC = settings.get.maxTempC();
  _cfg.maxDeltaC = settings.get.maxDeltaC();
  _cfg.stuckOnPct = settings.get.stuckOnPct();
  _cfg.stuckOnS = settings.get.stuckOnS();
  _cfg.minRiseC = settings.get.minRiseC();
  _cfg.riseWindowS = settings.get.riseWindowS();
  _cfg.runawayEnable = settings.get.runawayEnable();
  _cfg.runawayRateCPerMin = settings.get.runawayRateCPerMin();
  _cfg.runawayWindowS = settings.get.runawayWindowS();
  _cfg.runawayMarginC = settings.get.runawayMarginC();
  _cfg.runawayLatch = settings.get.runawayLatch();
  _cfg.mqttLossMode = failsafeFromInt(settings.get.mqttLossMode());
  _cfg.mqttTimeoutMs = static_cast<uint32_t>(settings.get.mqttTimeoutS()) * 1000UL;
  _cfg.bmsFallback = settings.get.bmsEnable() ? settings.get.bmsFallback() : false;
  _cfg.outputType = outputTypeFromInt(settings.get.heaterOutType());
  _cfg.outputInvert = settings.get.heaterOutInvert();
  _cfg.outputPin = settings.get.heaterOutPin();
  _cfg.oneWirePin = settings.get.oneWirePin();
  _cfg.pwmFreq = settings.get.pwmFreq();
  _cfg.pwmResolution = static_cast<uint8_t>(settings.get.pwmResolution());
  _cfg.windowMs = settings.get.windowMs();

  _cfg.enableInput.pin = settings.get.enableInPin();
  _cfg.enableInput.pull = static_cast<InputPull>(settings.get.enableInPull());
  _cfg.enableInput.active = static_cast<ActiveLevel>(settings.get.enableInActive());
  _cfg.enableInput.debounceMs = settings.get.enableInDebounce();

  _cfg.modeInput.pin = settings.get.modeInPin();
  _cfg.modeInput.pull = static_cast<InputPull>(settings.get.modeInPull());
  _cfg.modeInput.active = static_cast<ActiveLevel>(settings.get.modeInActive());
  _cfg.modeInput.debounceMs = settings.get.modeInDebounce();

  _cfg.manualInput.pin = settings.get.manualInPin();
  _cfg.manualInput.pull = static_cast<InputPull>(settings.get.manualInPull());
  _cfg.manualInput.active = static_cast<ActiveLevel>(settings.get.manualInActive());
  _cfg.manualInput.debounceMs = settings.get.manualInDebounce();

  _requestedMode = _cfg.mode;

  _enableInput.config = _cfg.enableInput;
  _modeInput.config = _cfg.modeInput;
  _manualInput.config = _cfg.manualInput;

  _enableInput.begin();
  _modeInput.begin();
  _manualInput.begin();

  configureOutput();
}

void HeaterController::configureOutput() {
  _outputConfigured = false;
  _heaterOn = false;
  _outputEnabled = false;
  _appliedPct = 0.0f;
  _outputLastChangeMs = millis();
  _windowStartMs = millis();

  if (_cfg.outputPin < 0) return;
  if (!GpioValidator::isValidOutputPin(_cfg.outputPin)) return;

  if (_cfg.outputType == OutputType::PWM) {
    ledcDetachPin(_cfg.outputPin);
    ledcSetup(_pwmChannel, _cfg.pwmFreq, _cfg.pwmResolution);
    ledcAttachPin(_cfg.outputPin, _pwmChannel);
    ledcWrite(_pwmChannel, 0);
  } else {
    pinMode(_cfg.outputPin, OUTPUT);
    digitalWrite(_cfg.outputPin, _cfg.outputInvert ? HIGH : LOW);
  }

  _outputConfigured = true;
}

void HeaterController::DebouncedInput::begin() {
  configured = config.pin >= 0;
  if (!configured) return;
  uint8_t mode = INPUT;
  if (config.pull == InputPull::PULL_UP) mode = INPUT_PULLUP;
  else if (config.pull == InputPull::PULL_DOWN) mode = INPUT_PULLDOWN;
  pinMode(config.pin, mode);
  stableState = digitalRead(config.pin);
  lastReading = stableState;
  lastChangeMs = millis();
}

void HeaterController::DebouncedInput::update(uint32_t nowMs) {
  if (!configured) return;
  bool reading = digitalRead(config.pin);
  if (reading != lastReading) {
    lastReading = reading;
    lastChangeMs = nowMs;
  }
  if ((nowMs - lastChangeMs) >= config.debounceMs && reading != stableState) {
    stableState = reading;
  }
}

bool HeaterController::DebouncedInput::isActive() const {
  if (!configured) return false;
  bool level = stableState;
  if (config.active == ActiveLevel::ACTIVE_LOW) level = !level;
  return level;
}

bool HeaterController::DebouncedInput::isConfigured() const {
  return configured;
}

void HeaterController::updateInputs(uint32_t nowMs) {
  _enableInput.update(nowMs);
  _modeInput.update(nowMs);
  _manualInput.update(nowMs);

  bool modeActive = _modeInput.isActive();
  if (modeActive && !_lastModeInputActive) {
    ControlMode next = _requestedMode;
    if (next == ControlMode::IDLE) next = ControlMode::CHARGE;
    else if (next == ControlMode::CHARGE) next = ControlMode::DISCHARGE;
    else if (next == ControlMode::DISCHARGE) next = _cfg.frostEnable ? ControlMode::FROST_PROTECT : ControlMode::IDLE;
    else if (next == ControlMode::FROST_PROTECT) next = ControlMode::IDLE;
    else next = ControlMode::IDLE;
    _requestedMode = next;
  }
  _lastModeInputActive = modeActive;
}

ControlMode HeaterController::applyModeOverrides(uint32_t nowMs, MqttBridge& mqtt, ControlMode baseMode) {
  ControlMode mode = baseMode;

  if (mqtt.bmsModeValid(nowMs)) {
    mode = mqtt.bmsMode();
  }

  if (_manualInput.isActive()) {
    mode = ControlMode::MANUAL;
  }

  if (_cfg.mqttLossMode != FailsafeMode::KEEP_LAST_SAFE && mqtt.isTimedOut(nowMs)) {
    if (_cfg.mqttLossMode == FailsafeMode::IDLE) {
      mode = ControlMode::IDLE;
    } else if (_cfg.mqttLossMode == FailsafeMode::FROST_PROTECT) {
      mode = _cfg.frostEnable ? ControlMode::FROST_PROTECT : ControlMode::IDLE;
    }
  }

  if (mode == ControlMode::FROST_PROTECT && !_cfg.frostEnable) {
    mode = ControlMode::IDLE;
  }

  return mode;
}

float HeaterController::computeTarget(ControlMode mode) const {
  switch (mode) {
    case ControlMode::CHARGE: return _cfg.targetChargeC;
    case ControlMode::DISCHARGE: return _cfg.targetDischargeC;
    case ControlMode::FROST_PROTECT: return _cfg.targetFrostC;
    case ControlMode::IDLE:
    default:
      return _cfg.targetIdleC;
  }
}

float HeaterController::computeOutputPid(uint32_t nowMs, float targetC, float tempC) {
  float dt = (nowMs - _lastControlMs) / 1000.0f;
  if (_lastControlMs == 0 || dt <= 0.0f) dt = 0.1f;
  _lastControlMs = nowMs;

  float rawSlopeCps = 0.0f;
  if (_pidTempSlopeValid) {
    rawSlopeCps = (tempC - _pidLastTempC) / dt;
    _pidTempSlopeCps = (_pidTempSlopeCps * kPidSlopeFilter) + (rawSlopeCps * (1.0f - kPidSlopeFilter));
  } else {
    _pidTempSlopeCps = 0.0f;
    _pidTempSlopeValid = true;
  }
  _pidLastTempC = tempC;

  float lookaheadDeltaC = _pidTempSlopeCps * kPidLookaheadS;
  if (lookaheadDeltaC > kPidLookaheadMaxDeltaC) lookaheadDeltaC = kPidLookaheadMaxDeltaC;
  if (lookaheadDeltaC < -kPidLookaheadMaxDeltaC) lookaheadDeltaC = -kPidLookaheadMaxDeltaC;
  const float predictedTempC = tempC + lookaheadDeltaC;
  const float error = targetC - predictedTempC;

  const float deriv = (error - _pidLastError) / dt;
  _pidLastError = error;
  _pidLastDeriv = (_pidLastDeriv * _cfg.pidDerivFilter) + (deriv * (1.0f - _cfg.pidDerivFilter));

  const float pTerm = _cfg.pidKp * error;
  const float dTerm = _cfg.pidKd * _pidLastDeriv;
  float iTerm = _cfg.pidKi * _pidIntegral;
  float output = pTerm + iTerm + dTerm;

  const float clamped = clampOutput(output);
  const bool atHighLimit = clamped >= _cfg.maxOutputPct && output > clamped;
  const bool atLowLimit = clamped <= 0.0f && output < clamped;
  const bool wouldWindUp = (atHighLimit && error > 0.0f) || (atLowLimit && error < 0.0f);

  if (!wouldWindUp) {
    _pidIntegral += error * dt;
    if (_pidIntegral > _cfg.pidIntegralLimit) _pidIntegral = _cfg.pidIntegralLimit;
    if (_pidIntegral < -_cfg.pidIntegralLimit) _pidIntegral = -_cfg.pidIntegralLimit;
    iTerm = _cfg.pidKi * _pidIntegral;
    output = pTerm + iTerm + dTerm;
  }

  return output;
}

float HeaterController::computeOutputHysteresis(float targetC, float tempC) {
  if (!_hystState && tempC <= (targetC - _cfg.hystOnDelta)) {
    _hystState = true;
  } else if (_hystState && tempC >= (targetC + _cfg.hystOffDelta)) {
    _hystState = false;
  }
  return _hystState ? _cfg.maxOutputPct : 0.0f;
}

float HeaterController::clampOutput(float pct) const {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > _cfg.maxOutputPct) pct = _cfg.maxOutputPct;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

void HeaterController::updateOutput(uint32_t nowMs, float desiredPct) {
  float pct = clampOutput(desiredPct);

  bool requestEnabled = pct > 0.0f;
  if (!_outputEnabled && requestEnabled) {
    if ((nowMs - _outputLastChangeMs) < _cfg.minOffMs) {
      pct = 0.0f;
      requestEnabled = false;
    }
  } else if (_outputEnabled && !requestEnabled) {
    if ((nowMs - _outputLastChangeMs) < _cfg.minOnMs) {
      pct = _appliedPct;
      requestEnabled = _appliedPct > 0.0f;
    }
  }

  if (requestEnabled != _outputEnabled) {
    _outputEnabled = requestEnabled;
    _outputLastChangeMs = nowMs;
  }

  const bool rampWindowActive = (_lastModeChangeMs != 0) &&
                                ((nowMs - _lastModeChangeMs) < ControlProfile::kHeatRampMs);
  const bool applyStartRamp = requestEnabled &&
                              rampWindowActive &&
                              !_testActive &&
                              _effectiveMode != ControlMode::MANUAL &&
                              (_cfg.algorithm == ControlAlgorithm::PID || _overrideActive);
  if (applyStartRamp) {
    const float startPct = min(_cfg.maxOutputPct, ControlProfile::kHeatStartPct);
    if (pct > startPct && _outputEnabled && ControlProfile::kHeatRampMs > 0) {
      const uint32_t rampMs = nowMs - _lastModeChangeMs;
      const float t = static_cast<float>(rampMs) / static_cast<float>(ControlProfile::kHeatRampMs);
      const float capPct = startPct + ((_cfg.maxOutputPct - startPct) * t);
      if (pct > capPct) pct = capPct;
    }
  }

  _appliedPct = pct;

  bool pinState = false;
  if (_cfg.outputType == OutputType::PWM) {
    uint32_t maxDuty = (1UL << _cfg.pwmResolution) - 1;
    uint32_t duty = (pct <= 0.0f) ? 0 : static_cast<uint32_t>((pct / 100.0f) * maxDuty);
    if (_cfg.outputInvert) duty = maxDuty - duty;
    if (_outputConfigured) {
      ledcWrite(_pwmChannel, duty);
    }
    pinState = duty > 0;
  } else {
    if (pct <= 0.0f) {
      pinState = false;
    } else if (pct >= 100.0f) {
      pinState = true;
    } else {
      if (nowMs - _windowStartMs >= _cfg.windowMs) {
        _windowStartMs = nowMs;
      }
      const uint32_t onMs = static_cast<uint32_t>(_cfg.windowMs * (pct / 100.0f));
      pinState = (nowMs - _windowStartMs) < onMs;
    }
    if (_cfg.outputInvert) pinState = !pinState;
    if (_outputConfigured) {
      digitalWrite(_cfg.outputPin, pinState ? HIGH : LOW);
    }
  }

  _heaterOn = pinState;
}

void HeaterController::pushRunawaySample(uint32_t nowMs, float tempC) {
  if (_runawayCount < kRunawayMaxSamples) {
    uint8_t idx = (_runawayHead + _runawayCount) % kRunawayMaxSamples;
    _runawaySamples[idx] = {nowMs, tempC};
    _runawayCount++;
  } else {
    _runawayHead = (_runawayHead + 1) % kRunawayMaxSamples;
    uint8_t idx = (_runawayHead + _runawayCount - 1) % kRunawayMaxSamples;
    _runawaySamples[idx] = {nowMs, tempC};
  }

  while (_runawayCount > 1) {
    const TempSample& oldest = _runawaySamples[_runawayHead];
    if ((nowMs - oldest.ms) <= (_cfg.runawayWindowS * 1000UL)) break;
    _runawayHead = (_runawayHead + 1) % kRunawayMaxSamples;
    _runawayCount--;
  }
}

void HeaterController::updateFaults(uint32_t nowMs, TempManager& temps, MqttBridge& mqtt) {
  _faultActiveMask = 0;

  if (!isConfigValid()) {
    setFault(FaultCode::CONFIG_INVALID, true, nowMs);
  }

  _usingBmsFallback = false;
  _controlTempValid = false;
  _controlTempC = NAN;
  _controlTempStale = false;

  bool primaryValid = false;
  float primaryTemp = NAN;
  temps.getRoleTemp(SensorRole::BATTERY_PRIMARY, &primaryTemp, &primaryValid);

  if (primaryValid) {
    _controlTempValid = true;
    _controlTempC = primaryTemp;
    _hadValidPrimary = true;
    _primaryInvalidSinceMs = 0;
    _lastGoodControlTempC = primaryTemp;
    _lastGoodControlTempMs = nowMs;
  } else if (_cfg.bmsFallback && mqtt.bmsTempValid(nowMs)) {
    _controlTempValid = true;
    _controlTempC = mqtt.bmsTempC();
    _usingBmsFallback = true;
    _primaryInvalidSinceMs = 0;
    _lastGoodControlTempC = _controlTempC;
    _lastGoodControlTempMs = nowMs;
  } else {
    if (_primaryInvalidSinceMs == 0) _primaryInvalidSinceMs = nowMs;
  }

  if (!_controlTempValid) {
    const uint32_t holdMs = 8000;
    if (_lastGoodControlTempMs != 0 && (nowMs - _lastGoodControlTempMs) <= holdMs) {
      _controlTempValid = true;
      _controlTempC = _lastGoodControlTempC;
      _controlTempStale = true;
    }
  }

  if (!_controlTempValid) {
    const uint32_t bootGraceMs = 10000;
    const uint32_t invalidHoldMs = 3000;
    const bool inBootGrace = (nowMs - _bootMs) < bootGraceMs;
    const bool inRescanGrace = temps.lastScanMs() != 0 && (nowMs - temps.lastScanMs()) < 4000;
    const bool shortInvalid = _primaryInvalidSinceMs != 0 && (nowMs - _primaryInvalidSinceMs) < invalidHoldMs;
    const bool latch = _hadValidPrimary && !inBootGrace && !inRescanGrace && !shortInvalid;
    const bool setNow = !inBootGrace && !shortInvalid;
    if (setNow) {
      setFault(FaultCode::SENSOR_PRIMARY_FAIL, latch, nowMs);
    }
  }

  if (_controlTempValid && _controlTempC > _cfg.maxTempC) {
    setFault(FaultCode::OVER_TEMP, true, nowMs);
  }

  bool secondaryValid = false;
  float secondaryTemp = NAN;
  temps.getRoleTemp(SensorRole::BATTERY_SECONDARY, &secondaryTemp, &secondaryValid);
  if (_controlTempValid && secondaryValid) {
    if (fabsf(_controlTempC - secondaryTemp) > _cfg.maxDeltaC) {
      setFault(FaultCode::PLAUSIBILITY_FAIL, true, nowMs);
    }
  }

  if (mqtt.isTimedOut(nowMs) && _cfg.mqttLossMode == FailsafeMode::OFF) {
    setFault(FaultCode::MQTT_TIMEOUT, true, nowMs);
  }

  if (_controlTempValid && _appliedPct >= _cfg.stuckOnPct) {
    if (!_stuckActive) {
      _stuckActive = true;
      _stuckStartMs = nowMs;
      _stuckStartTemp = _controlTempC;
    } else if ((nowMs - _stuckStartMs) >= (_cfg.stuckOnS * 1000UL)) {
      if ((nowMs - _stuckStartMs) >= (_cfg.riseWindowS * 1000UL)) {
        const float rise = _controlTempC - _stuckStartTemp;
        if (rise < _cfg.minRiseC) {
          setFault(FaultCode::STUCK_ON_NO_HEAT, true, nowMs);
        }
        _stuckActive = false;
      }
    }
  } else {
    _stuckActive = false;
  }

  bool runawayRateValid = false;
  float runawayRate = 0.0f;
  if (_cfg.runawayEnable && _controlTempValid) {
    if (_lastRunawaySampleMs != temps.lastUpdateMs() && temps.lastUpdateMs() != 0) {
      _lastRunawaySampleMs = temps.lastUpdateMs();
      pushRunawaySample(_lastRunawaySampleMs, _controlTempC);
    }

    if (_runawayCount >= 2) {
      const TempSample& oldest = _runawaySamples[_runawayHead];
      const uint8_t newestIdx = (_runawayHead + _runawayCount - 1) % kRunawayMaxSamples;
      const TempSample& newest = _runawaySamples[newestIdx];
      const float dtMin = (newest.ms - oldest.ms) / 60000.0f;
      if (dtMin > 0.0f) {
        runawayRate = (newest.tempC - oldest.tempC) / dtMin;
        runawayRateValid = true;
      }
    }
  }

  if (_runawayWaitForCooling && runawayRateValid && runawayRate < 0.0f) {
    _runawayWaitForCooling = false;
  }

  const bool runawayGraceActive = (_lastModeChangeMs != 0) && ((nowMs - _lastModeChangeMs) < kRunawayModeChangeGraceMs);
  if (_cfg.runawayEnable && !runawayGraceActive && !_runawayWaitForCooling && _controlTempValid && _appliedPct > 0.0f) {
    if (runawayRateValid && runawayRate > _cfg.runawayRateCPerMin) {
      webSerial.printf("[RUNAWAY] TRIGGER rate=%.3f limit=%.3f temp=%.2f target=%.2f applied=%.1f\n",
                       runawayRate, _cfg.runawayRateCPerMin, _controlTempC, _targetC, _appliedPct);
      setFault(FaultCode::THERMAL_RUNAWAY, _cfg.runawayLatch, nowMs);
    }

    if (_effectiveMode != ControlMode::MANUAL) {
      // Ignore pure overshoot when the pack is already cooling down.
      if ((!runawayRateValid || runawayRate >= 0.0f) &&
          _controlTempC > (_targetC + _cfg.runawayMarginC)) {
        if (_runawayOvershootStartMs == 0) {
          _runawayOvershootStartMs = nowMs;
        } else if ((nowMs - _runawayOvershootStartMs) >= kRunawayOvershootHoldMs) {
          webSerial.printf("[RUNAWAY] TRIGGER overshoot temp=%.2f target=%.2f margin=%.2f rate=%s%.3f applied=%.1f\n",
                           _controlTempC, _targetC, _cfg.runawayMarginC,
                           runawayRateValid ? "" : "n/a ",
                           runawayRateValid ? runawayRate : 0.0f,
                           _appliedPct);
          setFault(FaultCode::THERMAL_RUNAWAY, _cfg.runawayLatch, nowMs);
        }
      } else {
        _runawayOvershootStartMs = 0;
      }
    }
  } else {
    _runawayOvershootStartMs = 0;
  }

  if (_resetFaultsRequested) {
    if (_faultActiveMask == 0) {
      _faultLatchedMask = 0;
    }
    _resetFaultsRequested = false;
  }
}

bool HeaterController::isConfigValid() const {
  if (_cfg.outputPin < 0 || !GpioValidator::isValidOutputPin(_cfg.outputPin)) {
    return false;
  }
  if (_cfg.oneWirePin >= 0 && !GpioValidator::isValidOutputPin(_cfg.oneWirePin)) {
    return false;
  }
  if (_cfg.enableInput.pin >= 0 && !GpioValidator::isValidInputPin(_cfg.enableInput.pin)) {
    return false;
  }
  if (_cfg.modeInput.pin >= 0 && !GpioValidator::isValidInputPin(_cfg.modeInput.pin)) {
    return false;
  }
  if (_cfg.manualInput.pin >= 0 && !GpioValidator::isValidInputPin(_cfg.manualInput.pin)) {
    return false;
  }
  if (_cfg.outputPin == _cfg.oneWirePin && _cfg.oneWirePin >= 0) return false;
  if (_cfg.outputPin == _cfg.enableInput.pin && _cfg.enableInput.pin >= 0) return false;
  if (_cfg.outputPin == _cfg.modeInput.pin && _cfg.modeInput.pin >= 0) return false;
  if (_cfg.outputPin == _cfg.manualInput.pin && _cfg.manualInput.pin >= 0) return false;
  if (_cfg.targetIdleC > _cfg.maxTempC ||
      _cfg.targetChargeC > _cfg.maxTempC ||
      _cfg.targetDischargeC > _cfg.maxTempC ||
      _cfg.targetFrostC > _cfg.maxTempC) {
    return false;
  }
  return true;
}

void HeaterController::setFault(FaultCode code, bool latch, uint32_t nowMs) {
  const uint32_t bit = faultBit(code);
  const uint32_t prevMask = _faultActiveMask | _faultLatchedMask;
  _faultActiveMask |= bit;
  if (latch) _faultLatchedMask |= bit;
  if (!(prevMask & bit)) {
    _lastFault = code;
    _lastFaultMs = nowMs;
  }
}

void HeaterController::loop(uint32_t nowMs, TempManager& temps, MqttBridge& mqtt) {
  updateInputs(nowMs);

  bool hwEnable = !_enableInput.isConfigured() || _enableInput.isActive();
  _enabledEffective = _cfg.enabled && hwEnable;

  if (_cfg.mqttLossMode == FailsafeMode::OFF && mqtt.isTimedOut(nowMs)) {
    _enabledEffective = false;
  }

  ControlMode baseMode = _requestedMode;
  ControlMode newMode = applyModeOverrides(nowMs, mqtt, baseMode);
  if (newMode != _effectiveMode) {
    const float oldTarget = _targetC;
    const float newTarget = computeTarget(newMode);
    _lastModeChangeMs = nowMs;
    _runawayWaitForCooling = newTarget < oldTarget;
    if (newTarget < oldTarget) {
      _pidIntegral = 0.0f;
      _pidLastError = 0.0f;
      _pidLastDeriv = 0.0f;
      _lastControlMs = 0;
      _pidLastTempC = NAN;
      _pidTempSlopeCps = 0.0f;
      _pidTempSlopeValid = false;
    }
    _runawayCount = 0;
    _runawayHead = 0;
    _lastRunawaySampleMs = 0;
  }
  _effectiveMode = newMode;

  _targetC = computeTarget(_effectiveMode);
  if (_overrideActive) {
    _targetC = _overrideTargetC;
  }
  updateFaults(nowMs, temps, mqtt);

  bool faulted = (_faultLatchedMask != 0) || (_faultActiveMask != 0);
  if (!_enabledEffective || faulted) {
    _effectiveMode = faulted ? ControlMode::FAULT : _effectiveMode;
    _outputPct = 0.0f;
    _pidIntegral = 0.0f;
    _pidLastError = 0.0f;
    _pidLastDeriv = 0.0f;
    _lastControlMs = 0;
    _pidLastTempC = NAN;
    _pidTempSlopeCps = 0.0f;
    _pidTempSlopeValid = false;
    _hystState = false;
    updateOutput(nowMs, 0.0f);
    return;
  }

  if (_testActive && nowMs >= _testUntilMs) {
    _testActive = false;
  }

  float desiredPct = 0.0f;
  if (_overrideActive) {
    _pidIntegral = 0.0f;
    _pidLastError = 0.0f;
    _pidLastDeriv = 0.0f;
    _lastControlMs = 0;
    _pidLastTempC = NAN;
    _pidTempSlopeCps = 0.0f;
    _pidTempSlopeValid = false;
    _hystState = false;
    desiredPct = _overrideOutputPct;
  } else if (_testActive) {
    _pidIntegral = 0.0f;
    _pidLastError = 0.0f;
    _pidLastDeriv = 0.0f;
    _lastControlMs = 0;
    _pidLastTempC = NAN;
    _pidTempSlopeCps = 0.0f;
    _pidTempSlopeValid = false;
    _hystState = false;
    desiredPct = _testPct;
  } else if (_effectiveMode == ControlMode::MANUAL) {
    _pidIntegral = 0.0f;
    _pidLastError = 0.0f;
    _pidLastDeriv = 0.0f;
    _lastControlMs = 0;
    _pidLastTempC = NAN;
    _pidTempSlopeCps = 0.0f;
    _pidTempSlopeValid = false;
    _hystState = false;
    desiredPct = _cfg.manualOutputPct;
  } else if (_controlTempValid) {
    if (_cfg.algorithm == ControlAlgorithm::PID) {
      if (_lastControlMs == 0 || (nowMs - _lastControlMs) >= kPidControlIntervalMs) {
        desiredPct = computeOutputPid(nowMs, _targetC, _controlTempC);
      } else {
        desiredPct = _outputPct;
      }
    } else {
      _pidIntegral = 0.0f;
      _pidLastError = 0.0f;
      _pidLastDeriv = 0.0f;
      _lastControlMs = 0;
      _pidLastTempC = NAN;
      _pidTempSlopeCps = 0.0f;
      _pidTempSlopeValid = false;
      desiredPct = computeOutputHysteresis(_targetC, _controlTempC);
    }
  } else {
    _pidIntegral = 0.0f;
    _pidLastError = 0.0f;
    _pidLastDeriv = 0.0f;
    _lastControlMs = 0;
    _pidLastTempC = NAN;
    _pidTempSlopeCps = 0.0f;
    _pidTempSlopeValid = false;
  }

  _outputPct = clampOutput(desiredPct);
  updateOutput(nowMs, _outputPct);
}

void HeaterController::setRequestedMode(ControlMode mode) {
  _requestedMode = mode;
}

void HeaterController::setEnabled(bool enabled) {
  _cfg.enabled = enabled;
}

ControlMode HeaterController::requestedMode() const {
  return _requestedMode;
}

ControlMode HeaterController::effectiveMode() const {
  return _effectiveMode;
}

bool HeaterController::enabledEffective() const {
  return _enabledEffective;
}

float HeaterController::targetC() const {
  return _targetC;
}

float HeaterController::outputPct() const {
  return _outputPct;
}

bool HeaterController::heaterOn() const {
  return _heaterOn;
}

bool HeaterController::usingBmsFallback() const {
  return _usingBmsFallback;
}

float HeaterController::controlTempC() const {
  return _controlTempC;
}

bool HeaterController::controlTempValid() const {
  return _controlTempValid;
}

bool HeaterController::controlTempStale() const {
  return _controlTempStale;
}

uint32_t HeaterController::faultMaskLatched() const {
  return _faultLatchedMask;
}

uint32_t HeaterController::faultMaskActive() const {
  return _faultActiveMask;
}

FaultCode HeaterController::lastFault() const {
  return _lastFault;
}

uint32_t HeaterController::lastFaultMs() const {
  return _lastFaultMs;
}

bool HeaterController::requestFaultReset() {
  _resetFaultsRequested = true;
  return true;
}

bool HeaterController::startOutputTest(float pct, uint32_t durationMs) {
  if (pct < 0.0f || pct > 100.0f || durationMs == 0) return false;
  if (_overrideActive) return false;
  if (_faultLatchedMask != 0 || _faultActiveMask != 0) return false;
  _testActive = true;
  _testPct = pct;
  _testUntilMs = millis() + durationMs;
  return true;
}

void HeaterController::cancelOutputTest() {
  _testActive = false;
}

HeaterController::InputState HeaterController::inputState() const {
  return {
    _enableInput.isActive(),
    _modeInput.isActive(),
    _manualInput.isActive()
  };
}

void HeaterController::setExternalOverride(bool active, float targetC, float outputPct) {
  _overrideActive = active;
  if (active) {
    _overrideTargetC = targetC;
    _overrideOutputPct = outputPct;
    _testActive = false;
  } else {
    _overrideTargetC = NAN;
    _overrideOutputPct = 0.0f;
  }
}

bool HeaterController::externalOverrideActive() const {
  return _overrideActive;
}

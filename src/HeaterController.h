#pragma once

#include <Arduino.h>

#include "HeaterTypes.h"
#include "SettingsPrefs.h"

class TempManager;
class MqttBridge;

class HeaterController {
public:
  struct InputState {
    bool enableActive;
    bool modeActive;
    bool manualActive;
  };

  HeaterController();

  void begin(Settings& settings);
  void applySettings(Settings& settings);
  void loop(uint32_t nowMs, TempManager& temps, MqttBridge& mqtt);

  void setRequestedMode(ControlMode mode);
  void setEnabled(bool enabled);

  ControlMode requestedMode() const;
  ControlMode effectiveMode() const;
  bool enabledEffective() const;
  float targetC() const;
  float outputPct() const;
  bool heaterOn() const;
  bool usingBmsFallback() const;
  float controlTempC() const;
  bool controlTempValid() const;

  uint32_t faultMaskLatched() const;
  uint32_t faultMaskActive() const;
  FaultCode lastFault() const;
  uint32_t lastFaultMs() const;

  bool requestFaultReset();
  bool startOutputTest(float pct, uint32_t durationMs);
  void cancelOutputTest();
  void setExternalOverride(bool active, float targetC, float outputPct);
  bool externalOverrideActive() const;

  InputState inputState() const;

private:
  struct InputConfig {
    int32_t pin;
    InputPull pull;
    ActiveLevel active;
    uint16_t debounceMs;
  };

  struct DebouncedInput {
    InputConfig config;
    bool stableState;
    bool lastReading;
    uint32_t lastChangeMs;
    bool configured;

    void begin();
    void update(uint32_t nowMs);
    bool isActive() const;
    bool isConfigured() const;
  };

  struct Config {
    bool enabled;
    ControlMode mode;
    bool frostEnable;
    float targetIdleC;
    float targetChargeC;
    float targetDischargeC;
    float targetFrostC;
    ControlAlgorithm algorithm;
    float pidKp;
    float pidKi;
    float pidKd;
    float pidIntegralLimit;
    float pidDerivFilter;
    float hystOnDelta;
    float hystOffDelta;
    float manualOutputPct;
    float maxOutputPct;
    uint32_t minOnMs;
    uint32_t minOffMs;
    float maxTempC;
    float maxDeltaC;
    float stuckOnPct;
    uint32_t stuckOnS;
    float minRiseC;
    uint32_t riseWindowS;
    bool runawayEnable;
    float runawayRateCPerMin;
    uint32_t runawayWindowS;
    float runawayMarginC;
    bool runawayLatch;
    FailsafeMode mqttLossMode;
    uint32_t mqttTimeoutMs;
    bool bmsFallback;
    OutputType outputType;
    bool outputInvert;
    int32_t outputPin;
    int32_t oneWirePin;
    uint32_t pwmFreq;
    uint8_t pwmResolution;
    uint32_t windowMs;
    InputConfig enableInput;
    InputConfig modeInput;
    InputConfig manualInput;
  };

  struct TempSample {
    uint32_t ms;
    float tempC;
  };

  void configureOutput();
  void updateInputs(uint32_t nowMs);
  ControlMode applyModeOverrides(uint32_t nowMs, MqttBridge& mqtt, ControlMode baseMode);
  float computeTarget(ControlMode mode) const;
  float computeOutputPid(uint32_t nowMs, float targetC, float tempC);
  float computeOutputHysteresis(float targetC, float tempC);
  float clampOutput(float pct) const;
  void updateOutput(uint32_t nowMs, float desiredPct);
  void updateFaults(uint32_t nowMs, TempManager& temps, MqttBridge& mqtt);
  void pushRunawaySample(uint32_t nowMs, float tempC);
  bool isConfigValid() const;
  void setFault(FaultCode code, bool latch, uint32_t nowMs);

  Config _cfg;
  DebouncedInput _enableInput;
  DebouncedInput _modeInput;
  DebouncedInput _manualInput;

  ControlMode _requestedMode;
  ControlMode _effectiveMode;
  float _targetC;
  float _outputPct;
  float _appliedPct;
  bool _heaterOn;
  bool _outputEnabled;
  bool _enabledEffective;
  bool _usingBmsFallback;
  float _controlTempC;
  bool _controlTempValid;
  bool _lastModeInputActive;

  float _pidIntegral;
  float _pidLastError;
  float _pidLastDeriv;
  uint32_t _lastControlMs;
  bool _hystState;

  uint32_t _outputLastChangeMs;
  uint32_t _windowStartMs;
  uint8_t _pwmChannel;
  bool _outputConfigured;

  bool _testActive;
  uint32_t _testUntilMs;
  float _testPct;

  bool _overrideActive;
  float _overrideTargetC;
  float _overrideOutputPct;

  bool _stuckActive;
  uint32_t _stuckStartMs;
  float _stuckStartTemp;

  TempSample _runawaySamples[12];
  uint8_t _runawayCount;
  uint8_t _runawayHead;
  uint32_t _lastRunawaySampleMs;

  uint32_t _faultLatchedMask;
  uint32_t _faultActiveMask;
  FaultCode _lastFault;
  uint32_t _lastFaultMs;

  uint32_t _bootMs;
  bool _hadValidPrimary;
  uint32_t _primaryInvalidSinceMs;

  bool _resetFaultsRequested;
};

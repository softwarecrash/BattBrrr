#pragma once

#include <Arduino.h>

enum class ControlMode : uint8_t {
  IDLE = 0,
  CHARGE = 1,
  DISCHARGE = 2,
  FROST_PROTECT = 3,
  MANUAL = 4,
  FAULT = 5
};

enum class ControlAlgorithm : uint8_t {
  PID = 0,
  HYSTERESIS = 1
};

enum class OutputType : uint8_t {
  PWM = 0,
  WINDOW = 1
};

enum class InputPull : uint8_t {
  NONE = 0,
  PULL_UP = 1,
  PULL_DOWN = 2
};

enum class ActiveLevel : uint8_t {
  ACTIVE_HIGH = 0,
  ACTIVE_LOW = 1
};

enum class FailsafeMode : uint8_t {
  OFF = 0,
  FROST_PROTECT = 1,
  IDLE = 2,
  KEEP_LAST_SAFE = 3
};

enum class SensorRole : uint8_t {
  BATTERY_PRIMARY = 0,
  BATTERY_SECONDARY = 1,
  AMBIENT = 2,
  UNUSED = 3
};

enum class FaultCode : uint8_t {
  OVER_TEMP = 0,
  SENSOR_PRIMARY_FAIL = 1,
  PLAUSIBILITY_FAIL = 2,
  STUCK_ON_NO_HEAT = 3,
  THERMAL_RUNAWAY = 4,
  MQTT_TIMEOUT = 5,
  CONFIG_INVALID = 6
};

const char* modeToString(ControlMode mode);
ControlMode modeFromString(const String& value, ControlMode fallback = ControlMode::IDLE);

const char* algorithmToString(ControlAlgorithm algo);
ControlAlgorithm algorithmFromInt(int32_t value);

const char* outputTypeToString(OutputType type);
OutputType outputTypeFromInt(int32_t value);

const char* failsafeToString(FailsafeMode mode);
FailsafeMode failsafeFromInt(int32_t value);

const char* sensorRoleToString(SensorRole role);
SensorRole sensorRoleFromString(const String& value);

const char* faultCodeToString(FaultCode code);
uint32_t faultBit(FaultCode code);

#include "HeaterTypes.h"

const char* modeToString(ControlMode mode) {
  switch (mode) {
    case ControlMode::IDLE: return "IDLE";
    case ControlMode::CHARGE: return "CHARGE";
    case ControlMode::DISCHARGE: return "DISCHARGE";
    case ControlMode::FROST_PROTECT: return "FROST_PROTECT";
    case ControlMode::MANUAL: return "MANUAL";
    case ControlMode::FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

ControlMode modeFromString(const String& value, ControlMode fallback) {
  if (value.equalsIgnoreCase("IDLE")) return ControlMode::IDLE;
  if (value.equalsIgnoreCase("CHARGE")) return ControlMode::CHARGE;
  if (value.equalsIgnoreCase("DISCHARGE")) return ControlMode::DISCHARGE;
  if (value.equalsIgnoreCase("FROST") || value.equalsIgnoreCase("FROST_PROTECT")) {
    return ControlMode::FROST_PROTECT;
  }
  if (value.equalsIgnoreCase("MANUAL")) return ControlMode::MANUAL;
  if (value.equalsIgnoreCase("FAULT")) return ControlMode::FAULT;
  return fallback;
}

const char* algorithmToString(ControlAlgorithm algo) {
  switch (algo) {
    case ControlAlgorithm::PID: return "PID";
    case ControlAlgorithm::HYSTERESIS: return "HYSTERESIS";
    default: return "UNKNOWN";
  }
}

ControlAlgorithm algorithmFromInt(int32_t value) {
  return (value == 1) ? ControlAlgorithm::HYSTERESIS : ControlAlgorithm::PID;
}

const char* outputTypeToString(OutputType type) {
  switch (type) {
    case OutputType::PWM: return "PWM";
    case OutputType::WINDOW: return "WINDOW";
    default: return "UNKNOWN";
  }
}

OutputType outputTypeFromInt(int32_t value) {
  return (value == 0) ? OutputType::PWM : OutputType::WINDOW;
}

const char* failsafeToString(FailsafeMode mode) {
  switch (mode) {
    case FailsafeMode::OFF: return "OFF";
    case FailsafeMode::FROST_PROTECT: return "FROST_PROTECT";
    case FailsafeMode::IDLE: return "IDLE";
    case FailsafeMode::KEEP_LAST_SAFE: return "KEEP_LAST_SAFE";
    default: return "UNKNOWN";
  }
}

FailsafeMode failsafeFromInt(int32_t value) {
  switch (value) {
    case 1: return FailsafeMode::FROST_PROTECT;
    case 2: return FailsafeMode::IDLE;
    case 3: return FailsafeMode::KEEP_LAST_SAFE;
    default: return FailsafeMode::OFF;
  }
}

const char* sensorRoleToString(SensorRole role) {
  switch (role) {
    case SensorRole::BATTERY_PRIMARY: return "battery_primary";
    case SensorRole::BATTERY_SECONDARY: return "battery_secondary";
    case SensorRole::AMBIENT: return "ambient";
    case SensorRole::UNUSED: return "unused";
    default: return "unknown";
  }
}

SensorRole sensorRoleFromString(const String& value) {
  if (value.equalsIgnoreCase("battery_primary") || value.equalsIgnoreCase("primary")) {
    return SensorRole::BATTERY_PRIMARY;
  }
  if (value.equalsIgnoreCase("battery_secondary") || value.equalsIgnoreCase("secondary")) {
    return SensorRole::BATTERY_SECONDARY;
  }
  if (value.equalsIgnoreCase("ambient")) return SensorRole::AMBIENT;
  return SensorRole::UNUSED;
}

const char* faultCodeToString(FaultCode code) {
  switch (code) {
    case FaultCode::OVER_TEMP: return "OVER_TEMP";
    case FaultCode::SENSOR_PRIMARY_FAIL: return "SENSOR_PRIMARY_FAIL";
    case FaultCode::PLAUSIBILITY_FAIL: return "PLAUSIBILITY_FAIL";
    case FaultCode::STUCK_ON_NO_HEAT: return "STUCK_ON_NO_HEAT";
    case FaultCode::THERMAL_RUNAWAY: return "THERMAL_RUNAWAY";
    case FaultCode::MQTT_TIMEOUT: return "MQTT_TIMEOUT";
    case FaultCode::CONFIG_INVALID: return "CONFIG_INVALID";
    default: return "UNKNOWN";
  }
}

uint32_t faultBit(FaultCode code) {
  return 1UL << static_cast<uint8_t>(code);
}

#pragma once

#include <Arduino.h>

namespace GpioValidator {
bool isValidGpio(int pin);
bool isValidInputPin(int pin);
bool isValidOutputPin(int pin);
bool isStrappingPin(int pin);
bool isReservedPin(int pin);
}  // namespace GpioValidator

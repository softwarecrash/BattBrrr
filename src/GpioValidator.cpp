#include "GpioValidator.h"

#include <driver/gpio.h>
#include <sdkconfig.h>

namespace {
bool pinInList(int pin, const int* list, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    if (pin == list[i]) return true;
  }
  return false;
}
}  // namespace

namespace GpioValidator {

bool isStrappingPin(int pin) {
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  const int strappingPins[] = {0, 2, 8, 9};
  return pinInList(pin, strappingPins, sizeof(strappingPins) / sizeof(strappingPins[0]));
#else
  const int strappingPins[] = {0, 2, 4, 5, 12, 15};
  return pinInList(pin, strappingPins, sizeof(strappingPins) / sizeof(strappingPins[0]));
#endif
}

bool isReservedPin(int pin) {
#if defined(CONFIG_IDF_TARGET_ESP32C3)
  // SPI flash pins are not valid GPIOs on ESP32-C3; GPIO_IS_VALID_GPIO will reject them.
  (void)pin;
  return false;
#else
  const int flashPins[] = {6, 7, 8, 9, 10, 11};
  return pinInList(pin, flashPins, sizeof(flashPins) / sizeof(flashPins[0]));
#endif
}

bool isValidGpio(int pin) {
  if (pin < 0) return false;
  if (isReservedPin(pin)) return false;
  return GPIO_IS_VALID_GPIO(static_cast<gpio_num_t>(pin));
}

bool isValidInputPin(int pin) {
  if (!isValidGpio(pin)) return false;
  if (isStrappingPin(pin)) return false;
  return true;
}

bool isValidOutputPin(int pin) {
  if (!isValidGpio(pin)) return false;
  if (isStrappingPin(pin)) return false;
  return GPIO_IS_VALID_OUTPUT_GPIO(static_cast<gpio_num_t>(pin));
}

}  // namespace GpioValidator

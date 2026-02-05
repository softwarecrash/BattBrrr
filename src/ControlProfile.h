#pragma once

#include <stdint.h>

namespace ControlProfile {
// Shared startup behavior for closed-loop heating so autotune and runtime see similar dynamics.
constexpr float kHeatStartPct = 20.0f;
constexpr uint32_t kHeatRampMs = 60000;
}  // namespace ControlProfile

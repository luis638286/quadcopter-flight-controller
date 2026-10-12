#include "domain/AltitudeController.h"

#include <algorithm>

// Gains for the 1 kg / 16 N plant:
//   kp  how hard it pulls toward the setpoint
//   kd  damping; derivative on measurement acts on -vz
//   ki  0 for now: the hover feedforward is exact in this sim, so there is
//       no steady offset to remove. A real drone needs ki > 0.
namespace {
constexpr float kKp = 0.25f;
constexpr float kKi = 0.0f;
constexpr float kKd = 0.20f;
}

AltitudeController::AltitudeController(float hover_throttle)
    : hover_throttle_(hover_throttle),
      // PID output is a correction around hover, limited so that
      // hover + correction always stays inside 0..1
      pid_(kKp, kKi, kKd, -hover_throttle, 1.0f - hover_throttle) {}

float AltitudeController::update(float altitude_setpoint,
                                 float altitude_measured, float dt) {
    // exponential moving average: move a fraction alpha toward the new reading
    if (first_) {
        filtered_ = altitude_measured;
        first_    = false;
    } else {
        const float alpha = dt / (kFilterTau + dt);
        filtered_ += alpha * (altitude_measured - filtered_);
    }

    const float correction = pid_.update(altitude_setpoint, filtered_, dt);
    return std::clamp(hover_throttle_ + correction, 0.0f, 1.0f);
}

void AltitudeController::reset() {
    pid_.reset();
    first_ = true;
}
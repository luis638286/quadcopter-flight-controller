#pragma once
#include "domain/PIDController.h"

// Holds the drone at a commanded altitude.
//
// Pure maths, like PIDController: it receives a setpoint and a measurement and
// returns a throttle. Reading the sensor and writing the motors is done by the
// caller (main for now, FlightController later), so this class never touches
// hardware or the simulator.
//
// throttle = hover feedforward + PID correction
//   feedforward: the throttle that exactly cancels gravity (net force 0)
//   PID:         the push above/below that to reach and hold the setpoint
class AltitudeController {
public:
    explicit AltitudeController(float hover_throttle);

    // altitude_setpoint  m, where we want to be
    // altitude_measured  m, raw (noisy) altimeter reading
    // dt                 s, time since the previous call
    // returns throttle 0..1; the same value goes to all four motors
    float update(float altitude_setpoint, float altitude_measured, float dt);

    void reset();

    // last filtered altitude, useful for logging
    float filtered_altitude() const { return filtered_; }

private:
    float hover_throttle_;

    // low-pass filter on the altimeter: the D term differentiates the
    // measurement, and differentiating raw noise every 2 ms gives huge spikes
    static constexpr float kFilterTau = 0.2f;   // s, filter time constant
    float filtered_ = 0.0f;
    bool  first_    = true;

    PIDController pid_;
};
#pragma once

// single-axis PID controller with output clamping and anti-windup
//
// pure maths: no hardware, no clock, no simulator dependency. dt is supplied
// by the caller, so the controller stays a pure function of its inputs and
// the control loop keeps ownership of time

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max);

    // runs one control cycle
    //   setpoint  what we want        (same unit as measured)
    //   measured  what we have        (e.g. deg/s for a rate loop)
    //   dt        seconds since the previous call; must be > 0
    // returns the control output, clamped to [out_min, out_max]
    float update(float setpoint, float measured, float dt);

    // clears accumulated state. call on mode entry and on arming, so a stale
    // integral cannot kick the motors after a period of inactivity
    void reset();

private:
    // gains, fixed for the life of the object
    float kp_;
    float ki_;
    float kd_;

    // output limits, the range the actuator can actually accept
    float out_min_;
    float out_max_;

    // state carried between cycles
    float integral_      = 0.0f;   // error-seconds
    float prev_measured_ = 0.0f;
    bool  first_         = true;   // no valid previous measurement yet
};
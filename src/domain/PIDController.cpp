#include "domain/PIDController.h"

#include <algorithm>
#include <cassert>

PIDController::PIDController(float kp, float ki, float kd,
                             float out_min, float out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max) {
    assert(out_min < out_max);
}

float PIDController::update(float setpoint, float measured, float dt) {
    assert(dt > 0.0f);

    const float error = setpoint - measured;

    // derivative on measurement, not on error. a step change in the setpoint
    // would otherwise produce a one-cycle spike ("derivative kick")
    float derivative = 0.0f;
    if (!first_) {
        derivative = -(measured - prev_measured_) / dt;
    }
    prev_measured_ = measured;
    first_ = false;

    integral_ += error * dt;

    const float out     = kp_ * error + ki_ * integral_ + kd_ * derivative;
    const float clamped = std::clamp(out, out_min_, out_max_);

    // conditional integration (the "clamping" anti-windup method): while the
    // output is saturated, do not accumulate error that pushes further into
    // the limit. error of the opposite sign still unwinds the integral, so
    // the controller recovers as soon as the error reverses
    const bool saturated       = (out != clamped);
    const bool pushing_further = (error > 0.0f) == (out > out_max_);
    if (saturated && pushing_further) {
        integral_ -= error * dt;
    }

    return clamped;
}

void PIDController::reset() {
    integral_      = 0.0f;
    prev_measured_ = 0.0f;
    first_         = true;
}
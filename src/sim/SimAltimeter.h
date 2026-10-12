#pragma once
#include <random>
#include "hal/IAltitudeSensor.h"
#include "sim/PhysicsModel.h"

// Fake barometer: true altitude + Gaussian noise
class SimAltimeter : public IAltitudeSensor {
public:
    explicit SimAltimeter(const PhysicsModel& physics, float noiseStdDevM = 0.05f)
        : physics_(physics), rng_(42), noise_(0.0f, noiseStdDevM) {}

    bool read(float& altitudeM) override {
        altitudeM = physics_.truth().z + noise_(rng_);
        return true;
    }

private:
    const PhysicsModel& physics_;
    std::mt19937 rng_;                       // fixed seed -> same noise every run
    std::normal_distribution<float> noise_;  // mean 0, std dev in metres
};
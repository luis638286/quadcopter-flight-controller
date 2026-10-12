#pragma once
#include "Types.h"

// Vertical-only plant: no rotation, no drag, instant motor response.
class PhysicsModel {
public:
    void set_motor_commands(const MotorCommands& cmd);
    void step(float dt);
    TruthState truth() const;

private:
    static constexpr float kMass    = 1.0f;   // kg
    static constexpr float kGravity = 9.81f;  // m/s^2
    static constexpr float kThrust  = 4.0f;   // N per motor at command 1.0

    TruthState    state_{};
    MotorCommands cmd_{};
};
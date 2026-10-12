#pragma once
#include "Types.h"

// Sends normalised motor commands (0..1 per motor) to whatever drives the motors.
// Real hardware maps 0..1 to its own protocol (PWM, DShot); the controller never knows.
class IMotorOutput {
public:
    virtual ~IMotorOutput() = default;
    virtual void write(const MotorCommands& cmd) = 0;
};
#pragma once
#include "hal/IMotorOutput.h"
#include "sim/PhysicsModel.h"

// Fake motors: hand the commands straight to the physics model.
class SimMotors : public IMotorOutput {
public:
    explicit SimMotors(PhysicsModel& physics) : physics_(physics) {}

    void write(const MotorCommands& cmd) override {
        physics_.set_motor_commands(cmd);
    }

private:
    PhysicsModel& physics_;   // not const: motors change the world
};
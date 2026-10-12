#include "sim/PhysicsModel.h"

void PhysicsModel::step(float dt) {
    float total_thrust = (cmd_.front_left + cmd_.front_right + cmd_.rear_left + cmd_.rear_right) * kThrust;
    float weight       = kMass * kGravity;
    float net_force    = total_thrust - weight;
    float acceleration = net_force / kMass;

    float velocity = state_.vz + acceleration * dt;
    float altitude = state_.z + velocity * dt;

    if (altitude < 0.0f) {     // ground
        altitude = 0.0f;
        velocity = 0.0f;
    }

    state_.z  = altitude;
    state_.vz = velocity;
}

void PhysicsModel::set_motor_commands(const MotorCommands& cmd) {
    cmd_ = cmd;
}

TruthState PhysicsModel::truth() const {
    return state_;
}
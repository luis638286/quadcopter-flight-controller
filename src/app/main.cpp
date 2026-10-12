#include <cstdio>

#include "sim/PhysicsModel.h"
#include "sim/SimAltimeter.h"
#include "sim/SimClock.h"
#include "sim/SimMotors.h"
#include "domain/AltitudeController.h"

// scripted pilot: where we want the drone to be at time t
float altitude_setpoint(float t) {
    if (t < 1.0f)  return 0.0f;   // sit on the ground
    if (t < 10.0f) return 5.0f;   // climb to 5 m and hold
    return 2.0f;                  // descend to 2 m and hold
}

int main() {
    // the simulated world
    PhysicsModel physics;
    SimClock     clock;
    SimAltimeter altimeter_sim(physics);
    SimMotors    motors_sim(physics);

    // the controller side only ever sees the interfaces
    IAltitudeSensor& altimeter = altimeter_sim;
    IMotorOutput&    motors    = motors_sim;

    // hover throttle: weight / max total thrust = 9.81 N / (4 motors * 4 N)
    AltitudeController controller(9.81f / 16.0f);

    const uint64_t dt_micros  = 2000;   // 2 ms, 500 Hz
    const float    dt_seconds = static_cast<float>(dt_micros) / 1e6f;
    const float    run_time   = 20.0f;  // s

    std::printf("    t   setpoint   z_true   z_meas   throttle\n");

    int cycle = 0;
    while (clock.seconds() < run_time) {
        const float t = clock.seconds();

        // 1. sense
        float z_meas = 0.0f;
        altimeter.read(z_meas);

        // 2. decide
        const float setpoint = altitude_setpoint(t);
        const float throttle = controller.update(setpoint, z_meas, dt_seconds);

        // 3. act: same throttle on all four motors (no roll/pitch/yaw yet)
        MotorCommands cmd;
        cmd.front_left = cmd.front_right = cmd.rear_left = cmd.rear_right = throttle;
        motors.write(cmd);

        // 4. advance the world and time
        physics.step(dt_seconds);
        clock.advance(dt_micros);

        // print 4 lines per second
        if (cycle % 125 == 0) {
            std::printf("%5.2f   %8.2f   %6.3f   %6.3f   %8.3f\n",
                        t, setpoint, physics.truth().z, z_meas, throttle);
        }
        ++cycle;
    }
    return 0;
}
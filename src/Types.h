#pragma once
#include <cstdint>

// one sample from the inertial measurement unit
struct ImuSample {
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;   // angular rate, deg/s, body axes
    float ax = 0.0f, ay = 0.0f, az = 0.0f;   // specific force, m/s^2, body axes
};

// estimated orientation
struct Attitude {
    float roll  = 0.0f;   // deg, positive = right side down
    float pitch = 0.0f;   // deg, positive = nose up
};

// how the system interprets the stick channels
enum class Mode { Manual, Stabilised, AltitudeHold };

// one frame from the receiver: stick positions
struct ReceiverFrame {
    float roll     = 0.0f;   // -1 .. +1   right stick, horizontal
    float pitch    = 0.0f;   // -1 .. +1   right stick, vertical
    float yaw      = 0.0f;   // -1 .. +1   left stick, horizontal
    float throttle = 0.0f;   //  0 .. +1   left stick, vertical
    Mode  mode_switch = Mode::Manual;
};

// what the controller asks of the airframe, before mixing
struct Demand {
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;   // torque demands, -1 .. +1
    float throttle = 0.0f;                          //  0 .. +1
};

// what each motor is told to do, each 0 .. +1
struct MotorCommands {
    float front_left = 0.0f, front_right = 0.0f;
    float rear_left  = 0.0f, rear_right  = 0.0f;
};

// gains for one PID loop
struct PidGains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
};

// the real state of the drone: only sim/ and the logger may read this
struct TruthState {
    float roll  = 0.0f;   // deg, positive = right side down
    float pitch = 0.0f;   // deg, positive = nose up
    float yaw   = 0.0f;   // deg, positive = nose right
    float z     = 0.0f;   // m above takeoff point
    float vz    = 0.0f;   // m/s, positive = up
};
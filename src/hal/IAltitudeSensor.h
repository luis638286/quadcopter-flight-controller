#pragma once

class IAltitudeSensor {
public:
    virtual ~IAltitudeSensor() = default;

    // Writes altitude above the takeoff point in metres.
    // Returns false if no valid reading is available (sensor fault, not ready).
    virtual bool read(float& altitudeM) = 0;
};
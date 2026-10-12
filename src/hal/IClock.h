#pragma once
#include <cstdint>

class IClock {
public:
    virtual ~IClock() = default;
    virtual uint64_t now_micros() const = 0;
};
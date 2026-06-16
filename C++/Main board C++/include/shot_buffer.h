#pragma once

#include "leg_checker.h"
#include "shot_vector.h"
#include <Arduino.h>

// Ring buffer of up to 3 Shots for leg-consistency checking.
class ShotBuffer {
  public:
    static constexpr uint8_t CAPACITY = 3;

    ShotBuffer(const ILegChecker &checker) : checker_(checker) {}

    void push(const Shot &s) {
        if (count_ < CAPACITY) {
            buf_[count_++] = s;
        } else {
            buf_[0] = buf_[1];
            buf_[1] = buf_[2];
            buf_[2] = s;
        }
    }

    void clear() { count_ = 0; }

    uint8_t count() const { return count_; }
    bool full() const { return count_ == CAPACITY; }

    const Shot &operator[](uint8_t i) const { return buf_[i]; }

    bool hasValidLeg() const {
        if (count_ < CAPACITY) {
            return false;
        }
        return checker_.hasValidLeg(buf_, count_);
    }

  private:
    const ILegChecker &checker_;
    Shot buf_[CAPACITY];
    uint8_t count_ = 0;
};

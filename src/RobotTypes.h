#pragma once
#include <Arduino.h>

#include <array>

#include "Streaming.h"

typedef std::array<float, 12> ActuatorPositionVector;
typedef std::array<float, 12> ActuatorVelocityVector;
typedef std::array<float, 12> ActuatorCurrentVector;
typedef std::array<bool, 12> ActuatorActivations;

template <class T, unsigned int SIZE>
Print &operator<<(Print &stream, const std::array<T, SIZE> &vec) {
  for (auto e : vec) {
    stream << e << " ";
  }
  return stream;
}
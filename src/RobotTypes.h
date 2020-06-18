#pragma once
#include <array>
#include <Arduino.h>
#include "Streaming.h"

typedef std::array<float, 12> ActuatorPositionVector;

Print &operator<<(Print &stream, const ActuatorPositionVector &vec);
#pragma once

#include <Streaming.h>

struct PDGains
{
    float kp;
    float kd;
};

Print &operator<<(Print &stream, const PDGains &gains);

void PD(float &torque_command, float measurement_pos, float measurement_vel, float reference_pos, float reference_vel, PDGains gains);
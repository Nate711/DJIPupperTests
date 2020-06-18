#include "PID.h"

void PD(float &torque_command, float measurement_pos, float measurement_vel, float reference_pos, float reference_vel, PDGains gains)
{
    torque_command = gains.kp * (reference_pos - measurement_pos) + gains.kd * (reference_vel - measurement_vel);
}

Print &operator<<(Print &stream, const PDGains &gains)
{
    stream << "kp: " << gains.kp << " kd: " << gains.kd;
    return stream;
}
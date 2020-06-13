#pragma once

struct PDGains
{
    float kp;
    float kd;
};

// void pid(int32_t &torque_command, int32_t measurement_pos, int32_t measurement_vel, int32_t reference_pos, int32_t reference_vel, PDGains gains)
// {
//     torque_command = gains.kp * (reference_pos - measurement_pos) + gains.kd * (reference_vel - measurement_vel);
// }

void PD(float &torque_command, float measurement_pos, float measurement_vel, float reference_pos, float reference_vel, PDGains gains);
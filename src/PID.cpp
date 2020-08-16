#include "PID.h"

#include <BasicLinearAlgebra.h>

void PD(float &torque_command, float measurement_pos, float measurement_vel,
        float reference_pos, float reference_vel, PDGains gains) {
  torque_command = gains.kp * (reference_pos - measurement_pos) +
                   gains.kd * (reference_vel - measurement_vel);
}

Print &operator<<(Print &stream, const PDGains &gains) {
  stream << "kp: " << gains.kp << " kd: " << gains.kd;
  return stream;
}

BLA::Matrix<3> PDControl3(BLA::Matrix<3> measured_position,
                          BLA::Matrix<3> measured_velocity,
                          BLA::Matrix<3> reference_position,
                          BLA::Matrix<3> reference_velocity, PDGains3x3 gains) {
  return gains.kp * (reference_position - measured_position) +
         gains.kd * (reference_velocity - measured_velocity);
}
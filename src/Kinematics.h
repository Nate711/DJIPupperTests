// Cartesian PD Control
#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

struct LegParameters {
  float thigh_length = 0.08;
  float shank_length = 0.11;
  float hip_offset = 0.01;  // unsigned
};

struct HipLayoutParameters {
  float x_offset = 0.1;
  float y_offset = 0.06;
  float z_offset = 0.0;
};

enum class RobotSide { kLeft, kRight };

float HipOffset(LegParameters leg_params, uint8_t leg_index);
BLA::Matrix<3, 3> RotateX(float theta);
BLA::Matrix<3> ForwardKinematics(BLA::Matrix<3> joint_angles,
                             LegParameters leg_params, uint8_t leg_index);
BLA::Matrix<3, 3> LegJacobian(BLA::Matrix<3> joint_angles,
                              LegParameters leg_params, uint8_t leg_index);

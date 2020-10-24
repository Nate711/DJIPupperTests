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

// Returns the position of the leg relative to the center of the body
BLA::Matrix<3> HipPosition(HipLayoutParameters hip_layoout_params, uint8_t i);

// Return a rotation matrix around x-axis
BLA::Matrix<3, 3> RotateX(float theta);

// Return the hip-relative cartesian coordaintes of a foot given its leg's joint angle
BLA::Matrix<3> ForwardKinematics(BLA::Matrix<3> joint_angles,
                             LegParameters leg_params, uint8_t leg_index);

// Return the velocity jacobian of the specified leg
BLA::Matrix<3, 3> LegJacobian(BLA::Matrix<3> joint_angles,
                              LegParameters leg_params, uint8_t leg_index);

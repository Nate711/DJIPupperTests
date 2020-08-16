// Cartesian PD Control
#pragma once

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

struct LegParameters {
  float thigh_length;
  float shank_length;
  float hip_offset;  // unsigned
};

enum class RobotSide { kLeft, kRight };

float HipOffset(LegParameters leg_params, RobotSide side);
BLA::Matrix<3, 3> RotateX(float theta);
BLA::Matrix<3> LegKinematics(BLA::Matrix<3> joint_angles,
                             LegParameters leg_params, RobotSide side);
BLA::Matrix<3, 3> LegJacobian(BLA::Matrix<3> joint_angles,
                              LegParameters leg_params, RobotSide side);
BLA::Matrix<3> ForceToTorque(BLA::Matrix<3> leg_angles, BLA::Matrix<3> force,
                             LegParameters leg_params, RobotSide side);

BLA::Matrix<3, 3> DiagonalMatrix3x3(float value);
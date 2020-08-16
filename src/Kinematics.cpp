#include "Kinematics.h"

#include <BasicLinearAlgebra.h>

float HipOffset(LegParameters leg_params, RobotSide side) {
  if (side == RobotSide::kLeft) {
    return leg_params.hip_offset;
  } else if (side == RobotSide::kRight) {
    return -leg_params.hip_offset;
  }
  return 0;
}

BLA::Matrix<3, 3> RotateX(float theta) {
  BLA::Matrix<3, 3> out = {1,           0, 0,          0,         cos(theta),
                           -sin(theta), 0, sin(theta), cos(theta)};
  return out;
}

BLA::Matrix<3> LegKinematics(BLA::Matrix<3> joint_angles,
                             LegParameters leg_params, RobotSide side) {
  float l1 = leg_params.thigh_length;
  float l2 = leg_params.shank_length;

  float alpha = joint_angles(0);
  float theta = joint_angles(1);
  float phi = joint_angles(2);

  float px = -l1 * sin(theta) - l2 * sin(theta + phi);
  float py = HipOffset(leg_params, side);
  float pz = -l1 * cos(theta) - l2 * cos(theta + phi);

  BLA::Matrix<3> tilted_frame_coordinates = {px, py, pz};
  BLA::Matrix<3> cartesian_coordinates =
      RotateX(alpha) * tilted_frame_coordinates;
  return cartesian_coordinates;
}

/*
Calculate the leg jacobian for a given configuration, leg parameters, and leg
side.

joint_angles: 3-vector of joint angles. Order is {abduction, thigh, knee}
leg_parameters: LegParameters for the leg
side: RobotSide of the leg
*/
BLA::Matrix<3, 3> LegJacobian(BLA::Matrix<3> joint_angles,
                              LegParameters leg_params, RobotSide side) {
  float l1 = leg_params.thigh_length;
  float l2 = leg_params.shank_length;

  float alpha = joint_angles(0);
  float theta = joint_angles(1);
  float phi = joint_angles(2);

  float px = -l1 * sin(theta) - l2 * sin(theta + phi);
  float py = HipOffset(leg_params, side);
  float pz = -l1 * cos(theta) - l2 * cos(theta + phi);

  BLA::Matrix<3, 3> jac = {0,
                           pz,
                           -l2 * cos(theta + phi),
                           -py * sin(alpha) - pz * cos(alpha),
                           sin(alpha) * px,
                           -l2 * sin(alpha) * sin(theta + phi),
                           py * cos(alpha) - pz * sin(alpha),
                           -px * cos(alpha),
                           l2 * cos(alpha) * sin(theta + phi)};
  return jac;
}

/*
Convert a desired force to torques at the leg motors.

Args:
    leg_angles:
    force:
    leg_params:
    side:
*/
BLA::Matrix<3> ForceToTorque(BLA::Matrix<3> leg_angles, BLA::Matrix<3> force,
                             LegParameters leg_params, RobotSide side) {
  return LegJacobian(leg_angles, leg_params, side) * force;
}

BLA::Matrix<3, 3> DiagonalMatrix3x3(float value) {
  BLA::Matrix<3, 3> retval;
  retval.Fill(0);
  for (int i = 0; i < 3; i++) {
    retval(i, i) = value;
  }
  return retval;
}
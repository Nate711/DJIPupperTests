#include "Kinematics.h"

#include <BasicLinearAlgebra.h>

float HipOffset(LegParameters leg_params, uint8_t leg_index) {
  if (leg_index == 0 || leg_index == 2) {
    return -leg_params.hip_offset;
  }
  if (leg_index == 1 || leg_index == 3) {
    return leg_params.hip_offset;
  }
  return 0;
}

BLA::Matrix<3, 3> RotateX(float theta) {
  return {1, 0, 0, 0, cos(theta), -sin(theta), 0, sin(theta), cos(theta)};
}

BLA::Matrix<3> ForwardKinematics(BLA::Matrix<3> joint_angles,
                             LegParameters leg_params, uint8_t leg_index) {
  float l1 = leg_params.thigh_length;
  float l2 = leg_params.shank_length;

  float alpha = joint_angles(0);
  float theta = joint_angles(1);
  float phi = joint_angles(2);

  float px = -l1 * sin(theta) - l2 * sin(theta + phi);
  float py = HipOffset(leg_params, leg_index);
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
                              LegParameters leg_params, uint8_t leg_index) {
  float l1 = leg_params.thigh_length;
  float l2 = leg_params.shank_length;

  float alpha = joint_angles(0);
  float theta = joint_angles(1);
  float phi = joint_angles(2);

  float px = -l1 * sin(theta) - l2 * sin(theta + phi);
  float py = HipOffset(leg_params, leg_index);
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

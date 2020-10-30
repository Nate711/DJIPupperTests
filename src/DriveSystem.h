#pragma once

#include <BasicLinearAlgebra.h>

#include "C610Bus.h"
#include "Kinematics.h"
#include "PID.h"
#include "RobotTypes.h"

// Enum for the various control modes: idle, position control, current control
enum class DriveControlMode {
  kIdle,
  kError,  // For robot errors, not coding mistakes.
  kPositionControl,
  kCartesianPositionControl,
  kCurrentControl,
};

// Makes it easier to pass options to the PrintStatus function
struct DrivePrintOptions {
  uint32_t print_delay_millis = 10;
  uint32_t header_delay_millis = 10000;
  bool time = true;
  bool positions = true;
  bool velocities = true;
  bool currents = true;
  bool position_references = true;
  bool velocity_references = true;
  bool current_references = true;
  bool last_current = true;
  char delimiter = '\t';
};

const uint8_t kNumDriveSystemDebugValues = 7*12 + 1;

// Class for controlling the 12 (no more and no less) actuators on Pupper
class DriveSystem {
 public:
  static const size_t kNumActuators = 12;  // TODO something else with this

 private:
  C610Bus<CAN1> front_bus_;
  C610Bus<CAN2> rear_bus_;

  DriveControlMode control_mode_;

  ActuatorPositionVector zero_position_;
  ActuatorPositionVector position_reference_;
  ActuatorVelocityVector velocity_reference_;
  ActuatorCurrentVector current_reference_;

  ActuatorCurrentVector last_commanded_current_;
  ActuatorPositionVector cartesian_position_reference_;
  ActuatorVelocityVector cartesian_velocity_reference_;

  PDGains position_gains_;
  PDGains3x3 cartesian_position_gains_;

  BLA::Matrix<12> ff_force_;

  // Indicates which motors are "active". Those which are inactive get 0
  // torque.
  ActuatorActivations active_mask_;

  // Maximum current for current control and PD mode.
  float max_current_;
  // Maximum commandable current before system triggers a fault. Different
  // than SW saturation current.
  float fault_current_;

  // Max position before system errors out.
  float fault_position_;

  // Max velocity before system errors out.
  float fault_velocity_;

  // Constants defining the robot geometry.
  LegParameters leg_parameters_;
  HipLayoutParameters hip_layout_parameters_;

  // Important direction multipliers
  std::array<float, 12> direction_multipliers_; /* */

  // Initialize the two CAN buses
  void InitializeDrive();

  // Returns enum corresponding to which side the leg is on
  RobotSide LegSide(uint8_t leg_index);

  BLA::Matrix<3> LegFeedForwardForce(uint8_t leg_index);

 public:
  // Construct drive system and initialize CAN buses.
  // Set position and current-control references to zero.
  DriveSystem();

  // Run one iteration through the control loop. Action depends on the current
  // mode.
  void Update();

  // Calculate motor torques for cartesian position control
  BLA::Matrix<12> CartesianPositionControl();

  // Check for messages on the CAN bus and run callbacks.
  void CheckForCANMessages();

  // Check for errors
  DriveControlMode CheckErrors();

  // Go into idle mode, which sends 0A to all motors.
  void SetIdle();

  // Set the measured position to the zero point for the actuators.
  void ZeroCurrentPosition();

  // Set the zero point for all actuators from the provided vector.
  void SetZeroPositions(ActuatorPositionVector zero);

  // Use forward kinematics to determine the body-relative foot position
  // given the joint angles
  // ActuatorPositionVector CartesianPositions(BLA::Matrix<3> joint_angles);

  // 
  ActuatorPositionVector DefaultCartesianPositions();

  // Sets the cartesian reference positions to the position of the leg taken
  // When all joing angles are zero.
  void SetDefaultCartesianPositions();

  // Sets the positions for all twelve actuators.
  void SetJointPositions(ActuatorPositionVector pos);

  // Set position gains all actuators
  void SetPositionKp(float kp);
  void SetPositionKd(float kd);

  // Set the cartesian space stiffness matrix for the foot. 
  void SetCartesianKp3x3(BLA::Matrix<3, 3> kp);

  // Set the cartesian-space damping matrix. 
  void SetCartesianKd3x3(BLA::Matrix<3, 3> kd);

  // Set the reference cartesian positions for the feet.
  // The vel 12-vector argument is expected to be a concatenation
  // of the four individual foot position vectors, ie, 
  // {1x, 1y, 1z, 2x, 2y, 2z, 3x, 3y, 3z, 4x, 4y, 4z}
  void SetCartesianPositions(ActuatorPositionVector pos);

  // Set the reference cartesian velocities for the feet.
  // The vel 12-vector argument is expected to be a concatenation
  // of the four individual foot velocity vectors, ie, 
  // {1x, 1y, 1z, 2x, 2y, 2z, 3x, 3y, 3z, 4x, 4y, 4z}
  void SetCartesianVelocities(ActuatorVelocityVector vel);

  // Set feed forward force.
  void SetFeedForwardForce(BLA::Matrix<12> force);

  // Set current target for actuator i.
  void SetCurrent(uint8_t i, float target_current);

  // Set current level that would trigger a fault.
  void SetFaultCurrent(float fault_current);

  // Set maximum PID and current control torque.
  void SetMaxCurrent(float max_current);

  // Set activations for all the motors simultaneously.
  void SetActivations(ActuatorActivations acts);

  // Send zero torques to the escs.
  void CommandIdle();

  /*
  The ordering of the torques array goes like this:
  front-right abduction
  front-right hip
  front-right knee
  front-left ...
  back-right ...
  back-left ...
  */

  // Send torque commands to C610 escs.
  // The current argument has units amps.
  void CommandCurrents(ActuatorCurrentVector currents);

  // Get the C610 controller object corresponding to index i.
  C610 GetController(uint8_t i);

  // Returns the output shaft's position in [radians].
  float GetActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians].
  ActuatorPositionVector GetActuatorPositions();

  // Returns the output shaft's position in [radians].
  float GetRawActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians].
  ActuatorPositionVector GetRawActuatorPositions();

  // Returns the output shaft's velocity in [radians/s].
  float GetActuatorVelocity(uint8_t i);

  // Returns the motor's actual current in [A]
  float GetActuatorCurrent(uint8_t i);

  // Return the total motor power
  float GetTotalElectricalPower();

  // Return the total motor mechanical power
  float GetTotalMechanicalPower();

  // Returns vector of joint angles for the given leg i.
  // Order is {abductor, hip, knee}
  BLA::Matrix<3> LegJointAngles(uint8_t i);

  // Returns vector of joint angles for the given leg i.
  // Order is {abductor, hip, knee}
  BLA::Matrix<3> LegJointVelocities(uint8_t i);

  // Get the cartesian reference position for leg i.
  BLA::Matrix<3> LegCartesianPositionReference(uint8_t i);

  // Return the cartesian reference velocity for leg i.
  BLA::Matrix<3> LegCartesianVelocityReference(uint8_t i);

  void PrintMsgPackStatus(DrivePrintOptions options);
  // Print drive information to screen
  void PrintStatus(DrivePrintOptions options);

  // Print a header for the messages
  void PrintHeader(DrivePrintOptions options);

  BLA::Matrix<kNumDriveSystemDebugValues> DebugData();
};
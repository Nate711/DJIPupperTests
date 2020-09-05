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
};

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

 public:
  ActuatorPositionVector cartesian_position_reference_;

 private:
  ActuatorVelocityVector cartesian_velocity_reference_;

  PDGains position_gains_;
  PDGains3x3 cartesian_position_gains_;

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

  // Constants specific to the C610 + M2006 setup.
  static constexpr float kReduction = 36.0F;
  static constexpr float kCountsPerRad =
      C610::COUNTS_PER_REV * kReduction / (2 * M_PI);
  static constexpr float kRPMPerRadS = kReduction * 2.0F * M_PI / 60.0F;
  static constexpr float kMilliAmpPerAmp = 1000.0F;

  // Constants defining the robot geometry.
  LegParameters leg_parameters_;
  HipLayoutParameters hip_layout_parameters_;

  // Important direction multipliers
  std::array<float, 12> direction_multipliers_; /* */

  // Initialize the two CAN buses
  void InitializeDrive();

  // Returns enum corresponding to which side the leg is on
  RobotSide LegSide(uint8_t leg_index);

  // Returns the position of the leg relative to the center of the body
  BLA::Matrix<3> HipPosition(uint8_t i);

 public:
  // Construct drive system and initialize CAN buses.
  // Set position and current-control references to zero.
  DriveSystem();

  // Run one iteration through the control loop. Action depends on the current
  // mode.
  void Update();

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

  ActuatorPositionVector CartesianPositions(BLA::Matrix<3> joint_angles);

  ActuatorPositionVector DefaultCartesianPositions();

  // Sets the cartesian reference positions to the position of the leg taken
  // When all joing angles are zero.
  void SetDefaultCartesianPositions();

  // Sets the positions for all twelve actuators.
  void SetAllPositions(ActuatorPositionVector pos);

  // Set the position target for actuator i
  // Note that the current commanded to the motor controller is
  // only updated when Update() is called
  void SetPosition(uint8_t i, float target_position);

  // Set position gains all actuators
  void SetPositionKp(float kp);
  void SetPositionKd(float kd);

  // Set position gains for all actuators
  void SetPositionGains(PDGains gains);

  void SetCartesianKp(BLA::Matrix<3> kp);

  void SetCartesianKd(BLA::Matrix<3> kd);

  void SetCartesianKp3x3(BLA::Matrix<3, 3> kp);

  void SetCartesianKd3x3(BLA::Matrix<3, 3> kd);

  void SetCartesianPositions(ActuatorPositionVector pos);

  void SetCartesianVelocities(ActuatorVelocityVector vel);

  // Set current target for actuator i
  void SetCurrent(uint8_t i, float target_current);

  // Set current level that would trigger a fault
  void SetFaultCurrent(float fault_current);

  // Set maximum PID and current control torque
  void SetMaxCurrent(float max_current);

  // Activates an actuator. Deactive actuators will be commanded 0 amps.
  void ActivateActuator(uint8_t i);

  // Deactivate an actuator.
  void DeactivateActuator(uint8_t i);

  void SetActivations(ActuatorActivations acts);

  // Activates all twelve actuators.
  void ActivateAll();

  // Deactivates all twelve actuators.
  void DeactivateAll();

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

  // Send torque commands to C610 escs. Converts from decimal value of amps to
  // integer milliamps.
  void CommandCurrents(
      ActuatorCurrentVector currents);  // passing "torques" by reference
                                        // enforces that it have 12 elements

  // Get the C610 controller object corresponding to index i.
  C610 GetController(uint8_t i);

  // Returns the output shaft's position in [radians].
  float GetActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians]
  ActuatorPositionVector GetActuatorPositions();

  // Returns the output shaft's position in [radians].
  float GetRawActuatorPosition(uint8_t i);

  // Returns all output shaft positions [radians]
  ActuatorPositionVector GetRawActuatorPositions();

  // Returns the output shaft's velocity in [radians/s].
  float GetActuatorVelocity(uint8_t i);

  // Returns the motor's actual current in [A]
  float GetActuatorCurrent(uint8_t i);

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

  // Print drive information to screen
  void PrintStatus(DrivePrintOptions options);

  // Print a header for the messages
  void PrintHeader(DrivePrintOptions options);
};
#pragma once

#include "C610Bus.h"
#include "PID.h"
#include "RobotTypes.h"

// Enum for the various control modes: idle, position control, current control
enum class DriveControlMode
{
    kIdle,
    kError, // For robot errors, not coding mistakes.
    kPositionControl,
    kComplianceControl,
    kCurrentControl,
};

// Makes it easier to pass options to the PrintStatus function
struct DrivePrintOptions
{
    uint32_t print_delay_millis = 10;
    uint32_t header_delay_millis = 10000;
    bool time = true;
    bool positions = true;
    bool velocities = true;
    bool currents = true;
    bool position_references = true;
    bool velocity_references = true;
    bool current_references = true;
};

// Class for controlling the 12 (no more and no less) actuators on Pupper
class DriveSystem
{
public:
    static const uint8_t kNumActuators = 12;

private:
    C610Bus<CAN1> front_bus_;
    C610Bus<CAN2> rear_bus_;

    DriveControlMode control_mode_;

    float position_reference_[kNumActuators];
    float velocity_reference_[kNumActuators];

    PDGains position_gains_[kNumActuators];

    // Holder for current control mode. Otherwise not used.
    float current_reference_[kNumActuators];

    // Indicates which motors are "active". Those which are inactive get 0 torque.
    bool active_mask_[kNumActuators];

    // Maximum current for current control and PD mode.
    float max_current_;

    // Maximum commandable current before system triggers a fault. Different than SW saturation current.
    float fault_current_;

    static constexpr float kReduction = 36.0F;
    static constexpr float kCountsPerRad = C610::COUNTS_PER_REV * kReduction / (2 * M_PI);
    static constexpr float kRPMPerRadS = kReduction * 2.0F * M_PI / 60.0F;
    static constexpr float kMilliAmpPerAmp = 1000.0F;

    // Initialize the two CAN buses
    void InitializeDrive();

public:
    // Construct drive system and initialize CAN buses.
    // Set position and current-control references to zero.
    DriveSystem();

    // Run one iteration through the control loop. Action depends on the current mode.
    void Update();

    // Check for messages on the CAN bus and run callbacks.
    void CheckForCANMessages();

    // Go into idle mode, which sends 0A to all motors.
    void SetIdle();

    // Sets the positions for all twelve actuators.
    void SetAllPositions(ActuatorPositionVector pos);

    // Set the position target for actuator i
    // Note that the current commanded to the motor controller is
    // only updated when Update() is called
    void SetPosition(uint8_t i, float target_position);

    // Set position gains for actuator i
    void SetPositionKp(uint8_t i, float kp);
    void SetPositionKd(uint8_t i, float kd);

    // Set position gains for all actuators
    void SetAllPositionGains(PDGains gains);
    void SetAllPositionKp(float kp);
    void SetAllPositionKd(float kd);

    // Set current target for actuator i
    void SetCurrent(uint8_t i, float target_current);

    // Set current level that would trigger a fault
    void SetFaultCurrent(float fault_current);

    // Set maximum PID and current control torque
    void SetMaxCurrent(float max_current);

    // Activates an actuator. Deactive actuators will be commanded 0 amps.
    void ActivateActuator(uint8_t i);

    // Deactivate an actuator.
    void DeactivateActuator(uint8_t);

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

    // Send torque commands to C610 escs. Converts from decimal value of amps to integer milliamps.
    void CommandCurrents(const float (&currents)[kNumActuators]); // passing "torques" by reference enforces that it have 12 elements

    // Get the C610 controller object corresponding to index i.
    C610 GetController(uint8_t i);

    // Returns the output shaft's position in [radians].
    float GetActuatorPosition(uint8_t i);

    // Returns the output shaft's velocity in [radians/s].
    float GetActuatorVelocity(uint8_t i);

    // Returns the motor's actual current in [A]
    float GetActuatorCurrent(uint8_t i);

    // Get the C610Bus object for the front actuators.
    C610Bus<CAN1> &FrontBus();

    // Get the C610Bus object for the rear actuators.
    C610Bus<CAN2> &RearBus();

    // Print drive information to screen
    void PrintStatus(DrivePrintOptions options);

    // Print a header for the messages
    void PrintHeader(DrivePrintOptions options);
};
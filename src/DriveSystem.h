#pragma once
#include "C610Bus.h"

enum class DriveControlMode {
    kIdle,
    kPositionControl,
    kCurrentControl,
};

class DriveSystem
{
public:
    static const uint8_t kNumActautors = 12;

private:
    C610Bus<CAN1> front_bus_;
    C610Bus<CAN2> rear_bus_;

    DriveControlMode control_mode_;

    float position_target_[kNumActautors];
    float position_kp_[kNumActautors];
    float position_kd_[kNumActautors];

    static constexpr float kReduction = 36.0F;
    static constexpr float kCountsPerRad = C610::COUNTS_PER_REV * kReduction / (2 * M_PI);
    static constexpr float kRPMPerRadS = kReduction * 2.0F * M_PI / 60.0F;
    static constexpr float kMilliAmpPerAmp = 1000.0F;

    void InitializeDrive();

public:
    // Construct drive system and initialize CAN buses.
    DriveSystem();

    // Run one iteration through the control loop. Action depends on the current mode.
    void Update();

    // Set the position target for actuator i
    // Note that the current commanded to the motor controller is 
    // only updated when Update() is called
    void SetPosition(uint8_t i, float target_position);

    // Set position gains for actuator i
    void SetPositionGains(uint8_t i, float kp, float kd);

    // Set position gains for all actuators 
    void SetUniformPositionGains(uint8_t i, float kp, float kd);
    
    // Set current target for actuator i
    void SetCurrent(uint8_t i, float target_torque);

    /*
    The ordering of the torques array goes like this:
    front-right abduction
    front-right hip
    front-right knee
    front-left ...
    back-right ...
    back-left ...
    */
    // TODO: pass in float values and do conversion inside
    void CommandTorques(const int32_t (&torques)[kNumActautors]); // passing "torques" by reference enforces that it have 12 elements

    // Get the C610 controller object corresponding to index i.
    C610 GetController(uint8_t i);

    // Returns the output shaft's position in [radians].
    float GetActuatorPositions(uint8_t i);

    // Returns the output shaft's velocity in [radians/s].
    float GetActuatorVelocity(uint8_t i);

    // Returns the motor's actual current in [A]
    float GetActuatorCurrent(uint8_t i);

    // Get the C610Bus object for the front actuators.
    C610Bus<CAN1> FrontBus();

    // Get the C610Bus object for the rear actuators.
    C610Bus<CAN2> RearBus();
};
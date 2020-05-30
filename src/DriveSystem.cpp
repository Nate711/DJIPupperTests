#include "DriveSystem.h"

DriveSystem::DriveSystem() : front_bus_(), rear_bus_()
{
    control_mode_ = DriveControlMode::kIdle;
    InitializeDrive();
}

void DriveSystem::InitializeDrive()
{
    rear_bus_.initializeCAN();
    front_bus_.initializeCAN();

    // Must call the following lines in a static setting
    // drive_system.rearBus().can().onReceive([](const CAN_message_t &msg) { drive_system.rearBus().callback(msg); });
    // drive_system.frontBus().onReceive([](const CAN_message_t &msg) { drive_system.frontBus().callback(msg); });
}

void DriveSystem::SetPosition(uint8_t i, float position_target)
{
    control_mode_ = DriveControlMode::kPositionControl;
    position_target_[i] = position_target;
}

void DriveSystem::SetPositionGains(uint8_t i, float kp, float kd)
{
    position_kp_[i] = kp;
    position_kd_[i] = kd;
}

void DriveSystem::SetUniformPositionGains(uint8_t i, float kp, float kd)
{
    for (uint8_t i = 0; i < kNumActautors; i++)
    {
        SetPositionGains(i, kp, kd);
    }
}

void DriveSystem::Update()
{
    switch (control_mode_)
    {
    case DriveControlMode::kIdle:
    {
        break;
    }
    case DriveControlMode::kPositionControl:
    {
        break;
    }
    case DriveControlMode::kCurrentControl:
    {
        break;
    }
    }
}

C610Bus<CAN1> DriveSystem::FrontBus()
{
    return front_bus_;
}

C610Bus<CAN2> DriveSystem::RearBus()
{
    return rear_bus_;
}

void DriveSystem::CommandTorques(const int32_t (&torques)[DriveSystem::kNumActautors])
{
    // TODO: sanity checks
    front_bus_.commandTorques(torques[0], torques[1], torques[2], torques[3], 0);
    front_bus_.commandTorques(torques[4], torques[5], 0, 0, 1);
    rear_bus_.commandTorques(torques[6], torques[7], torques[8], torques[9], 0);
    rear_bus_.commandTorques(torques[10], torques[11], 0, 0, 1);
}

C610 DriveSystem::GetController(uint8_t i)
{
    if (i >= 0 && i <= 5)
    {
        return front_bus_.get(i);
    }
    else if (i >= 6 && i <= 11)
    {
        return rear_bus_.get(i - 6);
    }
    else
    {
        Serial.println("Invalid actuator index. Must be 0<=i<=11.");
        // Should not be used
        // Maybe should error out here
        return C610();
    }
}

float DriveSystem::GetActuatorPositions(uint8_t i)
{
    return GetController(i).counts() / kCountsPerRad;
}

float DriveSystem::GetActuatorVelocity(uint8_t i)
{
    return GetController(i).rpm() / kRPMPerRadS;
}

float DriveSystem::GetActuatorCurrent(uint8_t i)
{
    return GetController(i).torque() / kMilliAmpPerAmp;
}
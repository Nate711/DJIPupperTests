#include "DriveSystem.h"

// TODO Put in a array helper class?
template <class T, int N>
void FillZeros(T (&array)[N])
{
    for (int i = 0; i < N; i++)
    {
        array[i] = T(0);
    }
}

DriveSystem::DriveSystem() : front_bus_(), rear_bus_()
{
    control_mode_ = DriveControlMode::kIdle;
    InitializeDrive();
    FillZeros(position_reference_);
    FillZeros(velocity_reference_);
    FillZeros(current_reference_);
}

void DriveSystem::InitializeDrive()
{
    rear_bus_.initializeCAN();
    front_bus_.initializeCAN();

    // Must call the following lines in a static setting
    // drive_system.rearBus().can().onReceive([](const CAN_message_t &msg) { drive_system.rearBus().callback(msg); });
    // drive_system.frontBus().onReceive([](const CAN_message_t &msg) { drive_system.frontBus().callback(msg); });
}

void DriveSystem::CheckForCANMessages()
{
    front_bus_.pollCAN();
    rear_bus_.pollCAN();
}

void DriveSystem::SetIdle()
{
    control_mode_ = DriveControlMode::kIdle;
}

void DriveSystem::SetPosition(uint8_t i, float position_reference)
{
    control_mode_ = DriveControlMode::kPositionControl;
    position_reference_[i] = position_reference;
}

void DriveSystem::SetPositionGains(uint8_t i, float kp, float kd)
{
    position_gains_[i].kp = kp;
    position_gains_[i].kd = kd;
}

void DriveSystem::SetUniformPositionGains(uint8_t i, float kp, float kd)
{
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        SetPositionGains(i, kp, kd);
    }
}

void DriveSystem::SetCurrent(uint8_t i, float current_reference)
{
    control_mode_ = DriveControlMode::kCurrentControl;
    current_reference_[i] = current_reference;
}

void DriveSystem::Update()
{
    switch (control_mode_)
    {
    case DriveControlMode::kIdle:
    {
        CommandIdle();
        break;
    }
    case DriveControlMode::kPositionControl:
    {
        float pid_current[kNumActuators];
        for (uint8_t i = 0; i < kNumActuators; i++)
        {
            PD(pid_current[i], GetActuatorPosition(i), GetActuatorVelocity(i), position_reference_[i], velocity_reference_[i], position_gains_[i]);
        }
        CommandCurrents(pid_current);
        break;
    }
    case DriveControlMode::kCurrentControl:
    {
        CommandCurrents(current_reference_);
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

void DriveSystem::CommandIdle()
{
    float currents[kNumActuators];
    FillZeros(currents);
    CommandCurrents(currents);
}

// TODO put this somewhere else
template <int N>
void DriveSystem::CurrentConversion(int32_t (&out)[N], const float (&in)[N])
{
    for (int i = 0; i < N; i++)
    {
        out[i] = in[i] * kMilliAmpPerAmp;
    }
}

void DriveSystem::CommandCurrents(const float (&currents)[DriveSystem::kNumActuators])
{
    // TODO: sanity checks
    int32_t currents_mA[kNumActuators];
    CurrentConversion(currents_mA, currents);
    front_bus_.commandTorques(currents_mA[0], currents_mA[1], currents_mA[2], currents_mA[3], 0);
    front_bus_.commandTorques(currents_mA[4], currents_mA[5], 0, 0, 1);
    rear_bus_.commandTorques(currents_mA[6], currents_mA[7], currents_mA[8], currents_mA[9], 0);
    rear_bus_.commandTorques(currents_mA[10], currents_mA[11], 0, 0, 1);
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

float DriveSystem::GetActuatorPosition(uint8_t i)
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

void DriveSystem::PrintStatus(DrivePrintOptions options)
{
    char delimiter = '\t';
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        if (options.time)
        {
            Serial.print(millis());
            Serial.print(delimiter);
        }
        if (options.positions)
        {
            Serial.print(GetActuatorPosition(i), 3);
            Serial.print(delimiter);
        }
        if (options.velocities)
        {
            Serial.print(GetActuatorVelocity(i), 1);
            Serial.print(delimiter);
        }
        if (options.currents)
        {
            Serial.print(GetActuatorCurrent(i), 3);
            Serial.print(delimiter);
        }
        if (options.position_references)
        {
            Serial.print(position_reference_[i], 3);
            Serial.print(delimiter);
        }
        if (options.velocity_references)
        {
            Serial.print(velocity_reference_[i], 1);
            Serial.print(delimiter);
        }
        if (options.current_references)
        {
            Serial.print(current_reference_[i], 3);
            Serial.print(delimiter);
        }
    }
    Serial.println();
}
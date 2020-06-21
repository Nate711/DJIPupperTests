#include "DriveSystem.h"
#include <Streaming.h>

// TODO: Make it work with const arrays
template <class T, int N>
T Maximum(const T (&array)[N])
{
    T max = array[0];
    for (int i = 0; i < N; i++)
    {
        max = array[i] >= max ? array[i] : max;
    }
    return max;
}

template <class T, int N>
T Minimum(const T (&array)[N])
{
    T min = array[0];
    for (int i = 0; i < N; i++)
    {
        min = array[i] <= min ? array[i] : min;
    }
    return min;
}

template <class T, int N>
void Constrain(T (&array)[N], T min, T max)
{
    for (int i = 0; i < N; i++)
    {
        array[i] = (array[i] <= min) ? min : (array[i] >= max ? max : array[i]);
    }
}

template <class T, int N>
void MaskArray(T (&array)[N], const bool (&mask)[N])
{
    for (int i = 0; i < N; i++)
    {
        array[i] = mask[i] ? array[i] : T(0);
    }
}

// TODO put this somewhere else
template <int N>
void ConvertToFixedPoint(int32_t (&out)[N], const float (&in)[N], float factor)
{
    for (int i = 0; i < N; i++)
    {
        out[i] = in[i] * factor;
    }
}

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
    fault_current_ = 10.0; // TODO: don't make this so high at default
    max_current_ = 0.0;    // TODO: make this a parameter
    FillZeros(position_reference_);
    FillZeros(velocity_reference_);
    FillZeros(current_reference_);
    FillZeros(active_mask_);
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

void DriveSystem::SetAllPositions(ActuatorPositionVector pos)
{
    if (pos.size() != kNumActuators)
    {
        Serial.println("Error. Position command vector does not have the same size as the drive system");
        return;
    }
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        position_reference_[i] = pos[i];
    }
}

void DriveSystem::SetPosition(uint8_t i, float position_reference)
{
    control_mode_ = DriveControlMode::kPositionControl;
    position_reference_[i] = position_reference;
}

void DriveSystem::SetPositionKp(uint8_t i, float kp)
{
    position_gains_[i].kp = kp;
}

void DriveSystem::SetPositionKd(uint8_t i, float kd)
{
    position_gains_[i].kd = kd;
}

void DriveSystem::SetAllPositionKp(float kp)
{
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        SetPositionKp(i, kp);
    }
}

void DriveSystem::SetAllPositionKd(float kd)
{
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        SetPositionKd(i, kd);
    }
}

void DriveSystem::SetAllPositionGains(PDGains gains)
{
    SetAllPositionKp(gains.kp);
    SetAllPositionKd(gains.kd);
}

void DriveSystem::SetCurrent(uint8_t i, float current_reference)
{
    control_mode_ = DriveControlMode::kCurrentControl;
    current_reference_[i] = current_reference;
}

void DriveSystem::SetFaultCurrent(float fault_current)
{
    fault_current_ = fault_current;
}

void DriveSystem::SetMaxCurrent(float max_current)
{
    max_current_ = max_current;
}

// TODO: Add saturation to PID!
void DriveSystem::Update()
{
    switch (control_mode_)
    {
    case DriveControlMode::kError:
    {
        // TODO: Add some sort of error handling.
        CommandIdle();
        break;
    }
    case DriveControlMode::kIdle:
    {
        CommandIdle();
        break;
    }
    case DriveControlMode::kPositionControl:
    {
        float pd_current[kNumActuators];
        for (uint8_t i = 0; i < kNumActuators; i++)
        {
            PD(pd_current[i], GetActuatorPosition(i), GetActuatorVelocity(i), position_reference_[i], velocity_reference_[i], position_gains_[i]);
        }
        CommandCurrents(pd_current);
        break;
    }
    case DriveControlMode::kComplianceControl:
    {
        // TODO Implement arbitrary axis compliance, but probably just xyz
        // This would need kinematic knowledge however, so maybe this code should not belong here?
        CommandIdle(); // Placeholder
        break;
    }
    case DriveControlMode::kCurrentControl:
    {
        CommandCurrents(current_reference_);
        break;
    }
    }
}

C610Bus<CAN1> &DriveSystem::FrontBus()
{
    return front_bus_;
}

C610Bus<CAN2> &DriveSystem::RearBus()
{
    return rear_bus_;
}

// TODO: make the activation simpler
void DriveSystem::ActivateActuator(uint8_t i)
{
    active_mask_[i] = 1;
}

void DriveSystem::DeactivateActuator(uint8_t i)
{
    active_mask_[i] = 0;
}

void DriveSystem::ActivateAll()
{
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        ActivateActuator(i);
    }
}

void DriveSystem::DeactivateAll()
{
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        DeactivateActuator(i);
    }
}

void DriveSystem::CommandIdle()
{
    float currents[kNumActuators];
    FillZeros(currents);
    CommandCurrents(currents);
}

void DriveSystem::CommandCurrents(const float (&currents)[kNumActuators])
{
    // TODO: make this copy less stupid
    float current_command[kNumActuators];
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        current_command[i] = currents[i];
    }

    Constrain(current_command, -max_current_, max_current_);
    // TODO: kind of redundant with constrain right above
    if (Maximum(current_command) > fault_current_ || Minimum(current_command) < -fault_current_)
    {
        Serial << "Requested current too large. Erroring out." << endl;
        control_mode_ = DriveControlMode::kError;
        return;
    }

    MaskArray(current_command, active_mask_);
    int32_t currents_mA[kNumActuators];
    ConvertToFixedPoint(currents_mA, current_command, kMilliAmpPerAmp);

    front_bus_.commandTorques(currents_mA[0], currents_mA[1], currents_mA[2], currents_mA[3], 0);
    front_bus_.commandTorques(currents_mA[4], currents_mA[5], 0, 0, 1);
    rear_bus_.commandTorques(currents_mA[6], currents_mA[7], currents_mA[8], currents_mA[9], 0);
    rear_bus_.commandTorques(currents_mA[10], currents_mA[11], 0, 0, 1);
}

C610 DriveSystem::GetController(uint8_t i)
{
    // TODO put these constants somewhere else
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
        Serial << "Invalid actuator index. Must be 0<=i<=11." << endl;
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

void DriveSystem::PrintHeader(DrivePrintOptions options)
{
    char delimiter = '\t';
    if (options.time)
    {
        Serial << "T" << delimiter;
    }
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        if (!active_mask_[i])
            continue;
        if (options.positions)
        {
            Serial << "p[" << i << "]" << delimiter;
        }
        if (options.velocities)
        {
            Serial << "v[" << i << "]" << delimiter;
        }
        if (options.currents)
        {
            Serial << "I[" << i << "]" << delimiter;
        }
        if (options.position_references)
        {
            Serial << "pr[" << i << "]" << delimiter;
        }
        if (options.velocity_references)
        {
            Serial << "vr[" << i << "]" << delimiter;
        }
        if (options.current_references)
        {
            Serial << "Ir[" << i << "]" << delimiter;
        }
    }
    Serial << endl;
}

void DriveSystem::PrintStatus(DrivePrintOptions options)
{
    char delimiter = '\t';
    if (options.time)
    {
        Serial << millis() << delimiter;
    }
    for (uint8_t i = 0; i < kNumActuators; i++)
    {
        if (!active_mask_[i])
            continue;
        if (options.positions)
        {
            Serial.print(GetActuatorPosition(i), 3);
            Serial << delimiter;
        }
        if (options.velocities)
        {
            Serial.print(GetActuatorVelocity(i), 1);
            Serial << delimiter;
        }
        if (options.currents)
        {
            Serial.print(GetActuatorCurrent(i), 3);
            Serial << delimiter;
        }
        if (options.position_references)
        {
            Serial.print(position_reference_[i], 3);
            Serial << delimiter;
        }
        if (options.velocity_references)
        {
            Serial.print(velocity_reference_[i], 1);
            Serial << delimiter;
        }
        if (options.current_references)
        {
            Serial.print(current_reference_[i], 3);
            Serial << delimiter;
        }
    }
    Serial << endl;
}
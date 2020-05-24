#include "C610.h"
#include <FlexCAN_T4.h>

C610::C610(const int32_t counts_per_rev)
{
    _counts_per_rev = counts_per_rev;
    _initialized_mechanical_angle = false;
    _rotations = 0;
    _last_pos_measurement = 0;
    _counts = 0;
    _rpm = 0;
    _torque = 0;
}

void C610::torqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower)
{
    upper = (torque >> 8) & 0xFF;
    lower = torque & 0xFF;
}

void C610::interpretMessage(const CAN_message_t &msg, int32_t &pos, int32_t &vel, int32_t &torque)
{
    pos = uint16_t((msg.buf[0] << 8) | msg.buf[1]);
    vel = int16_t((msg.buf[2] << 8) | msg.buf[3]);
    torque = int16_t((msg.buf[4] << 8) | msg.buf[5]);
}

int32_t C610::counts()
{
    return _counts;
}

int32_t C610::rpm()
{
    return _rpm;
}

int32_t C610::torque()
{
    return _torque;
}

void C610::updateState(const int32_t pos_measurement, const int32_t velocity_measurement, const int32_t torque_measurement)
{
    // Initial setup
    if (!_initialized_mechanical_angle)
    {
        _initialized_mechanical_angle = true;
        _last_pos_measurement = pos_measurement;
    }

    // Position
    int32_t delta = pos_measurement - _last_pos_measurement;
    if (delta > _counts_per_rev / 2)
    { // Crossed from >= 0 counts to <= 8191 counts. Could also trigger if spinning super fast (>2000rps)
        _rotations -= 1;
    }
    else if (delta < -_counts_per_rev / 2)
    { // Crossed from <= 8191 counts to >= 0 counts. Could also trigger if spinning super fast (>2000rps)
        _rotations += 1;
    }
    _counts = _rotations * _counts_per_rev + pos_measurement;
    _last_pos_measurement = pos_measurement;

    // Velocity
    _rpm = velocity_measurement;

    // Torque
    _torque = torque_measurement;
}
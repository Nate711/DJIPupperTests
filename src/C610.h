#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

class C610
{
private:
    int32_t _counts_per_rev;
    uint8_t _initialized_mechanical_angle;
    int32_t _rotations;
    int32_t _last_pos_measurement;
    int32_t _counts;
    int32_t _rpm;
    int32_t _torque;

public:
    static void torqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower);
    static void interpretMessage(const CAN_message_t &msg, int32_t &pos, int32_t &vel, int32_t &torque);
    
    C610(int32_t counts_per_rev = 8192);
    void updateState(const int32_t pos_measurement, const int32_t velocity_measurement, const int32_t torque_measurement);
    int32_t counts();
    int32_t rpm();
    int32_t torque();
};

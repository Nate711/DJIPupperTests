#pragma once

#include <Arduino.h>
#include <C610.h>
#include <FlexCAN_T4.h>

template <CAN_DEV_TABLE _bus = CAN1>
class C610Bus
{
public:
    static const uint8_t MAX_PER_CAN = 8;

private:
    static const uint32_t ZERO_TO_THREE_COMMAND_ID = 0x200;
    static const uint32_t FOUR_TO_SEVEN_COMMAND_ID = 0x1FF;
    static const uint32_t RECEIVE_BASE_ID = 0x200;

    C610 _controllers[MAX_PER_CAN];
    FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16> _can;

public:
    C610Bus(void);
    void pollCAN();
    void callback(CAN_message_t &msg);
    void commandTorques(int32_t torque0, int32_t torque1 = 0, int32_t torque2 = 0, int32_t torque3 = 0, uint8_t subbus = 0);
    void initializeCANBus();
    C610& get(uint8_t i);
};

#include "C610Bus.tpp"
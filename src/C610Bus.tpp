#include "C610Bus.h"

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::pollCAN()
{
    _can.events();
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::initializeCANBus()
{
    _can.begin();
    _can.setBaudRate(1000000);
    _can.setMaxMB(16);
    _can.enableFIFO();
    _can.enableFIFOInterrupt();
    // onReceive expectes a function pointer, a member function isn't one since it has 
    // an implicit this parameter
    // FAQ says it's illegal to bind member function to void *!
    // _can.onReceive([this](CAN_message_t msg){this->callback(msg);});
    _can.mailboxStatus();
}

template <CAN_DEV_TABLE _bus>
C610Bus<_bus>::C610Bus(void)
{
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::callback(CAN_message_t &msg)
{
    if (msg.id >= RECEIVE_BASE_ID + 1 && msg.id <= RECEIVE_BASE_ID + MAX_PER_CAN)
    {
        uint8_t esc_index = msg.id - RECEIVE_BASE_ID - 1; // ESC 1 corresponds to index 0
        if (esc_index >= 0 && esc_index <= 7)
        {
            int32_t pos, velocity, torque;
            C610::interpretMessage(msg, pos, velocity, torque);
            _controllers[esc_index].updateState(pos, velocity, torque);
        }
        else
        {
            Serial.println("Invalid esc index in CAN message");
            return;
        }
    }
    else
    {
        Serial.print("Invalid ID for feedback message: ");
        Serial.print(msg.id);
        Serial.println();
        return;
    }
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::commandTorques(int32_t torque0, int32_t torque1, int32_t torque2, int32_t torque3, uint8_t subbus)
{
    // IDs 0 through 3 go on ID 0x200
    // IDs 4 through 7 go on ID 0x1FF

    int16_t t0 = constrain(torque0, -32000, 32000); // prevent overflow of int16_t
    int16_t t1 = constrain(torque1, -32000, 32000);
    int16_t t2 = constrain(torque2, -32000, 32000);
    int16_t t3 = constrain(torque3, -32000, 32000);

    CAN_message_t msg;
    if (subbus == 0)
    {
        msg.id = ZERO_TO_THREE_COMMAND_ID;
    }
    else if (subbus == 1)
    {
        msg.id = FOUR_TO_SEVEN_COMMAND_ID;
    }
    else
    {
        Serial.print("Invalid ESC subbus: ");
        Serial.println(subbus);
        return;
    }
    C610::torqueToBytes(t0, msg.buf[0], msg.buf[1]);
    C610::torqueToBytes(t1, msg.buf[2], msg.buf[3]);
    C610::torqueToBytes(t2, msg.buf[4], msg.buf[5]);
    C610::torqueToBytes(t3, msg.buf[6], msg.buf[7]);
    _can.write(msg);
}

template <CAN_DEV_TABLE _bus>
C610& C610Bus<_bus>::get(uint8_t i)
{
    return _controllers[i];
}
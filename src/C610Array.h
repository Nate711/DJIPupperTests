#include <C610.h>
class C610Array
{
private:
    static const uint8_t NUM_C610S = 8;
    static const uint32_t ZERO_TO_THREE_COMMAND_ID = 0x200;
    static const uint32_t FOUR_TO_SEVEN_COMMAND_ID = 0x1FF;
    static const uint32_t RECEIVE_BASE_ID = 0x200;

    C610 _controllers0[NUM_C610S];                   // controllers on 1st can bus (could be CAN1 or CAN2)
    C610 _controllers1[NUM_C610S];                   // controllers on 2nd can bus
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> _can0; // TODO big potential issue: revert this to CAN1 later
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> _can1;

public:
    C610Array();
    void pollCAN();
    void callback0(CAN_message_t &msg);
    void callback1(CAN_message_t &msg);
    void callback(CAN_message_t &msg, uint8_t bus);
    void commandTorques(int32_t torque0, int32_t torque1 = 0, int32_t torque2 = 0, int32_t torque3 = 0, uint8_t subbus = 0);
    void initializeCANBus();
};

void C610Array::pollCAN()
{
    _can0.events();
    _can1.events();
}

void C610Array::initializeCANBus()
{
    _can0.begin();
    _can0.setBaudRate(1000000);
    _can0.setMaxMB(16);
    _can0.enableFIFO();
    _can0.enableFIFOInterrupt();
    _can0.onReceive(callback0);
    _can0.mailboxStatus();

    _can1.begin();
    _can1.setBaudRate(1000000);
    _can1.setMaxMB(16);
    _can1.enableFIFO();
    _can1.enableFIFOInterrupt();
    _can1.onReceive(callback1);
    _can1.mailboxStatus();
}

C610Array::C610Array()
{
}

void C610Array::callback0(CAN_message_t &msg)
{
    callback(msg, 0);
}
void C610Array::callback1(CAN_message_t &msg)
{
    callback(msg, 1);
}

void C610Array::callback(CAN_message_t &msg, uint8_t network)
{
    if (msg.id >= RECEIVE_BASE_ID + 1 && msg.id <= RECEIVE_BASE_ID + NUM_C610S)
    {
        uint8_t esc_index = msg.id - RECEIVE_BASE_ID - 1; // ESC 1 corresponds to index 0
        if (esc_index >= 0 && esc_index <= 7)
        {
            int32_t pos, velocity, torque;
            C610::interpretMessage(msg, pos, velocity, torque);
            if (network == 0)
            {
                _controllers0[esc_index].updateState(pos, velocity, torque);
            }
            else
            {
                _controllers1[esc_index].updateState(pos, velocity, torque);
            }
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

void C610Array::commandTorques(int32_t torque0, int32_t torque1 = 0, int32_t torque2, int32_t torque3, uint8_t subbus)
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
    _can0.write(msg);
}
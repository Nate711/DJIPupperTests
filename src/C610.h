#define NUM_C610S 8
const uint32_t ZERO_TO_THREE_COMMAND_ID = 0x200;
const uint32_t FOUR_TO_SEVEN_COMMAND_ID = 0x200;
const uint32_t RECEIVE_BASE_ID = 0x200;

/////////////// OUTWARD ///////////////
void torqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower)
{
    upper = (torque >> 8) & 0xFF;
    lower = torque & 0xFF;
}

void sendTorqueCommand(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &can_bus, int32_t torque0, int32_t torque1 = 0, int32_t torque2 = 0, int32_t torque3 = 0, uint8_t subbus = 0)
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
    torqueToBytes(t0, msg.buf[0], msg.buf[1]);
    torqueToBytes(t1, msg.buf[2], msg.buf[3]);
    torqueToBytes(t2, msg.buf[4], msg.buf[5]);
    torqueToBytes(t3, msg.buf[6], msg.buf[7]);
    can_bus.write(msg);
}

////////////// INWARD /////////////
void interpretC610Message(const CAN_message_t &msg, int32_t &pos, int32_t &vel, int32_t &torque)
{
    pos = uint16_t((msg.buf[0] << 8) | msg.buf[1]);
    vel = int16_t((msg.buf[2] << 8) | msg.buf[3]);
    torque = int16_t((msg.buf[4] << 8) | msg.buf[5]);
}

struct MotorState
{
    // Position
    int32_t counts_per_rev;
    uint8_t initialized_mechanical_angle;
    int32_t rotations;
    int32_t last_pos_measurement;
    int32_t counts;

    // Velocity
    int32_t velocity;
    // Torque
    int32_t torque;
};

void initializeMotorState(MotorState &ms, int32_t counts_per_rev = 8192)
{
    ms.counts_per_rev = counts_per_rev;
    ms.initialized_mechanical_angle = false;
    ms.rotations = 0;
    ms.last_pos_measurement = 0;
    ms.counts = 0;
    ms.velocity = 0;
    ms.torque = 0;
}

void initializeMotorStates(MotorState states[], uint8_t num_escs, int32_t counts_per_rev = 8192)
{
    for (uint8_t i = 0; i < num_escs; i++)
    {
        initializeMotorState(states[i], counts_per_rev);
    }
}

void updateMotorState(MotorState &ms, int32_t pos_measurement, int32_t velocity_measurement, int32_t torque_measurement)
{
    // Initial setup
    if (!ms.initialized_mechanical_angle)
    {
        ms.initialized_mechanical_angle = true;
        ms.last_pos_measurement = pos_measurement;
    }

    // Position
    int32_t delta = int32_t(pos_measurement) - int32_t(ms.last_pos_measurement);
    if (delta > ms.counts_per_rev / 2)
    { // Crossed from >= 0 counts to <= 8191 counts. Could also trigger if spinning super fast (>2000rps)
        ms.rotations -= 1;
    }
    else if (delta < -ms.counts_per_rev / 2)
    { // Crossed from <= 8191 counts to >= 0 counts. Could also trigger if spinning super fast (>2000rps)
        ms.rotations += 1;
    }
    ms.counts = ms.rotations * ms.counts_per_rev + pos_measurement;
    ms.last_pos_measurement = pos_measurement;

    // Velocity
    ms.velocity = velocity_measurement;

    // Torque
    ms.torque = torque_measurement;
}

void CS610ReadCallback(const CAN_message_t &msg, MotorState (&motor_states)[NUM_C610S]) {
    if (msg.id >= RECEIVE_BASE_ID + 1 && msg.id <= RECEIVE_BASE_ID + 8)
    {
        uint8_t esc_index = msg.id - RECEIVE_BASE_ID - 1; // ESC 1 corresponds to index 0
        if (esc_index >= 0 && esc_index <= 7)
        {
            int32_t pos, velocity, torque;
            interpretC610Message(msg, pos, velocity, torque);
            updateMotorState(motor_states[esc_index], pos, velocity, torque);
        }
        else
        {
            Serial.println("Invalid esc index in CAN message");
            return;
        }
    }
}
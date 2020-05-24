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
    C610(int32_t counts_per_rev = 8192);
    static void torqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower);
    void updateState(const int32_t pos_measurement, const int32_t velocity_measurement, const int32_t torque_measurement);
    static void interpretMessage(const CAN_message_t &msg, int32_t &pos, int32_t &vel, int32_t &torque);
};

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

class C610Array
{
private:
    static const uint8_t NUM_C610S = 8;
    static const uint32_t ZERO_TO_THREE_COMMAND_ID = 0x200;
    static const uint32_t FOUR_TO_SEVEN_COMMAND_ID = 0x1FF;
    static const uint32_t RECEIVE_BASE_ID = 0x200;

    C610 _controllers[NUM_C610S];
    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> _can0; // TODO big potential issue: revert this to CAN1 later
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> _can1;

public:
    C610Array();
    void pollCAN();
    void callback(CAN_message_t &msg);
    void commandTorques(int32_t torque0, int32_t torque1 = 0, int32_t torque2 = 0, int32_t torque3 = 0, uint8_t subbus = 0);
    void initializeCANBus();
};

void C610Array::pollCAN() {
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
    // _can0.onReceive(callback);
    _can0.mailboxStatus();
    
    _can1.begin();
    _can1.setBaudRate(1000000);
    _can1.setMaxMB(16);
    _can1.enableFIFO();
    _can1.enableFIFOInterrupt();
    // _can0.onReceive(callback);
    _can1.mailboxStatus();
}

C610Array::C610Array()
{
    
}

void C610Array::callback(CAN_message_t &msg)
{
    if (msg.id >= RECEIVE_BASE_ID + 1 && msg.id <= RECEIVE_BASE_ID + NUM_C610S)
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

void C610Array::commandTorques(int32_t torque0, int32_t torque1 = 0, int32_t torque2 = 0, int32_t torque3 = 0, uint8_t subbus = 0)
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
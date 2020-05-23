#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610.h"
#include "PID.h"

const int PRINT_DELAY = 20 * 1000;
const int CONTROL_DELAY = 1000;
const int FEEDBACK_DELAY = 1000;

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

MotorState MOTOR_STATES[NUM_C610S];

PDGAINS EXP_GAINS;

int32_t torque_setting = 0;
int32_t torque_commands[NUM_C610S] = {0, 0, 0, 0, 0, 0, 0, 0};

enum Mode
{
    TORQUE,
    PID,
    IDLE
};
Mode control_mode = Mode::IDLE;

long last_command_ts;
long last_print_ts;

// TODO: Make the callback an object function so it can access scoped data, rather than
// this way (global function) which can only access global data
void readCAN(const CAN_message_t &msg)
{
    CS610ReadCallback(msg, MOTOR_STATES);
}

void zeroTorqueCommands(int32_t (&torque_commands)[NUM_C610S])
{
    for (int i = 0; i < NUM_C610S; i++)
    {
        torque_commands[i] = 0;
    }
}

void setup(void)
{
    Serial.begin(115200);
    delay(400);
    pinMode(6, OUTPUT);
    digitalWrite(6, LOW); /* optional tranceiver enable pin */
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.onReceive(readCAN);
    Can0.mailboxStatus();

    initializeMotorStates(MOTOR_STATES, NUM_C610S);

    EXP_GAINS.kp = 0.16;
    EXP_GAINS.kd = 0.6;

    last_command_ts = micros();
    last_print_ts = micros();
}

void loop()
{
    Can0.events();
    if (micros() - last_command_ts > CONTROL_DELAY)
    {
        switch (control_mode)
        {
        case Mode::IDLE:
        {
            zeroTorqueCommands(torque_commands);
            break;
        }
        case Mode::PID:
        {
            float target_pos = 0;
            zeroTorqueCommands(torque_commands);
            for (int i = 0; i < NUM_C610S; i++)
            {
                pid(torque_commands[i], MOTOR_STATES[i].counts, MOTOR_STATES[i].velocity, target_pos, 0, EXP_GAINS);
                torque_commands[i] = constrain(torque_commands[i], -5000, 5000);
                break;
            }
        }
        case Mode::TORQUE:
        {
            zeroTorqueCommands(torque_commands);
            torque_commands[0] = torque_setting;
            break;
        }
        }
        // SINUSOIDAL
        // const float freq = 40;
        // float phase = freq * micros() * 2 * PI / 1000000;
        // torque0_command = int32_t(torque_setting * (0.5 + sin(phase) / 2.0));

        // send to esc 0 (id = 1)
        // sendTorqueCommand(Can0, torque_commands[0], torque_commands[1], torque_commands[2], torque_commands[3], 0);
        last_command_ts = micros();
    }

    if (micros() - last_print_ts > PRINT_DELAY)
    {
        Serial.print(millis());
        Serial.print("\t");
        for (uint8_t i = 0; i < NUM_C610S; i++)
        {
            Serial.print(MOTOR_STATES[i].counts);
            Serial.print("\t");
            Serial.print(MOTOR_STATES[i].velocity);
            Serial.print("\t");
            Serial.print(MOTOR_STATES[i].torque);
            Serial.print("\t");
            Serial.print(torque_commands[i]);
            Serial.print("\t");
        }
        Serial.print(torque_setting);
        // Serial.print("\t");
        // Serial.print(6000);
        // Serial.print("\t");
        // Serial.print(-6000);
        Serial.println();
        last_print_ts = micros();
    }

    while (Serial.available())
    {
        char c = Serial.read();
        Serial.print("Received: ");
        Serial.print(c);
        Serial.println(" * 1000mA");

        uint8_t int_input = c - '0';
        if (int_input >= 0 && int_input <= 20)
        {
            torque_setting = -int_input * 1000;
        }
        if (c == '`')
        {
            torque_setting = 0;
        }
        while (Serial.available())
        {
            Serial.read();
        }
    }
}

// has a 3 second watchdog
// 200 is the min current required to turn the output shaft (0.2A)
// stalls at 5A when holding very still but if small turns made doesn't hit stall condition
// 5 min at 3000 is totally ok, maybe 40deg c
// 8 min at 4000 is warm, maybe 60-70c at the middle aluminum
// takes around 0.03 seconds to go from 0 to 7150, with command 8000

// Notes: PID worked at 1000 hz, also worked at 100hz!
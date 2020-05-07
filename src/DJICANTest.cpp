#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610.h"
#include "PID.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

long last_print_ts = millis();
MotorState MOTOR_STATE_1;
MotorState MOTOR_STATES[NUM_C610S];

PDGAINS EXP_GAINS;

int32_t current_torque_command = 0;
long last_command_ts = millis();

void readCAN(const CAN_message_t &msg)
{
    if (msg.id >= ID_ONE_TO_EIGHT + 1 && msg.id <= ID_ONE_TO_EIGHT + 8)
    {
        int32_t esc_index = msg.id - ID_ONE_TO_EIGHT - 1; // ESC 1 corresponds to index 0
        int32_t pos, velocity, torque;
        interpretC610Message(msg, pos, velocity, torque);
        updateMotorState(MOTOR_STATES[esc_index], pos, velocity, torque);

        if (millis() - last_print_ts > 10)
        {
            Serial.print(MOTOR_STATES[0].counts / 10);
            Serial.print("\t");
            Serial.print(MOTOR_STATES[0].velocity);
            Serial.print("\t");
            Serial.print(MOTOR_STATES[0].torque);
            Serial.print("\t");
            Serial.print(current_torque_command);
            Serial.print("\t");
            Serial.print(6000);
            Serial.print("\t");
            Serial.print(-6000);
            Serial.println();
            last_print_ts = millis();
        }
    }
}

void plotCAN(const CAN_message_t &msg)
{
    int32_t pos, velocity, torque;
    interpretC610Message(msg, pos, velocity, torque);
    updateMotorState(MOTOR_STATE_1, pos, velocity, torque);

    if (millis() - last_print_ts > 10)
    {
        Serial.print(MOTOR_STATE_1.counts / 10);
        Serial.print("\t");
        Serial.print(MOTOR_STATE_1.velocity);
        Serial.print("\t");
        Serial.print(MOTOR_STATE_1.torque);
        Serial.print("\t");
        Serial.print(6000);
        Serial.print("\t");
        Serial.print(-6000);
        Serial.println();
        last_print_ts = millis();
    }
}

void canSniff(const CAN_message_t &msg)
{
    Serial.print("MB ");
    Serial.print(msg.mb);
    Serial.print("  OVERRUN: ");
    Serial.print(msg.flags.overrun);
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print(" TS: ");
    Serial.print(msg.timestamp);
    Serial.print(" ID: ");
    Serial.print(msg.id, HEX);
    Serial.print(" Buffer: ");
    for (uint8_t i = 0; i < msg.len; i++)
    {
        Serial.print(msg.buf[i], HEX);
        Serial.print(" ");
    }

    uint16_t mech_angle = 0;
    mech_angle = (msg.buf[0] << 8) | msg.buf[1];
    Serial.print(" Angle: ");
    Serial.print(mech_angle);
    int16_t rotor_speed = (msg.buf[2] << 8) | msg.buf[3];
    Serial.print(" Speed: ");
    Serial.print(rotor_speed);
    int16_t torque = (msg.buf[4] << 8) | msg.buf[5];
    Serial.print(" Torque: ");
    Serial.print(torque);
    Serial.println();
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
    //  Can0.onReceive(canSniff);
    //  Can0.onReceive(plotCAN);
    Can0.onReceive(readCAN);
    Can0.mailboxStatus();

    initializeMotorState(MOTOR_STATE_1);
    initializeMotorStates(MOTOR_STATES, NUM_C610S);

    EXP_GAINS.kp = 0.16;
    EXP_GAINS.kd = 0.6;
}

// has a 3 second watchdog
// 200 is the min current required to turn the output shaft (0.2A)
// stalls at 5A when holding very still but if small turns made doesn't hit stall condition
// 5 min at 3000 is totally ok, maybe 40deg c
// 8 min at 4000 is warm, maybe 60-70c at the middle aluminum
void loop()
{
    Can0.events();
    // if (micros() - last_command_ts > 1000)
    // { // 1000 hz, also worked at 100hz!
    //     float target_pos = 100.0 * sin(millis() / 1000.0);
    //     pid(current_torque_command, MOTOR_STATES[0].counts, MOTOR_STATES[0].velocity, target_pos, 0, EXP_GAINS);
    //     current_torque_command = constrain(current_torque_command, -4000, 4000);
    //     sendTorqueCommand(Can0, current_torque_command, 1);
    //     last_command_ts = micros();
    // }

    if (millis() - last_command_ts > 100)
    {
        sendTorqueCommand(Can0, current_torque_command, 1);
        last_command_ts = millis();
    }

    while (Serial.available())
    {
        char c = Serial.read();
        Serial.print("Received: ");
        Serial.print(c);
        Serial.println(" * 1000mA");
        for (int8_t i = 0; i < 10; i++)
        {
            if (c == i + '0')
            {
                current_torque_command = - i * 1000;
            }
        }
        if (c == '`')
            {
                current_torque_command = 0;
            }
        while (Serial.available())
        {
            Serial.read();
        }
    }
}
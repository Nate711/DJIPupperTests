#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "PID.h"
#include "DriveSystem.h"

enum ControlMode
{
    CONST_TORQUE,
    RIPPLE_TORQUE,
    PID,
    IDLE,
    PID_ADJUSTED
};

enum PIDMode
{
    SIN,
    CONST
};

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 20 * 1000;
const int CONTROL_DELAY = 1000;

const int32_t MAX_TORQUE = 6000;
PDGAINS EXP_GAINS = {0.15, 1.5};
const uint8_t CONST_TORQUE_ESC = 2;
const uint8_t CONTROL_MASK[C610Bus<>::SIZE] = {0, 0, 1, 0, 0, 0, 0, 0};
ControlMode control_mode = ControlMode::PID_ADJUSTED;
PIDMode position_mode = PIDMode::CONST;
////////////////////// END CONFIG ///////////////////////

C610Bus<CAN2> controller_bus;

int32_t torque_setting = 0;
int32_t torque_commands[C610Bus<>::SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

long last_command_ts;
long last_print_ts;

/*
TODOs
object-based control of motors
one high-level object to control all escs (12 in total)
- needs to keep two separate can objects
- needs to assign esc objects to certain can objecst
- set torques
- callback to update objects
- wrapper for retrieving all positions and velocities
- send torques (keeps track of which subbus)
object per esc
- update position / velocity, keep track of wrap-around
*/

void zeroTorqueCommands(int32_t torque_commands[], const uint8_t vec_length)
{
    for (int i = 0; i < vec_length; i++)
    {
        torque_commands[i] = 0;
    }
}

void maskTorques(int32_t torque_commands[], const uint8_t mask[], const uint8_t vec_length)
{
    for (int i = 0; i < vec_length; i++)
    {
        torque_commands[i] = torque_commands[i] * mask[i];
    }
}

void parseSerialInput(char c, int32_t &torque_setting)
{
    // Right now this code treats all inputs as constant torque requests
    // TODO: make the serial parsing more flexible

    // Enter 'x' to go to IDLE mode!
    if (c == 'x')
    {
        control_mode = IDLE;
    }

    Serial.print("Received: ");
    Serial.print(c);
    Serial.println(" * 1000mA");

    uint8_t int_input = c - '0';
    if (int_input >= 0 && int_input <= 20)
    {
        torque_setting = int_input * 1000;
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

template <CAN_DEV_TABLE _bus>
void printMotorStates(C610Bus<_bus> bus, int32_t torque_commands[], uint8_t torque_length)
{
    Serial.print(millis() % 5000);
    Serial.print("\t");
    for (uint8_t i = 0; i < C610Bus<>::SIZE; i++)
    {
        Serial.print(bus.get(i).counts() / 100);
        Serial.print("\t");
        Serial.print(bus.get(i).rpm());
        Serial.print("\t");
        Serial.print(bus.get(i).torque());
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

void setup(void)
{
    Serial.begin(115200);
    delay(400);

    last_command_ts = micros();
    last_print_ts = micros();

    // I'm so sorry you have to read this. This code just sets up the callback for the CAN bus.
    controller_bus.can().onReceive([](const CAN_message_t &msg) { controller_bus.callback(msg); });
}

void loop()
{
    controller_bus.pollCAN();
    if (micros() - last_command_ts > CONTROL_DELAY)
    {
        switch (control_mode)
        {
        case ControlMode::IDLE:
        {
            zeroTorqueCommands(torque_commands, C610Bus<>::SIZE);
            break;
        }
        case ControlMode::PID:
        {
            float target_pos = 0;
            zeroTorqueCommands(torque_commands, C610Bus<>::SIZE);
            for (int i = 0; i < C610Bus<>::SIZE; i++)
            {
                pid(torque_commands[i], controller_bus.get(i).counts(), controller_bus.get(i).rpm(), target_pos, 0, EXP_GAINS);
                torque_commands[i] = constrain(torque_commands[i], -MAX_TORQUE / 1.3, MAX_TORQUE / 1.3);
            }
            maskTorques(torque_commands, CONTROL_MASK, C610Bus<>::SIZE);
            break;
        }
        case ControlMode::PID_ADJUSTED:
        {
            float target_pos = 0;
            zeroTorqueCommands(torque_commands, C610Bus<>::SIZE);
            for (int i = 0; i < C610Bus<>::SIZE; i++)
            {
                // torque_commands[i] = 1000;
                pid(torque_commands[i], controller_bus.get(i).counts(), controller_bus.get(i).rpm(), target_pos, 0, EXP_GAINS);
                torque_commands[i] = constrain(torque_commands[i], -MAX_TORQUE / 1.3, MAX_TORQUE / 1.3);
                // if (abs(controller_bus.get(i).rpm()) > 0)
                // {
                if (torque_commands[i] * controller_bus.get(i).rpm() > 0)
                {
                    // Positive work
                    torque_commands[i] *= 1.3;
                }
                else if (torque_commands[i] * controller_bus.get(i).rpm() < 0)
                {
                    torque_commands[i] /= 1.3;
                    // Negative work
                }
                else
                {
                    // torque_commands[i] = 0;
                    torque_commands[i] /= 2;
                }

                torque_commands[i] = constrain(torque_commands[i], -MAX_TORQUE, MAX_TORQUE);
            }
            maskTorques(torque_commands, CONTROL_MASK, C610Bus<>::SIZE);
            break;
        }
        case ControlMode::CONST_TORQUE:
        {
            zeroTorqueCommands(torque_commands, C610Bus<>::SIZE);
            torque_commands[CONST_TORQUE_ESC] = torque_setting;
            break;
        }
        case ControlMode::RIPPLE_TORQUE:
        {
            zeroTorqueCommands(torque_commands, C610Bus<>::SIZE);
            float phase = 90.0 * millis() / 1000.0;                                // 30hz // 20hz
            torque_commands[CONST_TORQUE_ESC] = torque_setting + 600 * sin(phase); // 30hz ripple
            break;
        }
        }
        // SINUSOIDAL
        // const float freq = 40;
        // float phase = freq * micros() * 2 * PI / 1000000;
        // torque0_command = int32_t(torque_setting * (0.5 + sin(phase) / 2.0));

        controller_bus.commandTorques(torque_commands[0], torque_commands[1], torque_commands[2], torque_commands[3], 0);
        last_command_ts = micros();
    }

    if (micros() - last_print_ts > PRINT_DELAY)
    {
        printMotorStates(controller_bus, torque_commands, C610Bus<>::SIZE);
    }

    while (Serial.available())
    {
        char c = Serial.read();
        parseSerialInput(c, torque_setting);
    }
}

// has a 3 second watchdog
// 200 is the min current required to turn the output shaft (0.2A)
// stalls at 5A when holding very still but if small turns made doesn't hit stall condition
// 5 min at 3000 is totally ok, maybe 40deg c
// 8 min at 4000 is warm, maybe 60-70c at the middle aluminum
// takes around 0.03 seconds to go from 0 to 7150, with command 8000

// Notes: PID worked at 1000 hz, also worked at 100hz!
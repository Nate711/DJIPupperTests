#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "DriveSystem.h"

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 20 * 1000;
const int CONTROL_DELAY = 1000;

const int32_t MAX_TORQUE = 6000;
PDGains PID_GAINS = {0.15, 1.5};
const uint8_t CONST_TORQUE_ESC = 2;
const uint8_t CONTROL_MASK[DriveSystem::kNumActuators] = {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
////////////////////// END CONFIG ///////////////////////


DriveSystem drive;

int32_t torque_setting = 0;
int32_t torque_commands[DriveSystem::kNumActuators] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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


void parseSerialInput(char c, int32_t &torque_setting)
{
    // Right now this code treats all inputs as constant torque requests
    // TODO: make the serial parsing more flexible

    // Enter 'x' to go to IDLE mode!
    if (c == 'x')
    {
        drive.SetIdle();
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

void setup(void)
{
    Serial.begin(115200);
    delay(400);

    last_command_ts = micros();
    last_print_ts = micros();

    // Set up the CAN bus message callbacks. 
    drive.RearBus().can().onReceive([](const CAN_message_t &msg) { drive.RearBus().callback(msg); });
    drive.FrontBus().can().onReceive([](const CAN_message_t &msg) { drive.FrontBus().callback(msg); });
}

void loop()
{
    drive.CheckForCANMessages();
    if (micros() - last_command_ts > CONTROL_DELAY)
    {
        drive.Update();
        last_command_ts = micros();
    }

    if (micros() - last_print_ts > PRINT_DELAY)
    {
        DrivePrintOptions options;
        drive.PrintStatus(options);
        last_print_ts = micros();
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
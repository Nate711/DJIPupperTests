#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "DriveSystem.h"

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 2 * 1000;
const int CONTROL_DELAY = 1000;
const float MAX_TORQUE = 6.0;
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;

long last_command_ts;
long last_print_ts;

void RespondToSerialInput(char c, DriveSystem &d)
{
    // Right now this code treats all inputs as position requests
    // TODO: make the serial parsing more flexible

    // Enter 'x' to go to IDLE mode!
    if (c == 'x')
    {
        drive.SetIdle();
    }

    Serial.print("Received: ");
    Serial.println(c);

    uint8_t int_input = c - '0';
    if (int_input >= 0 && int_input <= 20)
    {
        for (int i = 0; i < DriveSystem::kNumActuators; i++)
        {
            d.SetPosition(i, int_input / 4.0);
        }
    }
    if (c == '`')
    {
        for (int i = 0; i < DriveSystem::kNumActuators; i++)
        {
            d.SetPosition(i, 0.0);
        }
    }
    if (c == 's')
    {
        d.SetPosition(0, 1.5);
        d.SetPosition(2, -3);
    }
    if (c == 'j')
    {
        d.SetPosition(0, 0.0);
        d.SetPosition(2, 0.0);
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

    ////////////// Runtime config /////////////////////
    drive.SetMaxCurrent(MAX_TORQUE);

    // Put it in PID mode
    drive.SetUniformPositionGains(4.5, 0.0003);
    for (uint8_t i = 0; i < DriveSystem::kNumActuators; i++)
    {
        drive.SetPosition(i, 0.0);
    }
    drive.ActivateActuator(0);
    drive.ActivateActuator(2);

    // drive.SetIdle(); // alternative mode
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
        RespondToSerialInput(c, drive);
    }
}

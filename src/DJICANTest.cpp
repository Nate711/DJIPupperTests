#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "DriveSystem.h"
#include <NonBlockingSerialBuffer.h>
#include <ArduinoJson.h>
#include <Streaming.h>
#include <CommandInterpreter.h>

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 2 * 1000;
const int CONTROL_DELAY = 1000;
const float MAX_TORQUE = 6.0;
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;
CommandInterpreter interpreter;

DrivePrintOptions options;

long last_command_ts;
long last_print_ts;

void setup(void)
{
    Serial.begin(115200);
    delay(400);

    last_command_ts = micros();
    last_print_ts = micros();

    ////////////// Runtime config /////////////////////
    drive.SetMaxCurrent(MAX_TORQUE);
    options.print_delay_millis = PRINT_DELAY;

    // Put it in PID mode
    drive.SetUniformPositionGains(4.5, 0.0003);
    for (uint8_t i = 0; i < DriveSystem::kNumActuators; i++)
    {
        drive.SetPosition(i, 0.0);
    }
    drive.ActivateActuator(6); // ID 1 on CAN2
    drive.ActivateActuator(8); // ID 3 on CAN2
    // drive.SetIdle(); // alternatively, set the drive to idle
}

void loop()
{
    drive.CheckForCANMessages();
    CommandResult r = interpreter.Feed();
    if (r == CommandResult::kNewCommand)
    {
        Serial << "Got new command." << endl;
        Serial << interpreter.LatestPositionCommand() << endl;
    }

    if (micros() - last_command_ts > CONTROL_DELAY)
    {
        drive.Update();
        last_command_ts = micros();
    }

    if (micros() - last_print_ts > options.print_delay_millis)
    {
        drive.PrintStatus(options);
        last_print_ts = micros();
    }
}

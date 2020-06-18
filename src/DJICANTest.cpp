#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "DriveSystem.h"
#include <NonBlockingSerialBuffer.h>
#include <ArduinoJson.h>
#include <Streaming.h>
#include <CommandInterpreter.h>

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 20 * 1000;
const int CONTROL_DELAY = 1000;
const float MAX_TORQUE = 6.0;
PDGains DEFAULT_GAINS = {6, 0.002};
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;

// Example json message with default start and stop characters: <{"kp":2.0}>
CommandInterpreter interpreter(false); // use_msgpack: false, use default arguments for the rest

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
    drive.SetAllPositionGains(DEFAULT_GAINS);
    for (uint8_t i = 0; i < DriveSystem::kNumActuators; i++)
    {
        drive.SetPosition(i, 0.0);
    }
    drive.ActivateActuator(6); // ID 1 on CAN2
    drive.ActivateActuator(7); // ID 2 on CAN2
    drive.ActivateActuator(8); // ID 3 on CAN2
    // drive.SetIdle(); // alternatively, set the drive to idle
}

void loop()
{
    drive.CheckForCANMessages();
    CheckResult r = interpreter.CheckForMessages();
    if (r.flag == CheckResultFlag::kNewCommand)
    {
        Serial << "Got new command." << endl;
        if (r.new_position)
        {
            drive.SetAllPositions(interpreter.LatestPositionCommand());
            Serial << interpreter.LatestPositionCommand() << endl;
        }
        if (r.new_kp)
        {
            drive.SetAllPositionKp(interpreter.LatestKp());
            Serial << "Kp: " << interpreter.LatestKp() << endl;
        }
        if (r.new_kd)
        {
            drive.SetAllPositionKd(interpreter.LatestKd());
            Serial << "Kd: " << interpreter.LatestKd() << endl;
        }
    }

    if (micros() - last_command_ts >= CONTROL_DELAY)
    {
        // TODO: characterize maximum lag between control updates
        drive.Update();
        last_command_ts = micros();
    }

    if (micros() - last_print_ts >= options.print_delay_millis)
    {
        drive.PrintStatus(options);
        last_print_ts = micros();
    }
}

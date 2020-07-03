#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "DriveSystem.h"
#include <NonBlockingSerialBuffer.h>
#include <ArduinoJson.h>
#include <Streaming.h>
#include <CommandInterpreter.h>

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 50; // millis
const int HEADER_DELAY = 5000; // millis
const int CONTROL_DELAY = 1000; // micros
const float MAX_TORQUE = 2.0;
PDGains DEFAULT_GAINS = {8, 0.004};
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;

// Example json message with default start and stop characters: <{"kp":2.0}>
CommandInterpreter interpreter(true); // use_msgpack: true, use default arguments for the rest

DrivePrintOptions options;

long last_command_ts;
long last_print_ts;
long last_header_ts;

void setup(void)
{
    Serial.begin(115200);
    delay(400);

    last_command_ts = micros();
    last_print_ts = millis();
    last_header_ts = millis();

    ////////////// Runtime config /////////////////////
    drive.SetMaxCurrent(MAX_TORQUE);
    options.print_delay_millis = PRINT_DELAY;
    options.header_delay_millis = HEADER_DELAY;

    // Set behavioral options
    drive.SetAllPositionGains(DEFAULT_GAINS);
    // No motors are activated on startup, and the drive system will start up in idle mode
    
    drive.PrintHeader(options);

    interpreter.Flush();
}

void loop()
{
    drive.CheckForCANMessages();
    CheckResult r = interpreter.CheckForMessages();
    if (r.flag == CheckResultFlag::kNewCommand)
    {
        // Serial << "Got new command." << endl;
        if (r.new_position)
        {
            drive.SetAllPositions(interpreter.LatestPositionCommand());
            Serial << "Position command: " << interpreter.LatestPositionCommand() << endl;
        }
        if (r.new_kp)
        {
            drive.SetAllPositionKp(interpreter.LatestKp());
            Serial << "Kp: " << interpreter.LatestKp() << endl;
        }
        if (r.new_kd)
        {
            drive.SetAllPositionKd(interpreter.LatestKd());
            Serial.print("Kd: ");
            Serial.println(interpreter.LatestKd(), 4);
        }
        if (r.new_max_current)
        {
            drive.SetMaxCurrent(interpreter.LatestMaxCurrent());
            Serial << "Max Current: " << interpreter.LatestMaxCurrent() << endl;
        }
        if (r.new_activation)
        {
            drive.SetActivations(interpreter.LatestActivations());
            Serial << "Activations: " << interpreter.LatestActivations() << endl;
        }
        if (r.do_zero)
        {
            drive.ZeroCurrentPosition();
            Serial << "Setting current position as the zero point" << endl;
        }
    }

    if (micros() - last_command_ts >= CONTROL_DELAY)
    {
        // TODO: characterize maximum lag between control updates
        drive.Update();
        last_command_ts = micros();
    }

    if (millis() - last_print_ts >= options.print_delay_millis)
    {
        drive.PrintStatus(options);
        last_print_ts = millis();
    }

    if (millis() - last_header_ts >= options.header_delay_millis)
    {
        drive.PrintHeader(options);
        last_header_ts = millis();
    }
}

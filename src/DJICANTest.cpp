#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "DriveSystem.h"
#include <NonBlockingSerialBuffer.h>
#include <ArduinoJson.h>

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 2 * 1000;
const int CONTROL_DELAY = 1000;
const float MAX_TORQUE = 6.0;
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;

// start char is <, stop char is '\n', use Serial as input stream, true for adding \0 string termination
NonBlockingSerialBuffer<256> reader('<', '\n', Serial, true); 

StaticJsonDocument<256> doc;

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

    // Put it in PID mode
    drive.SetUniformPositionGains(4.5, 0.0003);
    for (uint8_t i = 0; i < DriveSystem::kNumActuators; i++)
    {
        drive.SetPosition(i, 0.0);
    }
    drive.ActivateActuator(6); // ID 1 on CAN2
    drive.ActivateActuator(8); // ID 3 on CAN2
    // drive.SetIdle(); // alternative mode
}

void loop()
{
    drive.CheckForCANMessages();
    ParseResult r = reader.Feed();
    if (r.result == ParseResultFlag::kDone)
    {
        Serial.print("GOT: ");
        Serial.println(reader.buffer_);

        doc.clear();
        auto err = deserializeJson(doc, reader.buffer_);
        if (err)
        {
            Serial.println(err.c_str());
        }
        else
        {
            auto obj = doc.as<JsonObject>();
            for (JsonPair p : obj)
            {                
                if (p.value().is<char*>()) {
                    Serial.println(p.value().as<char*>());
                }
            }
        }
    }

    if (micros() - last_command_ts > CONTROL_DELAY)
    {
        drive.Update();
        last_command_ts = micros();
    }

    if (micros() - last_print_ts > PRINT_DELAY)
    {
        // DrivePrintOptions options;
        // drive.PrintStatus(options);
        last_print_ts = micros();
    }
}

#include <NonBlockingSerialBuffer.h>
#include "Arduino.h"
#include <Streaming.h>
#include <array>
#include "PID.h"
#include "RobotTypes.h"

enum class CheckResultFlag
{
    kNothing,
    kNewCommand,
    kError
};
struct CheckResult
{
    bool new_position = false;
    bool new_kp = false;
    bool new_kd = false;
    CheckResultFlag flag = CheckResultFlag::kNothing;
};

class CommandInterpreter
{
private:
    ActuatorPositionVector position_command_;
    PDGains gain_command_;

    StaticJsonDocument<512> doc_;
    NonBlockingSerialBuffer<512> reader_;

    const bool use_msgpack_;

public:
    // Default to using < as the message start indicator, > as the message stop indicator, and Serial as the input stream
    CommandInterpreter(bool use_msgpack = true, uint8_t start_byte = '<', uint8_t stop_byte = '>', Stream &stream = Serial);

    // Checks the serial input buffer for bytes. Returns an enum indicating the result of the read: kNothing, kNewCommand, or kError. Should be called as fast as possible.
    CheckResult CheckForMessages();

    // Returns an ActuatorPositionVector with the latest position commands.
    ActuatorPositionVector LatestPositionCommand();

    // Return a PDGains object with the latest gain parameters.
    // PDGains LatestGainCommand();

    float LatestKp();
    float LatestKd();
};

// Start character: '<'
// Stop character: '\n'
// Stream: Serial
// Add '\0' to treat input as a string? : true
CommandInterpreter::CommandInterpreter(bool use_msgpack, uint8_t start_byte, uint8_t stop_byte, Stream &stream) : reader_(start_byte, stop_byte, stream, true), use_msgpack_(use_msgpack) {}

CheckResult CommandInterpreter::CheckForMessages()
{
    CheckResult result;
    BufferResult buffer_result = reader_.Read();
    if (buffer_result == BufferResult::kDone)
    {
        doc_.clear();
        auto err = use_msgpack_ ? deserializeMsgPack(doc_, reader_.buffer_) : deserializeJson(doc_, reader_.buffer_);
        if (err)
        {
            Serial << "deserialize failed: " << err.c_str() << endl;
            result.flag = CheckResultFlag::kError;
            return result;
        }
        else
        {
            auto obj = doc_.as<JsonObject>();
            if (obj.containsKey("pos"))
            {
                auto data = obj["pos"].as<JsonArray>();
                if (data.size() != position_command_.size())
                {
                    Serial << "Error: Invalid number of parameters in position command." << endl;
                    result.flag = CheckResultFlag::kError;
                    return result;
                }

                uint8_t i = 0;
                for (auto v : data)
                {
                    position_command_[i++] = v.as<float>();
                }
                result.new_position = true;
                result.flag = CheckResultFlag::kNewCommand;
            }
            if (obj.containsKey("kp"))
            {
                gain_command_.kp = obj["kp"].as<float>();
                result.new_kp = true;
                result.flag = CheckResultFlag::kNewCommand;
            }
            if (obj.containsKey("kd"))
            {
                gain_command_.kd = obj["kd"].as<float>();
                result.new_kd = true;
                result.flag = CheckResultFlag::kNewCommand;
            }
        }
    }
    return result;
}

ActuatorPositionVector CommandInterpreter::LatestPositionCommand()
{
    return position_command_;
}

float CommandInterpreter::LatestKp()
{
    return gain_command_.kp;
}
float CommandInterpreter::LatestKd()
{
    return gain_command_.kd;
}

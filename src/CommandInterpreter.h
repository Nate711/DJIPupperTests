#include <NonBlockingSerialBuffer.h>
#include "Arduino.h"
#include <Streaming.h>
#include <array>

enum class CommandResult
{
    kNothing,
    kNewCommand,
    kError
};

typedef std::array<float, 12> ActuatorPositionVector;
typedef std::array<float, 2> GainParameters;

Print& operator<<(Print& stream, const ActuatorPositionVector& vec) {
    for (auto e : vec) {
        stream << e << " ";
    }
    stream << endl;
    return stream;
}

class CommandInterpreter
{
private:
    ActuatorPositionVector position_command_;
    GainParameters gain_command_;

    StaticJsonDocument<256> doc_;
    NonBlockingSerialBuffer<256> reader_;

public:
    CommandInterpreter();
    CommandResult Feed();
    ActuatorPositionVector LatestPositionCommand();
    GainParameters LatestGainCommand();
};

// Static buffer size: 256 bytes
// Start character: '<'
// Stop character: '\n'
// Stream: Serial
// Add '\0' to treat input as a string? : true
CommandInterpreter::CommandInterpreter() : reader_('<', '\n', Serial, true) {}

CommandResult CommandInterpreter::Feed()
{
    CommandResult result = CommandResult::kNothing;
    ParseResult r = reader_.Feed();
    if (r.result == ParseResultFlag::kDone)
    {
        doc_.clear();
        auto err = deserializeMsgPack(doc_, reader_.buffer_);
        if (err)
        {
            Serial << "deserializeMsgPack() failed: " << err.c_str() << endl;
            return CommandResult::kError;
        }
        else
        {
            auto obj = doc_.as<JsonObject>();
            if (obj.containsKey("p"))
            {
                auto data = obj["p"].as<JsonArray>();
                if (data.size() != position_command_.size())
                {
                    Serial << "Error: Invalid number of parameters in position command." << endl;
                    return CommandResult::kError;
                }

                uint8_t i = 0;
                for (auto v : data)
                {
                    position_command_[i++] = v.as<float>();
                }
                result = CommandResult::kNewCommand;
            }
            if (obj.containsKey("g"))
            {
                JsonArray data = obj["g"].as<JsonArray>();
                if (data.size() != gain_command_.size())
                {
                    Serial << "Error: Invalid number of parameters in gain command." << endl;
                    return CommandResult::kError;
                }

                uint8_t i = 0;
                for (auto v : data)
                {
                    gain_command_[i++] = v.as<float>();
                }
                result = CommandResult::kNewCommand;
            }
        }
    }
    return result;
}

ActuatorPositionVector CommandInterpreter::LatestPositionCommand()
{
    return position_command_;
}

GainParameters CommandInterpreter::LatestGainCommand()
{
    return gain_command_;
}
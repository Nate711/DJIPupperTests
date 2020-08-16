#pragma once
#include <NonBlockingSerialBuffer.h>
#include <Streaming.h>

#include <array>

#include "Arduino.h"
#include "PID.h"
#include "RobotTypes.h"

enum class CheckResultFlag { kNothing, kNewCommand, kError };
struct CheckResult {
  bool new_position = false;
  bool new_kp = false;
  bool new_kd = false;
  bool new_max_current = false;
  bool new_activation = false;
  bool do_zero = false;
  bool do_idle = false;
  CheckResultFlag flag = CheckResultFlag::kNothing;
};

class CommandInterpreter {
 private:
  ActuatorPositionVector position_command_;
  ActuatorActivations activations_;

  PDGains gain_command_;
  float max_current_;

  StaticJsonDocument<512> doc_;
  NonBlockingSerialBuffer<512> reader_;

  const bool use_msgpack_;

 public:
  // Default to using msgpack, 0x00 as the message start indicator, and Serial
  // as the input stream
  CommandInterpreter(bool use_msgpack = true, uint8_t start_byte = 0x00,
                     Stream &stream = Serial);

  // Checks the serial input buffer for bytes. Returns an enum indicating the
  // result of the read: kNothing, kNewCommand, or kError. Should be called as
  // fast as possible.
  CheckResult CheckForMessages();

  // Returns an ActuatorPositionVector with the latest position commands.
  ActuatorPositionVector LatestPositionCommand();

  ActuatorActivations LatestActivations();

  // Empty the input buffer
  void Flush();

  float LatestKp();
  float LatestKd();
  float LatestMaxCurrent();
};

// Start character: 0x00
// Stream: Serial
// Add '\0' to treat input as a string? : true
CommandInterpreter::CommandInterpreter(bool use_msgpack, uint8_t start_byte,
                                       Stream &stream)
    : reader_(start_byte, stream, true), use_msgpack_(use_msgpack) {}

template <class T, unsigned int SIZE>
CheckResultFlag CopyJsonArray(JsonArray json, std::array<T, SIZE> &arr) {
  if (json.size() != arr.size()) {
    Serial << "Error: Invalid number of parameters in position command."
           << endl;
    return CheckResultFlag::kError;
  }
  uint8_t i = 0;
  for (auto v : json) {
    arr[i++] = v.as<T>();
  }
  return CheckResultFlag::kNewCommand;
}

CheckResult CommandInterpreter::CheckForMessages() {
  CheckResult result;
  BufferResult buffer_result = reader_.Read();
  if (buffer_result == BufferResult::kDone) {
    doc_.clear();
    auto err = use_msgpack_ ? deserializeMsgPack(doc_, reader_.buffer_)
                            : deserializeJson(doc_, reader_.buffer_);
    if (err) {
      Serial << "deserialize failed: " << err.c_str() << endl;
      result.flag = CheckResultFlag::kError;
      return result;
    } else {
      auto obj = doc_.as<JsonObject>();
      if (obj.containsKey("pos")) {
        auto json_array = obj["pos"].as<JsonArray>();
        result.flag = CopyJsonArray(json_array, position_command_);
        if (result.flag == CheckResultFlag::kNewCommand) {
          result.new_position = true;
        }
        return result;
      }
      if (obj.containsKey("kp")) {
        gain_command_.kp = obj["kp"].as<float>();
        result.new_kp = true;
        result.flag = CheckResultFlag::kNewCommand;
      }
      if (obj.containsKey("kd")) {
        gain_command_.kd = obj["kd"].as<float>();
        result.new_kd = true;
        result.flag = CheckResultFlag::kNewCommand;
      }
      if (obj.containsKey("max_current")) {
        max_current_ = obj["max_current"].as<float>();
        result.new_max_current = true;
        result.flag = CheckResultFlag::kNewCommand;
      }
      if (obj.containsKey("activations")) {
        auto json_array = obj["activations"].as<JsonArray>();
        result.flag = CopyJsonArray(json_array, activations_);
        if (result.flag == CheckResultFlag::kNewCommand) {
          result.new_activation = true;
        }
        return result;
      }
      if (obj.containsKey("zero")) {
        if (obj["zero"].as<bool>()) {
          result.flag = CheckResultFlag::kNewCommand;
          result.do_zero = true;
        }
      }
      if (obj.containsKey("idle")) {
        if (obj["idle"].as<bool>()) {
          result.flag = CheckResultFlag::kNewCommand;
          result.do_idle = true;
        }
      }
    }
  }
  return result;
}

ActuatorPositionVector CommandInterpreter::LatestPositionCommand() {
  return position_command_;
}

ActuatorActivations CommandInterpreter::LatestActivations() {
  return activations_;
}

float CommandInterpreter::LatestKp() { return gain_command_.kp; }
float CommandInterpreter::LatestKd() { return gain_command_.kd; }

float CommandInterpreter::LatestMaxCurrent() { return max_current_; }

void CommandInterpreter::Flush() { reader_.FlushStream(); }
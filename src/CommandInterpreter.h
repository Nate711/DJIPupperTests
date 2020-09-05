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
  bool new_cartesian_position = false;
  bool new_kp = false;
  bool new_kd = false;
  bool new_cartesian_kp = false;
  bool new_cartesian_kd = false;
  bool new_max_current = false;
  bool new_activation = false;
  bool do_zero = false;
  bool do_idle = false;
  CheckResultFlag flag = CheckResultFlag::kNothing;
};

class CommandInterpreter {
 private:
  ActuatorPositionVector position_command_;
  ActuatorPositionVector cartesian_position_command_;
  ActuatorActivations activations_;

  PDGains3x3 cartesian_gain_command_;
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

  ActuatorPositionVector LatestCartesianPositionCommand();

  ActuatorActivations LatestActivations();

  // Empty the input buffer
  void Flush();

  float LatestKp();
  float LatestKd();
  BLA::Matrix<3, 3> LatestCartesianKp3x3();
  BLA::Matrix<3, 3> LatestCartesianKd3x3();
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
      if (obj.containsKey("cart_pos")) {
        auto json_array = obj["cart_pos"].as<JsonArray>();
        result.flag = CopyJsonArray(json_array, cartesian_position_command_);
        if (result.flag == CheckResultFlag::kNewCommand) {
          result.new_cartesian_position = true;
        }
      }
      if (obj.containsKey("cart_kp")) {
        auto json_array = obj["cart_kp"].as<JsonArray>();
        std::array<float, 3> kp_gains;
        result.flag = CopyJsonArray(json_array, kp_gains);
        cartesian_gain_command_.kp(0, 0) = kp_gains[0];
        cartesian_gain_command_.kp(1, 1) = kp_gains[1];
        cartesian_gain_command_.kp(2, 2) = kp_gains[2];
        if (result.flag == CheckResultFlag::kNewCommand) {
          result.new_cartesian_kp = true;
        }
      }
      if (obj.containsKey("cart_kd")) {
        auto json_array = obj["cart_kd"].as<JsonArray>();
        std::array<float, 3> kd_gains;
        result.flag = CopyJsonArray(json_array, kd_gains);
        cartesian_gain_command_.kd(0, 0) = kd_gains[0];
        cartesian_gain_command_.kd(1, 1) = kd_gains[1];
        cartesian_gain_command_.kd(2, 2) = kd_gains[2];
        if (result.flag == CheckResultFlag::kNewCommand) {
          result.new_cartesian_kd = true;
        }
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

ActuatorPositionVector CommandInterpreter::LatestCartesianPositionCommand() {
  return cartesian_position_command_;
}

ActuatorActivations CommandInterpreter::LatestActivations() {
  return activations_;
}

float CommandInterpreter::LatestKp() { return gain_command_.kp; }
float CommandInterpreter::LatestKd() { return gain_command_.kd; }
BLA::Matrix<3, 3> CommandInterpreter::LatestCartesianKp3x3() {
  return cartesian_gain_command_.kp;
}
BLA::Matrix<3, 3> CommandInterpreter::LatestCartesianKd3x3() {
  return cartesian_gain_command_.kd;
}

float CommandInterpreter::LatestMaxCurrent() { return max_current_; }

void CommandInterpreter::Flush() { reader_.FlushStream(); }
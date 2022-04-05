#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <CommandInterpreter.h>
#include <Streaming.h>

#include "DataLogger.h"
#include "DriveSystem.h"
#include "Utils.h"

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 10000;       // micros, 100hz
const int HEADER_DELAY = 5000;   // millis
const int CONTROL_DELAY = 1000;  // micros
const int IMU_DELAY = 5000; // micros
constexpr int IMU_FILTER_FREQUENCY = 1000000 / IMU_DELAY; // Hz
const float MAX_TORQUE = 2.0;
PDGains DEFAULT_GAINS = {8.0, 2.0};

const bool ECHO_COMMANDS = false;
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;

const uint32_t kLogSize = 1000;
const uint32_t kNumAttributes = 1 + 6 + 7 * 12;
DataLogger<kLogSize, kNumAttributes> logger;

// Example json message with default start and stop characters: <{"kp":2.0}>
// use_msgpack: true, use default arguments for the rest
CommandInterpreter interpreter(true);
DrivePrintOptions options;

long last_command_ts;
long last_imu_ts;
long last_print_ts;
long last_header_ts;

bool print_debug_info = true;
bool print_header_periodically = false;

void setup(void) {
  Serial.begin(500000);
  pinMode(13, OUTPUT);

  // Wait 1 second before turning on. This allows the motors to boot up.
  for (int i = 0; i < 4; i++) {
    digitalWrite(13, HIGH);
    delay(125);
    digitalWrite(13, LOW);
    delay(125);
  }

  drive.SetupIMU(IMU_FILTER_FREQUENCY);

  last_command_ts = micros();
  last_print_ts = micros();
  last_header_ts = millis();

  ////////////// Runtime config /////////////////////
  drive.SetMaxCurrent(MAX_TORQUE);
  options.delimiter = ',';
  options.print_delay_micros = PRINT_DELAY;
  options.header_delay_millis = HEADER_DELAY;
  options.positions = true;
  options.velocities = true;
  options.currents = false;             // last actual current
  options.position_references = false;  // last commanded position
  options.velocity_references = false;
  options.current_references = false;
  options.last_current = false;  // last commanded current

  // Set behavioral options
  drive.SetPositionKp(DEFAULT_GAINS.kp);
  drive.SetPositionKd(DEFAULT_GAINS.kd);
  drive.SetIdle();
  drive.PrintHeader(options);

  interpreter.Flush();

  // Activating the motors on bootup is dumb but it allows you to see debug
  // information
  drive.SetActivations({1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  drive.SetMaxCurrent(0.0);

  // FOR TESTING ONLY
  // drive.SetMaxCurrent(6.0);
  // drive.ZeroCurrentPosition();
  // drive.SetCartesianPositions(drive.DefaultCartesianPositions());
  // drive.SetCartesianKp3x3({5000.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0});  //[A/m]
  // drive.SetCartesianKd3x3({75.0, 0, 0, 0, 0.0, 0, 0, 0, 0.0});  // [A /
  // (m/s)]

  drive.PrintHeader(options);

  // drive.ExecuteHomingSequence();
}

void loop() {
  drive.CheckForCANMessages();
  CheckResult r = interpreter.CheckForMessages();
  if (r.flag == CheckResultFlag::kNewCommand) {
    // Serial << "Got new command." << endl;
    if (r.new_position) {
      drive.SetJointPositions(interpreter.LatestPositionCommand());
      if (ECHO_COMMANDS) {
        Serial << "Position command: " << interpreter.LatestPositionCommand()
               << endl;
      }
    }
    if (r.new_cartesian_position) {
      drive.SetCartesianPositions(interpreter.LatestCartesianPositionCommand());
      if (ECHO_COMMANDS) {
        Serial << "Cartesian position command: "
               << interpreter.LatestCartesianPositionCommand();
      }
    }
    if (r.new_kp) {
      drive.SetPositionKp(interpreter.LatestKp());
      if (ECHO_COMMANDS) {
        Serial << "Kp: " << interpreter.LatestKp() << endl;
      }
    }
    if (r.new_kd) {
      drive.SetPositionKd(interpreter.LatestKd());
      if (ECHO_COMMANDS) {
        Serial.print("Kd: ");
        Serial.println(interpreter.LatestKd(), 4);
      }
    }
    if (r.new_cartesian_kp) {
      drive.SetCartesianKp3x3(interpreter.LatestCartesianKp3x3());
      if (ECHO_COMMANDS) {
        Serial << "Cartesian Kp: " << interpreter.LatestCartesianKp3x3()
               << endl;
      }
    }
    if (r.new_cartesian_kd) {
      drive.SetCartesianKd3x3(interpreter.LatestCartesianKd3x3());
      if (ECHO_COMMANDS) {
        Serial << "Cartesian Kd: " << interpreter.LatestCartesianKd3x3()
               << endl;
      }
    }
    if (r.new_feedforward_force) {
      auto ff = interpreter.LatestFeedForwardForce();
      drive.SetFeedForwardForce(Utils::ArrayToVector<12, 12>(ff));
      if (ECHO_COMMANDS) {
      Serial << "Feed forward: " << interpreter.LatestFeedForwardForce()
             << endl;
      }
    }
    if (r.new_max_current) {
      drive.SetMaxCurrent(interpreter.LatestMaxCurrent());
      if (ECHO_COMMANDS) {
        Serial << "Max Current: " << interpreter.LatestMaxCurrent() << endl;
      }
    }
    if (r.new_activation) {
      drive.SetActivations(interpreter.LatestActivations());
      if (ECHO_COMMANDS) {
        Serial << "Activations: " << interpreter.LatestActivations() << endl;
      }
    }
    if (r.do_zero) {
      drive.ZeroCurrentPosition();
      if (ECHO_COMMANDS) {
        Serial << "Setting current position as the zero point" << endl;
      }
    }
    if (r.do_idle) {
      drive.SetIdle();
      if (ECHO_COMMANDS) {
        Serial << "Setting drive to idle." << endl;
      }
    }
    if (r.do_homing) {
      drive.ExecuteHomingSequence();
      if (ECHO_COMMANDS) {
        Serial << "Homing axes." << endl;
      }
    }
    if (r.new_debug) {
      print_debug_info = interpreter.LatestDebug();
    }
  }
  if (micros() - last_imu_ts >= IMU_DELAY) {
    drive.UpdateIMU();
    last_imu_ts = micros();
  }

  if (micros() - last_command_ts >= CONTROL_DELAY) {
    drive.Update();
    last_command_ts = micros();
  }

  if (print_debug_info) {
    if (micros() - last_print_ts >= options.print_delay_micros) {
      // drive.PrintStatus(options);
      // logger.AddData(drive.DebugData());
      drive.PrintMsgPackStatus(options);
      last_print_ts = micros();
    }
    if (print_header_periodically) {
      if (millis() - last_header_ts >= options.header_delay_millis) {
        drive.PrintHeader(options);
        last_header_ts = millis();
      }
    }
  }
}

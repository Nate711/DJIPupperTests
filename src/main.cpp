#include <Arduino.h>
#include <ArduinoJson.h>
#include <BasicLinearAlgebra.h>
#include <CommandInterpreter.h>
#include <FlexCAN_T4.h>
#include <NonBlockingSerialBuffer.h>
#include <Streaming.h>

#include "C610Bus.h"
#include "DriveSystem.h"

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 200;      // millis
const int HEADER_DELAY = 5000;   // millis
const int CONTROL_DELAY = 1000;  // micros
const float MAX_TORQUE = 2.0;
PDGains DEFAULT_GAINS = {8, 0.004};
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;

// Example json message with default start and stop characters: <{"kp":2.0}>
// use_msgpack: true, use default arguments for the rest
CommandInterpreter interpreter(true);
DrivePrintOptions options;

long last_command_ts;
long last_print_ts;
long last_header_ts;

void setup(void) {
  Serial.begin(115200);
  delay(400);

  last_command_ts = micros();
  last_print_ts = millis();
  last_header_ts = millis();

  ////////////// Runtime config /////////////////////
  drive.SetMaxCurrent(MAX_TORQUE);
  options.print_delay_millis = PRINT_DELAY;
  options.header_delay_millis = HEADER_DELAY;
  options.position_references = false;
  options.velocity_references = false;
  options.current_references = false;
  options.velocities = false;
  options.currents = false;

  // Set behavioral options
  drive.SetPositionGains(DEFAULT_GAINS);
  // No motors are activated on startup, and the drive system will start up in
  // idle mode

  drive.SetIdle();
  drive.ActivateAll();

  drive.PrintHeader(options);

  interpreter.Flush();

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // TODO: TEST ONLY
  // drive.DeactivateAll();

  // drive.ActivateActuator(0);
  // drive.ActivateActuator(1);
  // drive.ActivateActuator(2);
  // drive.ActivateActuator(3);
  // drive.ActivateActuator(4);
  // drive.ActivateActuator(5);

  // drive.SetMaxCurrent(1.0);
  // drive.ZeroCurrentPosition();
  // drive.SetCartesianPositions(drive.DefaultCartesianPositions());
  // drive.SetCartesianKp({500.0, 500.0, 0.0});  // [A/m]
  // drive.SetCartesianKd({0.2, 0.2, 0.05});     // [A / (m/s)]

  // drive.SetCartesianPositions(drive.CartesianPositions({0, 0.79, -1.57}));
  // Serial << "TEST 1" << endl;
  // Serial << drive.cartesian_position_reference_ << endl;
}

void loop() {
  drive.CheckForCANMessages();
  CheckResult r = interpreter.CheckForMessages();
  if (r.flag == CheckResultFlag::kNewCommand) {
    // Serial << "Got new command." << endl;
    if (r.new_position) {
      drive.SetAllPositions(interpreter.LatestPositionCommand());
      Serial << "Position command: " << interpreter.LatestPositionCommand()
             << endl;
    }
    if (r.new_cartesian_position) {
      drive.SetCartesianPositions(interpreter.LatestCartesianPositionCommand());
      Serial << "Cartesian position command: "
             << interpreter.LatestCartesianPositionCommand();
    }
    if (r.new_kp) {
      drive.SetPositionKp(interpreter.LatestKp());
      Serial << "Kp: " << interpreter.LatestKp() << endl;
    }
    if (r.new_kd) {
      drive.SetPositionKd(interpreter.LatestKd());
      Serial.print("Kd: ");
      Serial.println(interpreter.LatestKd(), 4);
    }
    if (r.new_cartesian_kp) {
      drive.SetCartesianKp3x3(interpreter.LatestCartesianKp3x3());
      Serial << "Cartesian Kp: " << interpreter.LatestCartesianKp3x3() << endl;
    }
    if (r.new_cartesian_kd) {
      drive.SetCartesianKd3x3(interpreter.LatestCartesianKd3x3());
      Serial << "Cartesian Kd: " << interpreter.LatestCartesianKd3x3() << endl;
    }
    if (r.new_max_current) {
      drive.SetMaxCurrent(interpreter.LatestMaxCurrent());
      Serial << "Max Current: " << interpreter.LatestMaxCurrent() << endl;
    }
    if (r.new_activation) {
      drive.SetActivations(interpreter.LatestActivations());
      Serial << "Activations: " << interpreter.LatestActivations() << endl;
    }
    if (r.do_zero) {
      drive.ZeroCurrentPosition();
      Serial << "Setting current position as the zero point" << endl;
    }
    if (r.do_idle) {
      drive.SetIdle();
      Serial << "Setting drive to idle." << endl;
    }
  }

  if (micros() - last_command_ts >= CONTROL_DELAY) {
    // TODO: characterize maximum lag between control updates
    drive.Update();
    last_command_ts = micros();
  }

  // TODO: turn this printing on and off with msgpack/json commands
  if (millis() - last_print_ts >= options.print_delay_millis) {
    drive.PrintStatus(options);
    last_print_ts = millis();
  }

  if (millis() - last_header_ts >= options.header_delay_millis) {
    drive.PrintHeader(options);
    last_header_ts = millis();
  }
}

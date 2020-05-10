#define NUM_C610S 8
const uint32_t ID_ONE_TO_EIGHT = 0x200;

/////////////// OUTWARD ///////////////
void torqueToBytes(int16_t torque, uint8_t& upper, uint8_t& lower) {
  upper = (torque >> 8) & 0xFF;
  lower = torque & 0xFF;
}

//void sendTorqueCommand(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>& can_bus, int32_t torque, uint8_t id) {
//  torque = constrain(torque, -32000, 32000); // prevent overflow of int16_t
//  sendTorqueCommand(can_bus, int16_t(torque), id);
//}

void sendTorqueCommand(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>& can_bus, int32_t torque, uint8_t id) {
  // only for id 1 through 4
  int16_t torque16 = int16_t(constrain(torque, -32000, 32000)); // prevent overflow of int16_t
  CAN_message_t msg;
  msg.id = ID_ONE_TO_EIGHT;
  for (uint8_t i = 0; i < 8; i++ ) msg.buf[i] = 0;
  uint8_t upper_idx = (id - 1) * 2;
  uint8_t lower_idx = (id - 1) * 2 + 1;
  torqueToBytes(torque16, msg.buf[upper_idx], msg.buf[lower_idx]);
  can_bus.write(msg);
}

////////////// INWARD /////////////
void interpretC610Message(const CAN_message_t &msg, int32_t &pos, int32_t &vel, int32_t &torque) {
  pos = uint16_t((msg.buf[0] << 8) | msg.buf[1]);
  vel = int16_t((msg.buf[2] << 8) | msg.buf[3]);
  torque = int16_t((msg.buf[4] << 8) | msg.buf[5]);
}

struct MotorState{
  // Position
  int32_t counts_per_rev;
  uint8_t initialized_mechanical_angle;
  int32_t rotations;
  int32_t last_pos_measurement;
  int32_t counts;

  // Velocity
  int32_t velocity;
  // Torque
  int32_t torque;
};

void initializeMotorState(MotorState& ms, int32_t counts_per_rev = 8192) {
  ms.counts_per_rev = counts_per_rev;
  ms.initialized_mechanical_angle = false;
  ms.rotations = 0;
  ms.last_pos_measurement = 0;
  ms.counts = 0;

  ms.velocity = 0;

  ms.torque = 0;
}

void initializeMotorStates(MotorState states[], uint8_t num_escs, int32_t counts_per_rev = 8192) {
  for(uint8_t i=0; i < num_escs; i++) {
    initializeMotorState(states[i], counts_per_rev);
  }
}

void updateMotorState(MotorState& ms, int32_t pos_measurement, int32_t velocity_measurement, int32_t torque_measurement) {
  // Initial setup
  if (!ms.initialized_mechanical_angle) {
    ms.initialized_mechanical_angle = true;
    ms.last_pos_measurement = pos_measurement;
  }

  // Position
  int32_t delta = int32_t(pos_measurement) - int32_t(ms.last_pos_measurement);
  if (delta > ms.counts_per_rev / 2) { // Crossed from >= 0 counts to <= 8191 counts. Could also trigger if spinning super fast (>2000rps)
    ms.rotations -= 1;
  } else if (delta < - ms.counts_per_rev / 2) { // Crossed from <= 8191 counts to >= 0 counts. Could also trigger if spinning super fast (>2000rps)
    ms.rotations += 1;
  }
  ms.counts = ms.rotations * ms.counts_per_rev + pos_measurement;
  ms.last_pos_measurement = pos_measurement;

  // Velocity
  ms.velocity = velocity_measurement;

  // Torque
  ms.torque = torque_measurement;
}
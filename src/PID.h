struct PDGAINS {
  float kp;
  float kd;
};

void pid(int32_t &torque_command, int32_t measurement_pos, int32_t measurement_vel, int32_t reference_pos, int32_t reference_vel, PDGAINS gains) {
  torque_command = gains.kp * (reference_pos - measurement_pos) + gains.kd * (reference_vel - measurement_vel);
}

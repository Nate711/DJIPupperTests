#pragma once
#include "Arduino.h"
// #include <array>
#include <BasicLinearAlgebra.h>

template <uint32_t LOG_SIZE, uint32_t NUM_ATTRIBUTES>
class DataLogger {
  private:
    BLA::Matrix<LOG_SIZE, NUM_ATTRIBUTES> data_;
    uint32_t write_index_;
  public:
    DataLogger();
    void AddData(BLA::Matrix<NUM_ATTRIBUTES> data);
    void PrintData(HardwareSerial &serial);
};

// #include "DataLogger.h"
template <uint32_t LOG_SIZE, uint32_t NUM_ATTRIBUTES>
DataLogger<LOG_SIZE, NUM_ATTRIBUTES>::DataLogger() {
  data_.Fill(0.0);
}

template <uint32_t LOG_SIZE, uint32_t NUM_ATTRIBUTES>
void DataLogger<LOG_SIZE, NUM_ATTRIBUTES>::AddData(BLA::Matrix<NUM_ATTRIBUTES> data) {
  for(uint8_t i = 0; i < NUM_ATTRIBUTES; i++) {
    data_(write_index_, i) = data(i);
  }
  write_index_++;
  if(write_index_ >= LOG_SIZE) {
    write_index_ = 0;
  }
}

template <uint32_t LOG_SIZE, uint32_t NUM_ATTRIBUTES>
void DataLogger<LOG_SIZE, NUM_ATTRIBUTES>::PrintData(HardwareSerial &serial) {
  for(uint32_t row = 0; row < LOG_SIZE; row++) {
    for (uint8_t i = 0; i < NUM_ATTRIBUTES; i++) {
      serial.print(data_(row, i));
      serial.print(",");
    }
    serial.println();
  }
}
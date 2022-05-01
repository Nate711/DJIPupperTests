#include "IMU.h"

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

IMU::IMU()
{
}

void IMU::Setup(int filter_frequency)
{
  if(!icm.begin_SPI(ICM_CS)) {
    Serial.println("Failed to find ICM20649 chip");
  }
  delay(50);
  icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
  icm.setAccelRateDivisor(1); // 562.5Hz output data rate
  icm.setGyroRateDivisor(1); // 562.5Hz output data rate
  filter.begin(filter_frequency);
}

void IMU::Update()
{
  //  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp);

  filter.updateIMU(- gyro.gyro.x * 180 / PI,
                   - gyro.gyro.y * 180 / PI,
                   gyro.gyro.z * 180 / PI,
                   - accel.acceleration.x,
                   - accel.acceleration.y,
                   accel.acceleration.z);

  yaw = filter.getYaw() * PI / 180;
  pitch = filter.getPitch() * PI / 180;
  roll = filter.getRoll() * PI / 180;
  roll_rate = - gyro.gyro.x;
  pitch_rate = - gyro.gyro.y;
  yaw_rate = gyro.gyro.z;
}
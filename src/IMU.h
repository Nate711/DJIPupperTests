#pragma once

#include <BasicLinearAlgebra.h>
#include <MahonyAHRS.h>
#include "ICM_20948.h"

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SPI_PORT SPI
#define CS_PIN 10

class IMU {
    private:
        Mahony filter;
        Adafruit_ICM20649 icm;

    public:
        IMU();
        void Setup(int filter_frequency);
        void Update();
        float yaw;        // [radians]
        float pitch;      // [radians]
        float roll;       // [radians]
        float yaw_rate;   // [radians/sec]
        float pitch_rate; // [radians/sec]
        float roll_rate;  // [radians/sec]
};
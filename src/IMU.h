#pragma once

#include <BasicLinearAlgebra.h>
#include <MahonyAHRS.h>
#include "ICM_20948.h"

#define SPI_PORT SPI
#define CS_PIN 10

class IMU {
    private:
        ICM_20948_SPI myICM;
        Mahony filter;
    public:
        IMU();
        void Setup();
        void Update();
        float yaw;
        float pitch;
        float roll;
        float yaw_rate;
        float pitch_rate;
        float roll_rate;
};
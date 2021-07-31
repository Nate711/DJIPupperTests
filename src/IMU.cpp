#include "IMU.h"

IMU::IMU() {}

void IMU::Setup() {
    SPI_PORT.begin();
    myICM.begin( CS_PIN, SPI_PORT );
    filter.begin(1000);
}

void IMU::Update() {
   if( myICM.dataReady() ){
    myICM.getAGMT();
    filter.update(
        myICM.gyrY(), myICM.gyrX(), -myICM.gyrZ(),
        myICM.accY(), myICM.accX(), -myICM.accZ(),
        myICM.magY(), myICM.magX(), -myICM.magZ()
    );

    yaw = filter.getYaw() * PI/180;
    pitch = filter.getPitch() * PI/180;
    roll = filter.getRoll() * PI/180;
    yaw_rate = -myICM.gyrZ() * PI/180;
    pitch_rate = myICM.gyrX() * PI/180;
    roll_rate = myICM.gyrY() * PI/180;
  }
}
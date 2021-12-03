#include "IMU.h"

IMU::IMU() {}

void IMU::Setup(int filter_frequency) {
    SPI_PORT.begin();
    myICM.begin( CS_PIN, SPI_PORT, 1000000);
    filter.begin(filter_frequency);
}

void IMU::Update() {
   // Takes about 0.3ms but that is enough to interrupt PD
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
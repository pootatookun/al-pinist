#include "Wire.h"

const int8_t MPU_addr = 0x68;
float temperature;



void IMU_Registers()
{

    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_addr); //Start communication with the MPU-6050.
    Wire.write(0x1B);                 //We want to write to the GYRO_CONFIG register (1B hex).
    Wire.write(0x08);                 //Set the register bits as 00001000 (500dps full scale).
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_addr); //Start communication with the MPU-6050.
    Wire.write(0x1C);                 //We want to write to the ACCEL_CONFIG register (1A hex).
    Wire.write(0x10);                 //Set the register bits as 00010000 (+/- 8g full scale range).
    Wire.endTransmission(true);       //End the transmission with the gyro.

    Wire.beginTransmission(MPU_addr); //Start communication with the MPU-6050.
    Wire.write(0x1A);                 //We want to write to the CONFIG register (1A hex).
    Wire.write(0x03);                 //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
    Wire.endTransmission(true);
}

void IMU_read()
{
    Wire.beginTransmission(MPU_addr); //Start communicating with the MPU-6050
    Wire.write(0x3B);                 //Send the requested starting register
    Wire.endTransmission();           //End the transmission
    Wire.requestFrom(MPU_addr, 14 , true);   //Request 14 bytes from the MPU-6050
    while (Wire.available() < 14)
        ;                                         //Wait until all the bytes are received
    data.acc_x = Wire.read() << 8 | Wire.read();       //Add the low and high byte to the acc_x variable
    data.acc_y = Wire.read() << 8 | Wire.read();       //Add the low and high byte to the acc_y variable
    data.acc_z = Wire.read() << 8 | Wire.read();       //Add the low and high byte to the acc_z variable
    temperature = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the temperature variable
    data.gyro_pitch = Wire.read() << 8 | Wire.read();  //Add the low and high byte to the gyro_x variable
    data.gyro_roll = Wire.read() << 8 | Wire.read();   //Add the low and high byte to the gyro_y variable
    data.gyro_yaw = Wire.read() << 8 | Wire.read();    //Add the low and high byte to the gyro_z variable
     
    data.gyro_yaw *= -1;
}



#include "Arduino.h"
#include "SPI.h"
#include "nRF24L01.h"
#include "RF24.h"

#define CSN 10
#define CE 9
RF24 radio(CE, CSN);
const uint64_t address = 0xB00B00B00BLL;

struct Data_Package
{
    uint8_t Wrapper0 = 69;
    uint8_t Wrapper1 = 255;
    uint8_t Wrapper2 = 165;

    uint16_t A_BYTE = 0;

    uint16_t Throttle;
    uint16_t Yaw;
    uint16_t Pitch;
    uint16_t Roll;
    uint16_t servo;
    uint8_t function_1;
    uint8_t function_2;
    uint8_t transmission_bit;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_pitch;
    int16_t gyro_roll;
    int16_t gyro_yaw;

} data;

uint8_t size_of_struct = sizeof(Data_Package);
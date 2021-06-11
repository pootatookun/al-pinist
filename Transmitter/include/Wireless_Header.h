#include "Arduino.h"
#include "SPI.h"
#include "nRF24L01.h"
#include "RF24.h"

#define CSN 10
#define CE  8
RF24 radio( CE , CSN);
const uint64_t address = 0xB00B00B00BLL;   

unsigned long Millis;
float Battery;
uint8_t counter = 0 ;
boolean effecient = false;

struct Data_Package{
    
    uint16_t Throttle;
    uint16_t Yaw;
    uint16_t Pitch;
    uint16_t Roll;
    uint16_t servo;
    uint8_t  function_1;
    uint8_t  function_2;
    uint8_t  transmisson_bit = 1 ;
} data ;
//uint8_t size_struct = sizeof(Data_Package);
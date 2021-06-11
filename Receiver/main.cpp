#include "Wireless_Header.h"
//#include "IMU.h"
uint32_t loop_timer;

void Radio_read();
void r_imu(char a, char g);
void throttle();

void setup()
{
 
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.openReadingPipe(1, address);
  radio.startListening();

  while (!data.transmission_bit)
  {
    digitalWrite(2, LOW);
    delay(80);
    digitalWrite(2, HIGH);
    Radio_read();
    delay(80);
  }

  Serial.begin(2000000);

  // IMU_Registers();
  loop_timer = micros() + 1500;
}

void loop()
{

  data.transmission_bit = 0;

  Radio_read();

  if (data.transmission_bit)
    PORTD |=  1<<2;//0x04;
  else
    PORTD &= !(1<<2);// 0xfB;

  //IMU_read();

  // Serial.println();
  // Serial.print(" transmission:");
  // Serial.print(data.transmission_bit);
  //r_imu('a', 'g');
  // throttle();

  if (Serial.availableForWrite() > 29)
    Serial.write(&(data.Wrapper0), 30);

  while(micros() < loop_timer);
  loop_timer = micros() + 1500;
}

void Radio_read()
{

  if (radio.available())
  {
    radio.read(&(data.Throttle), 13);
  }
}

void r_imu(char a, char g)
{
  if (a == 'a')
  {
    Serial.print("  r.ax:");
    Serial.print(data.acc_x);
    Serial.print("  r.ay");
    Serial.print(data.acc_y);
    Serial.print("  r.az:");
    Serial.print(data.acc_z);
  }
  if (g == 'g')
  {

    Serial.print("  r.g.yaw:");
    Serial.print(data.gyro_yaw);
    Serial.print("  r.g.pitch:");
    Serial.print(data.gyro_pitch);
    Serial.print("  r.g.roll:");
    Serial.print(data.gyro_roll);
  }
}

void throttle()
{
  Serial.print("  throttle:");
  Serial.print(data.Throttle);
  Serial.print("  pitch:");
  Serial.print(data.Pitch);
  Serial.print("  roll:");
  Serial.print(data.Roll);
  Serial.print("  yaw:");
  Serial.print(data.Yaw);
}
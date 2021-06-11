#include "Wireless_Header.h"

void Radio_Config();
void Indicator_LED();
void Function_Switch();
void Battery_Check();

void throttle();

void setup()
{
  Battery_Check();

  ICR1 = 15625;
  OCR1A = 15620;

  Radio_Config();

 //Serial.begin(2000000);

  DDRD = (0 << 2) | (0 << 3);
  PORTD = (1 << 2) | (1 << 3);
  data.function_1 = 0;
  data.function_2 = 0;
}

void loop()
{
  if (!effecient)
  {
    Millis = millis() + 15000;
    effecient = true;
  }

  data.Throttle = analogRead(A0);
  data.Yaw = 1023 - analogRead(A4);
  data.Roll = analogRead(A2);
  data.Pitch = analogRead(A3);
  data.servo = analogRead(A1);
// Serial.println();
//   throttle();

  Function_Switch();

  if (counter < 3 && millis() > Millis)
    Indicator_LED();

  radio.write(&(data.Throttle), 13);


}

void Radio_Config()
{

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.openWritingPipe(address);
  radio.stopListening();
}

void Indicator_LED()
{

  Battery = analogRead(A5);
  Battery = (Battery / 1023) * 5;

  if (Battery > 1.15)
  {
    effecient = false;
  }
  else if (Battery < 1.08 && counter < 3)
  {
    ICR1 = 1302;
    OCR1A = 651;
    counter++;
  }
  else if (Battery < 1.12 && counter < 2)
  {
    ICR1 = 5208;
    OCR1A = 2604;
    counter++;
    effecient = false;
  }
  else if (Battery < 1.15 && counter < 1)
  {
    ICR1 = 15625;
    OCR1A = 7813;
    counter++;
    effecient = false;
  }
}

void Function_Switch()
{

  if (!(PIND & 1 << 3) && data.function_1 == 0)
  {
    data.function_1 = 1;
    delay(200);
  }
  else if (!(PIND & 1 << 3) && data.function_1 == 1)
  {
    data.function_1 = 0;
    delay(200);
  }

  if (!(PIND & 1 << 2) && data.function_2 == 0)
  {
    data.function_2 = 1;
    delay(200);
  }
  else if (!(PIND & 1 << 2) && data.function_2 == 1)
  {
    data.function_2 = 0;
    delay(200);
  }
}

void Battery_Check()
{
  boolean status = true;
  DDRB |= 1 << 1;
  TCCR1A = B10100010;
  TCCR1B = B00011101;

  Battery = analogRead(A5);
  Battery = (Battery / 1023) * 5;

  while (true)
  {
    if (status && Battery > 1.21 )
      break;
    else if (status)
    {
      ICR1 = 1302;
      OCR1A = 651;
      status = false;
    }

    delay(1000);
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
  Serial.print("  Servo:");
  Serial.print(data.servo);
  Serial.print("  function 1:");
  Serial.print(data.function_1);
  Serial.print("  funstion 2:");
  Serial.print(data.function_2);
}
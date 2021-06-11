#include "Header.h"

int8_t counter;

double acc_x_cal, acc_y_cal;
double gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal;

float angle_pitch, angle_roll;
float acc_total_vector, angle_pitch_acc, angle_roll_acc, angle_yaw_acc;

int16_t esc_1, esc_2, esc_3, esc_4;
uint16_t pid_pitch_setpoint, pid_roll_setpoint, pid_yaw_setpoint;
float esc_setpoint, servo_setpoint;

uint32_t loop_timer = 0;

int start = 0;

void RAW_PID();
void TO_DO_();
void esc();

void Angle_calculation();
void esc1_esc4();
void servo1_4();

void setup()
{

  TIMER_GPIO_setup();
  TIMER_REGISTER_setup();

  // Serial2.begin(1000000);
  Serial.begin(2000000);

  while (!data.transmission_bit || data.Throttle > 10)
  {
    digitalWrite(PB11, LOW);
    delay(80);
    digitalWrite(PB11, HIGH);
    Get_data_Serial();
    Serial_Buffer_CLEAR_64B();
    start = 0;
    esc_setpoint = 0;
    servo_setpoint = 0;
    delay(80);
  }

  // for (uint16_t i = 0; i < 3000; i++)
  // {

  //   Get_data_Serial();
  //   Serial_Buffer_CLEAR_64B();
  //   acc_x_cal += data.Acc_x;
  //   acc_y_cal += data.Acc_y;
  //   gyro_pitch_cal += data.Gyro_pitch;
  //   gyro_roll_cal += data.Gyro_roll;
  //   gyro_yaw_cal += data.Gyro_yaw;
  //   delay(4);
  // }
  // acc_x_cal /= 2000;
  // acc_y_cal /= 2000;
  // gyro_pitch_cal /= 2000;
  // gyro_roll_cal /= 2000;
  // gyro_yaw_cal /= 2000;

  loop_timer = micros() + 2083;
  TIM3->CNT = 18000;
  TIM4->CNT = 18000;
}

void loop()
{
  // Serial2.println();
  data.transmission_bit = 0;
  Get_data_Serial();
  // data.Acc_x -= acc_x_cal;
  // data.Acc_y -= acc_y_cal;
  // data.Acc_x *= -1;
  // data.Gyro_pitch -= gyro_pitch_cal;
  // data.Gyro_roll -= gyro_roll_cal;
  // data.Gyro_yaw -= gyro_yaw_cal;

  Serial_Buffer_CLEAR_64B();
  // Serial2.print(" function1:");
  // Serial2.print(data.function_1);

  if (data.transmission_bit)
    GPIOB->ODR |= GPIO_ODR_ODR11;
  else
    GPIOB->ODR &= !GPIO_ODR_ODR11;

  if (!data.transmission_bit)
  {
    counter++;
    if (counter > 8)
    {
      TIM4->CCR2 = 1000;
      TIM4->CCR1 = 1000;
      TIM4->CCR4 = 1000;
      TIM4->CCR3 = 1000;

      TIM3->CCR1 = 1000;
      TIM3->CCR2 = 1000;
      TIM3->CCR3 = 1000;
      TIM3->CCR4 = 1000;

      while (true)
      {
        Get_data_Serial();
        Serial_Buffer_CLEAR_64B();
        if (data.Throttle < 40 && data.servo < 40 && data.transmission_bit)
        {
          start = 0;
          esc_setpoint = 0;
          servo_setpoint = 0;
          counter = 0;
          break;
        }
        delay(2);
      }
    }
  }
  else
    counter = 0;

  if (data.function_1 == 1)
  {
    TIM4->CCR2 = 1000;
    TIM4->CCR1 = 1000;
    TIM4->CCR4 = 1000;
    TIM4->CCR3 = 1000;

    TIM3->CCR1 = 1000;
    TIM3->CCR2 = 1000;
    TIM3->CCR3 = 1000;
    TIM3->CCR4 = 1000;

    while (true)
    {

      Get_data_Serial();
      Serial_Buffer_CLEAR_64B();
      if (data.Throttle < 40 && data.servo < 40 && data.function_1 == 0)
      {
        start = 0;
        esc_setpoint = 0;
        servo_setpoint = 0;
        break;
      }
      delay(2);
    }
  }

  if (start == 0 && data.Throttle < 40 && data.Yaw < 40)
    start = 1;

  if (start == 1 && data.Throttle < 40 && data.Yaw > 450)
  {
    start = 2;
    esc_setpoint = 0;
    servo_setpoint = 0;
  }

  if (start == 2 && data.Throttle < 40 && data.Yaw > 950)
    start = 0;

  //Angle_calculation();
  //RAW_PID();

  if (start == 2)
  {
    esc1_esc4();
    servo1_4();
  }

  // Serial2.print(" setpoint:");
  // Serial2.print(esc_setpoint);

  // Serial2.print(" pitch:");
  // Serial2.print(data.Pitch);
  // Serial2.print(" bit:");
  // Serial2.print(data.transmission_bit);

  TO_DO_();

  // esc();
  // Serial2.print(" counter:");
  // Serial2.print(counter);
  // Serial2.print(" transmission:");
  // Serial2.print(data.transmission_bit);

  while (micros() < loop_timer)
    ;
  loop_timer = micros() + 2083;
}

void TO_DO_()
{
  if (start == 2)
  {
    if (data.Throttle > 800)
      data.Throttle = 800;        //We need some room to keep full col at full throttle.
    esc_1 = 1000 + esc_setpoint;  //1000 + data.Throttle; // + pid_pitch_setpoint; //- pid_roll_setpoint; //- pid_yaw_setpoint; //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = 1000 + data.Throttle; // - pid_pitch_setpoint; // - pid_roll_setpoint; //+ pid_yaw_setpoint; //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = 1000 + data.Throttle; //- pid_pitch_setpoint; //+ pid_roll_setpoint; //- pid_yaw_setpoint; //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = 1000 + esc_setpoint;  //1000 + data.Throttle; // + pid_pitch_setpoint; // + pid_roll_setpoint; //+ pid_yaw_setpoint; //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1070)
      esc_1 = 1070; //Keep the motors running.
    if (esc_2 < 1070)
      esc_2 = 1070; //Keep the motors running.
    if (esc_3 < 1070)
      esc_3 = 1070; //Keep the motors running.
    if (esc_4 < 1070)
      esc_4 = 1070; //Keep the motors running.

    if (esc_1 > 2000)
      esc_1 = 2000; //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)
      esc_2 = 2000; //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)
      esc_3 = 2000; //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)
      esc_4 = 2000;
  }
  else
  {
    esc_setpoint = 0;
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  TIM4->CCR2 = esc_1;
  TIM4->CCR1 = 2500 - servo_setpoint; // (data.servo * 2); //2020; //2400;//500 + ((int)angle_pitch * 17);//500 + (data.servo * 2);
  TIM4->CCR4 = esc_4;
  TIM4->CCR3 = 500 + servo_setpoint; //(data.servo * 2); //980; //600;//2500 -((int)angle_pitch * 17); //2500 - (data.servo * 2);

  TIM3->CCR1 = esc_2;
  TIM3->CCR2 = 500 + (data.servo * 2) - 150;
  TIM3->CCR3 = esc_3;
  TIM3->CCR4 = 2500 - (data.servo * 2);
}

void RAW_PID()
{
  pid_pitch_setpoint = 0;

  if (data.Pitch > 532)
    pid_pitch_setpoint = data.Pitch - 532;
  else if (data.Pitch < 516)
    pid_pitch_setpoint = data.Pitch - 516;

  pid_roll_setpoint = 0;

  if (data.Roll > 508)
    pid_roll_setpoint = data.Roll - 508;
  else if (data.Roll < 492)
    pid_roll_setpoint = data.Roll - 492;

  pid_yaw_setpoint = 0;

  if (data.Yaw > 511)
    pid_yaw_setpoint = (data.Yaw - 511);
  else if (data.Yaw < 495)
    pid_yaw_setpoint = (data.Yaw - 495);
}

void esc()
{
  Serial2.print(" esc1:");
  Serial2.print(esc_1);
  Serial2.print(" esc2:");
  Serial2.print(esc_2);
  Serial2.print(" esc3:");
  Serial2.print(esc_3);
  Serial2.print(" esc4:");
  Serial2.print(esc_4);
}

void Angle_calculation()
{

  //Gyro angle calculations
  //0.00002827 = 1 / (250Hz / 65.5)
  angle_pitch += (float)data.Gyro_pitch * 0.0000318;
  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)data.Gyro_roll * 0.0000318;
  //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.0000004935 = 0.00002827 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)data.Gyro_yaw * 0.0000005551); //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)data.Gyro_yaw * 0.0000005551); //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((data.Acc_x * data.Acc_x) + (data.Acc_y * data.Acc_y) + (data.Acc_z * data.Acc_z));

  if (abs(data.Acc_y) < acc_total_vector)
    angle_pitch_acc = atan2((float)data.Acc_y, (float)data.Acc_z) * 57.296; //Calculate the pitch angle.

  if (abs(data.Acc_x) < acc_total_vector)
    angle_roll_acc = atan((float)data.Acc_x / acc_total_vector) * 57.296; //Calculate the roll angle.

  //Complimentary filter
  angle_pitch = angle_pitch * 0.99 + angle_pitch_acc * 0.01;
  angle_roll = angle_roll * 0.99 + angle_roll_acc * 0.01;
}

void esc1_esc4()
{

  if (data.Pitch > 950)
    esc_setpoint += 0.1;

  if (data.Pitch < 100)
    esc_setpoint -= 0.1;
}

void servo1_4()
{
  if (data.Roll > 950)
    servo_setpoint += 2;

  if (data.Roll < 100)
    servo_setpoint -= 2;
}
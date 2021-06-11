#include "Header.h"

HardwareSerial Serial2(PA3, PA2);
int start = 0;

float rangep, ranger, rangey;
int16_t esc_1, esc_2, esc_3, esc_4;
double gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal;
double acc_x_cal, acc_y_cal;
uint32_t loop_timer = 0;

float pitch_level_adjust, roll_level_adjust;
int32_t acc_total_vector;
float angle_pitch_acc, angle_roll_acc, angle_pitch, angle_roll;

void TO_DO_();
void Angle_calculation();

void setup()
{

  TIMER_GPIO_setup();
  TIMER_REGISTER_setup();

  delay(6000);

  Serial2.begin(1000000);
  Serial.begin(2000000);  

  for (uint16_t i = 0; i < 5000; i++)
  {

    Get_data_Serial();
    Serial_Buffer_CLEAR_64B();
    acc_x_cal += data.Acc_x;
    acc_y_cal += data.Acc_y;
    gyro_pitch_cal += data.Gyro_pitch;
    gyro_roll_cal += data.Gyro_roll;
    gyro_yaw_cal += data.Gyro_yaw;
    delay(4);
  }
  acc_x_cal /= 5000;
  acc_y_cal /= 5000;
  gyro_pitch_cal /= 5000;
  gyro_roll_cal /= 5000;
  gyro_yaw_cal /= 5000;

  loop_timer = micros() + 1851;
  TIM3->CNT = 18000;
  TIM4->CNT = 18000;
}

void loop()
{
  Get_data_Serial();
  data.Acc_x -= acc_x_cal;
  data.Acc_y -= acc_y_cal;
  data.Acc_x *= -1;
  data.Gyro_pitch -= gyro_pitch_cal;
  data.Gyro_roll -= gyro_roll_cal;
  data.Gyro_yaw -= gyro_yaw_cal;

  Serial_Buffer_CLEAR_64B();

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)data.Gyro_roll / 65.5) * 0.3);    //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)data.Gyro_pitch / 65.5) * 0.3); //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)data.Gyro_yaw / 65.5) * 0.3);       //Gyro pid input is deg/sec.

  Angle_calculation();

  pitch_level_adjust = (angle_pitch)*15; //Calculate the pitch angle correction.
  roll_level_adjust = (angle_roll)*15;   //Calculate the roll angle correction.

  //For starting the motors: throttle low and yaw left (step 1).
  if (start == 0 && data.Throttle < 40 && data.Yaw < 40)
    start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && data.Throttle < 40 && data.Yaw > 450)
  {
    start = 2;
    angle_pitch = angle_pitch_acc; //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  if (start == 2 && data.Throttle < 40 && data.Yaw > 950)
  {
    start = 0;
  }

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 139 degrees per second ( (500-8)/3 = 139d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (data.Pitch > 532)
    pid_pitch_setpoint = data.Pitch - 532;
  else if (data.Pitch < 516)
    pid_pitch_setpoint = data.Pitch - 516;

  pid_pitch_setpoint -= pitch_level_adjust; //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;
  //Divide the setpoint for the PID pitch coller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 139 degrees per second ( (500-8)/3 = 139d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (data.Roll > 508)
    pid_roll_setpoint = data.Roll - 508;
  else if (data.Roll < 492)
    pid_roll_setpoint = data.Roll - 492;

  pid_roll_setpoint -= roll_level_adjust; //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;
  //Divide the setpoint for the PID roll coller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 139 degrees per second ( (500-8)/3 = 139d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (data.Yaw > 60)
  { //Do not yaw when turning off the motors.
    if (data.Yaw > 511)
      pid_yaw_setpoint = (data.Yaw - 511) / 3.0;
    else if (data.Yaw < 495)
      pid_yaw_setpoint = (data.Yaw - 495) / 3.0;
  }

  calculate_pid();
  TO_DO_();


  while (micros() < loop_timer)
    ;
  loop_timer = micros() + 1851;
}

void TO_DO_()
{
  if (start == 2)
  {
    if (data.Throttle > 800)
      data.Throttle = 800;                                                              //We need some room to keep full col at full throttle.
    esc_1 = 1000 + data.Throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = 1000 + data.Throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = 1000 + data.Throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = 1000 + data.Throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1060)
      esc_1 = 1060; //Keep the motors running.
    if (esc_2 < 1060)
      esc_2 = 1060; //Keep the motors running.
    if (esc_3 < 1060)
      esc_3 = 1060; //Keep the motors running.
    if (esc_4 < 1060)
      esc_4 = 1060; //Keep the motors running.

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

    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  TIM4->CCR2 = esc_1;
  TIM4->CCR1 = 2500 - (data.servo * 2);
  TIM4->CCR4 = esc_4;
  TIM4->CCR3 = 500 + (data.servo * 2);

  TIM3->CCR1 = esc_2;
  TIM3->CCR2 = 500 + (data.servo * 2);
  TIM3->CCR3 = esc_3;
  TIM3->CCR4 = 2500 - (data.servo * 2);
}

void Angle_calculation()
{

  //Gyro angle calculations
  //0.00002827 = 1 / (540Hz / 65.5)
  angle_pitch += (float)data.Gyro_pitch * 0.00002827;
  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += (float)data.Gyro_roll * 0.00002827;
  //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.0000004935 = 0.00002827 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  angle_pitch -= angle_roll * sin((float)data.Gyro_yaw * 0.0000004935); //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin((float)data.Gyro_yaw * 0.0000004935); //If the IMU has yawed transfer the pitch angle to the roll angel.

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



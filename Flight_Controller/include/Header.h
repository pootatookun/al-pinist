#include "Arduino.h"
#include "PID.h"

struct Data_Package
{    
    uint8_t A_BIT;
    uint16_t Throttle;
    uint16_t Yaw;
    uint16_t Pitch;
    uint16_t Roll;
    uint16_t servo;
    uint8_t function_1;
    uint8_t function_2;
    int16_t Acc_x;
    int16_t Acc_y;
    int16_t Acc_z;
    int16_t Gyro_pitch;
    int16_t Gyro_roll;
    int16_t Gyro_yaw;   
    
} data;

uint8_t size_of_struct = sizeof(Data_Package);

void TIMER_GPIO_setup()
{

    RCC->APB2ENR = RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR = RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    GPIOA->CRL = GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;
    GPIOB->CRL = GPIO_CRL_MODE0_1 | GPIO_CRL_CNF0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_CNF1_1 | GPIO_CRL_MODE6_1 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;
    GPIOB->CRH = GPIO_CRH_MODE8_1 | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;
}

void TIMER_REGISTER_setup()
{
    TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    TIM3->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE | (0b110 << 12) | TIM_CCMR1_OC2PE;
    TIM3->CCMR2 = (0b110 << 4) | TIM_CCMR2_OC3PE | (0b110 << 12) | TIM_CCMR2_OC4PE;
    TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM4->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    TIM4->CCMR1 = (0b110 << 4) | TIM_CCMR1_OC1PE | (0b110 << 12) | TIM_CCMR1_OC2PE;
    TIM4->CCMR2 = (0b110 << 4) | TIM_CCMR2_OC3PE | (0b110 << 12) | TIM_CCMR2_OC4PE;
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM3->PSC = 104;
    TIM3->ARR = 16666;
    TIM3->CCR1 = 1000;
    TIM3->CCR2 = 1000;
    TIM3->CCR3 = 1000;
    TIM3->CCR4 = 1000;

    TIM4->PSC = 104;
    TIM4->ARR = 16666;
    TIM4->CCR1 = 1000;
    TIM4->CCR2 = 1000;
    TIM4->CCR3 = 1000;
    TIM4->CCR4 = 1000;
}

void Serial_Buffer_CLEAR_64B()
{

    for (int i = 0; i < 64; i++)
    {
        Serial.read();
    }
}

void Get_data_Serial()
{

    while (Serial.available() > 0)
    {

        if (Serial.read() == 69 && Serial.read() == 255 && Serial.read() == 165)
        {
            Serial.readBytes(&(data.A_BIT), 26);
            break;
        }
    }
}



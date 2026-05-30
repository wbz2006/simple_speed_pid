#include "app_motor.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>

/*pid参数*/
float Target, Actual, Out;
float Kp = 0.3f, Ki= 0.2f, Kd = 0.0f;
float Error0, Error1, ErrorInt;

float total_angle = 0.0f;

uint8_t pid_control_flag = 0;

void Motor_PID_Init(void)
{
    /*外设初始化*/
    Motor_Init(&hmotorA);
    Encoder_Init(&hencoderA);
}

float Motor_PID_calc(void)
{
    float last_actual = Actual;
    Actual = Encoder_GetDeltaCount(&hencoderA);

    if (fabs(Actual - last_actual) < 2)
    {
        Actual = last_actual;
    }

    Error1 = Error0;			
	Error0 = Target - Actual;
    

    if (Ki != 0)				
	{
		ErrorInt += Error0;		
	}
	else						
	{
		ErrorInt = 0;			
	}


	Out = Kp * Error0 + Ki * ErrorInt + Kd * (Error0 - Error1);
    
    
    if (Out > 100) {Out = 100;}
    if (Out < -100) {Out = -100;}
  
    return Out;
}

void Motor_PID_Task(void)
{
    if (pid_control_flag == 1)
    {
        pid_control_flag  = 0;
        
        Target = AD_GetValue()* 2 - 100;
        if (Target == 0)
        {
            Motor_SetSpeed(&hmotorA, 0);
        }
        else
        {
            Motor_SetSpeed(&hmotorA, Motor_PID_calc());
        }
    }
}

void Motor_PID_Data_Transmit(void)
{
    char uart_buf[40];
    int len = sprintf(uart_buf, "%d,%d,%d\r\n",(int16_t)Target, (int16_t)Actual, (int16_t)Out);
    HAL_UART_Transmit(&huart5, (uint8_t *)uart_buf, len, HAL_MAX_DELAY);
    HAL_Delay(SPEED_MEASURE_PERIOD);
}







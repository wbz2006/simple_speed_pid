#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "stm32f4xx_hal.h"

extern uint8_t pid_control_flag;


void Motor_PID_Init(void);
void Motor_PID_Task(void);
void Motor_PID_Data_Transmit(void);

#endif /* APP_MOTOR_H */
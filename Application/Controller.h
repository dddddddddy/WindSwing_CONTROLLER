#ifndef _CONTROLLER_
#define _CONTROLLER_
#include "Gpio.h"
#include "delay.h"
#include "USART.h"
#include "WB_LCD.h"

void Loop(void);
void PID1(float setpoint,float angle_x);
void PID2(float setpoint,float angle_x);
void UpdateDuty(void);
void Controller(void);
void Controller_1(void);
void Controller_2(void);
void Controller_3(void);
void Controller_4(void);
void Controller_5(void);

	
#endif

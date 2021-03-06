/*******************************************************************************
  * Company: Wang Electronic Technology Co., Ltd.
  ******************************************************************************
  * 文件名称：main.c
  * 功能说明：红牛板-LCD刷屏实验
  * 版    本：V1.0
  * 作    者：openmcu666	
  * 日    期: 2016-05-04
********************************************************************************
  * 文件名称：
  * 功能说明：
  * 版    本：
  * 更新作者:	
  * 日    期：
  * 更新原因：
********************************************************************************/
#include "Gpio.h"
#include "delay.h"
#include "USART.h"
#include "WB_LCD.h"
#include "Controller.h"
#include "mpu6050.h"
#include "Timer.h"
#include "Debug.h"
#include "Config.h"
#include "Filter.h"
int pwmy,pwmx;
int pwmy,pwmx;
unsigned char flag=0,setlong=0,setR=0,flag2=0,flag2_1,flag7=0;
float setang=0;
char rsetlong;
struct Infostruct Info ;
extern struct Parameterstruct Parameter;
/*****************************************************************************
**   Main Function  main()
******************************************************************************/
struct TestStruct Test;

int main(void)
{
	Info.Thr=6500;
	Info.Xout=0;
	Info.Yout=0;
	Info.setpointx=15;
	Info.setpointy=15;
  LED_Init();
  LCD_Init();
	USART1_Init();
	MPU_Init();					 
  Delay_Init();   
	Key_Init();
	reset();
	TIM5_Init();
	TIM_Init();
	Delay_ms(10000);
	//UnlockMotor();	
	LCD_DisplayStr(96,120,"Powered by UMVIEW.COM",RED,BLACK);
	LCD_DisplayStr(110,180,"梦想",RED,BLACK);
	LCD_DisplayStr(100, 150, "LCD-Test",RED,BLACK);
	//LCD_DisplayStr(50, 210, "旺宝电子-红牛开发板",RED,BLACK);
	Delay(3000);	
	//Delay(3000);
//			 TIM3->CCR1 = 10000;
//		 	 TIM3->CCR2 = 8000;
//		 	 TIM3->CCR3 = 5000;
//			 TIM3->CCR4 = 2000;
	for(;;)Loop();

}
/*
		 switch(x)
		 {
		   	 case 0:LCD_Clear(WHITE);break;
			   case 1:LCD_Clear(BLACK);break;
			   case 2:LCD_Clear(BLUE);break;
			   case 3:LCD_Clear(RED);break;
			   case 4:LCD_Clear(MAGENTA);break;
			   case 5:LCD_Clear(GREEN);break;
			   case 6:LCD_Clear(CYAN);break;

			   case 7:LCD_Clear(YELLOW);break;
			   case 8:LCD_Clear(BRRED);break;
			   case 9:LCD_Clear(GRAY);break;
			   case 10:LCD_Clear(LGRAY);break;
			   case 11:LCD_Clear(BROWN);break;
			 
		 }

*/

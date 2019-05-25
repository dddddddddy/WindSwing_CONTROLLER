#ifndef _CONFIG_
#define _CONFIG_
#include "Gpio.h"
#include "delay.h"
#include "USART.h"
#include "WB_LCD.h"
#include "Controller.h"
#include "mpu6050.h"
#include "Timer.h"
#include "Debug.h"
#include "SDS.h"
#define k1 0.995
#define ThrMax 8990
#define Xbias 2.0f
#define Ybias 2.0f
struct TestStruct 
{
	uint16_t testduty1;
	uint16_t testduty2;
	uint16_t testduty;
};
#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
struct  Infostruct{
	int16_t gx,gy,gz;
	int16_t ax,ay,az;
	float thx;
	float thy;
	float GyroxIntegral;
	float GyroyIntegral;
	float GyrozIntegral;
	float Gyrox,Gyroy,Gyroz;
	float anglex,angley;
	int16_t Xout,Yout;
	int16_t Thr;
	float setpointy;
	float setpointx;
	float deltax,deltay;
	float deltax_last,deltay_last;
	float R2;
	float angle3;
	float angle5;
};



struct Parameterstruct{
	uint32_t kp1,kd1;
	uint32_t kp2,kd2;
	uint32_t kp3,kd3;
	uint32_t kp4,kd4;
	uint32_t kp5,kd5 ;
};

#endif

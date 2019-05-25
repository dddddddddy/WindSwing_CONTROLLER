#include "Controller.h"
#include "Config.h"
#include "Filter.h"
#include "math.h"
extern struct TestStruct Test;
extern struct Infostruct Info;
extern struct PIDstruct PID;
extern struct Parameterstruct Parameter;
uint16_t i=0;
extern float SDS[4];
#define pai 3.1415926535
void TIM5_IRQHandler(void)
{
    if ( TIM_GetITStatus(TIM5 , TIM_IT_Update) != RESET )   
    {     
        TIM_ClearITPendingBit(TIM5 , TIM_FLAG_Update);   
				i++;
			if(i==5){
				Angle();
				Controller();
				UpdateDuty();
				LED_Toggle(1),i=0;
			}
    }     
}
void Controller(void){
	//Info.angley=Info.Yout;
	static float Yerror[2]={0},Xerror[2]={0};
	Yerror[1]=Info.angley+Ybias-Info.setpointy;
	Xerror[1]=Info.anglex+Xbias-Info.setpointx;
	Info.gy=LIMIT(Info.gy,-2000,2000);
	Info.gx=LIMIT(Info.gx,-2000,2000);
	//Info.Yout=(int16_t)(-Info.gy/2.0f*2.0+Yerror[1]*30);//(int16_t)(Yerror[1]*20.0);//+Info.gy*1.0);
	//Info.Yout=(int16_t)(Info.gy/2.0f);//*2.0+Yerror[1]*30);//(int16_t)(Yerror[1]*20.0);//+Info.gy*1.0);
	Info.Yout=(int16_t)(-Yerror[1]*50+Info.gy/0.3f);//(int16_t)(Yerror[1]*20.0);//+Info.gy*1.0);
	//Info.Xout=(int16_t)(-Xerror[1]*20);
	Info.Xout=(int16_t)(Info.gx/0.3f-Xerror[1]*50);
}
void  Controller_1(void){

	const float T = 10000.0;       
	static uint32_t time = 0 ;
	float Omega ;
	float f ;
	float angley ;
	float Expect_angley;

	time+=5;

	f = (float)time/T;
	Omega = 2*pai*f;
	angley = 30.0;
	Expect_angley=angley*sin(Omega);
	
	Info.deltay=Expect_angley-Info.angley-Ybias;

	Info.Yout=(int16_t)((Expect_angley-Info.angley-Ybias)* 70+(Info.deltay-Info.deltay_last)*0);
	Info.deltay_last=Info.deltay;
	
	if(time==10000)time=0;
	
}

void  Controller_2(void){

	const float T = 5000.0;       
	static uint32_t time = 0 ;
	float Omega = 0.0;
	float f = 0.0;
	float angley = 0;
	float Expect_angley;

	time+=5;

	f = (float)time/T;
	Omega = 2*pai*f;
	angley = atan(Info.R2/78.0)*57.2958f;
	Expect_angley=angley*sin(Omega);
	Info.deltay=Expect_angley-Info.angley;
	Info.Yout=(int16_t)(Expect_angley-Info.angley)* Parameter.kp2+(Info.deltay-Info.deltay_last)* Parameter.kd2;
	Info.deltay_last=Info.deltay;
	
	if(time==5000)time=0;
	
}
void  Controller_3(void){

	const float T = 5000;       
	static uint32_t time = 0 ;
	float Omega = 0.0;
	float f = 0.0;
	float angle=0 , anglex , angley ;
	float Expect_anglex,Expect_angley;

	time+=5;

	f = (float)time/T;
	Omega = 2*pai*f;

	angle = atan(15/78.0)*57.2958f;

	anglex=angle*cos(Info.angle3*0.017453);
	angley=angle*sin(Info.angle3*0.017453);
	
	Expect_anglex=anglex*cos(Omega);
	Expect_angley=angley*sin(Omega);
	
	Info.deltax=Expect_anglex-Info.anglex;
	Info.deltay=Expect_angley-Info.angley;
	
	Info.Xout=(int16_t)((Expect_angley-Info.angley)* Parameter.kp3+(Info.deltax-Info.deltax_last)* Parameter.kd3);
	Info.Yout=(int16_t)((Expect_angley-Info.angley)* Parameter.kp3+(Info.deltax-Info.deltax_last)* Parameter.kd3);
	
	Info.deltax_last=Info.deltax;
	Info.deltay_last=Info.deltay;
	
	if(time==5000)time=0;
	
}
void  Controller_4(void){

	Info.deltax=-Info.anglex;
	Info.deltay=-Info.angley;

	Info.Xout=(int16_t)((0-Info.angley)* Parameter.kp4+(Info.deltax-Info.deltax_last)* Parameter.kd4);
	Info.Yout=(int16_t)((0-Info.angley)* Parameter.kp4+(Info.deltax-Info.deltax_last)* Parameter.kd4);
	
	Info.deltax_last=Info.deltax;
	Info.deltay_last=Info.deltay;
	
}
void  Controller_5(void){

	const float T = 10000;       
	static uint32_t time = 0 ;
	float Omega = 0.0;
	float f = 0.0;
	float angle=0, anglex, angley ;
	float Expect_anglex,Expect_angley;
	
	time+=5;

	f = (float)time/T;
	Omega = 2*pai*f;
	
	angle = atan(Info.angle5/78.0)*57.2958f;
	
	anglex=angle*cos(Info.angle5*0.017453);
	angley=angle*sin(Info.angle5*0.017453);
	
	Expect_anglex=anglex*cos(Omega);
	Expect_angley=angley*sin(Omega+pai/2.0f);
	
	Info.deltax=Expect_anglex-Info.anglex;
	Info.deltay=Expect_angley-Info.angley;
	
	Info.Xout=(int16_t)((Expect_angley-Info.angley)* Parameter.kp5+(Info.deltax-Info.deltax_last)* Parameter.kd5);
	Info.Yout=(int16_t)((Expect_angley-Info.angley)* Parameter.kp5+(Info.deltax-Info.deltax_last)* Parameter.kd5);
	
	Info.deltax_last=Info.deltax;
	Info.deltay_last=Info.deltay;
	
	if(time==10000)time=0;
	
}
void UpdateDuty(void){
	uint16_t M[5]={0};
	M[1]=Info.Thr;
	M[2]=Info.Thr;
	M[3]=Info.Thr;
	M[4]=Info.Thr;
	if(Info.Yout>=0){
		M[1]+=Info.Yout;
		M[3]=Info.Thr;
		M[1]=LIMIT(M[1],Info.Thr,ThrMax);
		M[3]=LIMIT(M[3],Info.Thr,ThrMax);
	}else{
		M[3]-=Info.Yout;
		M[1]=Info.Thr;
		M[1]=LIMIT(M[1],Info.Thr,ThrMax);
		M[3]=LIMIT(M[3],Info.Thr,ThrMax);
	}
	if(Info.Xout>=0){
		M[2]+=Info.Xout;
		M[4]=Info.Thr;
		M[2]=LIMIT(M[2],Info.Thr,ThrMax);
		M[4]=LIMIT(M[4],Info.Thr,ThrMax);
	}else{
		M[4]-=Info.Xout;
		M[2]=Info.Thr;
		M[2]=LIMIT(M[2],Info.Thr,ThrMax);
		M[4]=LIMIT(M[4],Info.Thr,ThrMax);
	}
		 TIM3->CCR1 = M[1];
	 	 TIM3->CCR2 = M[2];
		 TIM3->CCR3 = M[3];
		 TIM3->CCR4 = M[4];
	
//			 TIM3->CCR1 = 0;
//	 		 TIM3->CCR2 = 0;
//		 	 TIM3->CCR3 = 0;
//			 TIM3->CCR4 = 0;
}
void Loop(){
 // MPU_Get_Gyroscope(&gx,&gy,&gz);
	//MPU_Get_Accelerometer(&ax,&ay,&az);
	
	SDS[0]=(int16_t)Info.ax;
	SDS[1]=(int16_t)(Info.angley+Ybias);
	SDS[2]=(int16_t)Info.gx;
	SDS[3]=(int16_t)Info.gy;
	Plot(SDS);
	Delay_ms(5);
	//LCD_ShowNum(10, 10, gx, 16, RED, BLACK);//显示数字
}
//void PID1(float setpoint,float angle_x){
//  float  iError,Errorsum,dError,last_iError;
//	PID.setpointone = setpoint;
//	iError=PID.setpointone-angle_x;
//	Errorsum+=iError;
//	dError=iError-last_iError;
//	last_iError = iError;
//	PID.duty1=(int16_t)(PID.Kp*iError+PID.Ki*Errorsum+PID.Kd*dError); 
//	PID.duty2=(int16_t)(PID.Kp*iError+PID.Ki*Errorsum+PID.Kd*dError); 
//}

//void PID2(float setpoint,float angle_x){
//	float  iError,Errorsum,dError,last_iError;
//	PID.setpointtwo = setpoint;
//	iError=PID.setpointtwo-angle_x;
//	Errorsum+=iError;
//	dError=iError-last_iError;
//	last_iError = iError;
//	PID.duty1=(int16_t)(PID.Kp*iError+PID.Ki*Errorsum+PID.Kd*dError); 
//	PID.duty2=(int16_t)(PID.Kp*iError+PID.Ki*Errorsum+PID.Kd*dError); 
//}
//void PID3(float setpoint,float angle_x,float angle_y){
//	float  iError,Errorsum,dError,last_iError;
//	float Ax,Ay;
//	Ax = angle_x*cos(angle*0.017453);	             //计算出X方向摆幅分量0.017453为弧度转换
//	Ay = angle_y*sin(angle*0.017453);
//	PID.setpointtwo = setpoint;
//	iError=PID.setpointtwo-angle_x;
//	Errorsum+=iError;
//	dError=iError-last_iError;
//	last_iError = iError;
//	PID.duty1=PID.Kp*iError+PID.Ki*Errorsum+PID.Kd*dError; 
//	PID.duty2=PID.Kp*iError+PID.Ki*Errorsum+PID.Kd*dError; 
//	
//}







//	

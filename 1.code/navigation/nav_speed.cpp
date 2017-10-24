#include "nav_speed.h"
#include "dev_ms5611.h"
#include "cal_adrc.h"
#include "dev_mavlink.h"
#include "math.h"
#include "imu.h"
#define   g 980.6f/4096.0f
NAV_V_ nav_v;

s16  h;
void NAV_V_::get_ms_v(float dt)
{
	 upvm=1;
	 float vfac =dt / (dt + 1 / (2 * M_PI * 4));
	 h=Ms5611.BaroAlt;
	 static float  lh=0,lv=0;
	 vm=(h-lh)/dt;
	 //debug.aux1=(s16)vm;
	 vm=vfac*vm+(1-vfac)*lv;
	 lh=h;
	 lv=vm;
	 //debug.aux3+=(s16)(imu.ea.z*dt);
	 //debug.aux2=(s16)vm;
}
void NAV_V_::get_a_v(float dt)
{
	va+=g*imu.ea.z*dt;
};

float k=0.01,i=0;

void NAV_V_::east(float dt)
{
 float e=0;
 static float sum=0;
 static u16 ms_flag=0;	
  static u8 ready=0;
  static u16 ms_up=0;
 GPIO_SetBits(GPIOE,GPIO_Pin_0);
	//评估气压预热成功
 if(abs(h)<500)
 {
	
	 ms_up++;
	 if(ms_up>250) 
	 {
		 ready=1;
		  ms_up=250;
		 
	 }
 }
	if(ready==1)
	{
 GPIO_ResetBits(GPIOE,GPIO_Pin_0); 
 GPIO_ResetBits(GPIOE,GPIO_Pin_2); 
 if(upvm==1)
  e=vm-vz;
	sum+=e;
  k=0.5*(1-(fabs(imu.ea.z)/4096))/100;
	//VAL_LIMIT(k,0.001,0.01);
	vz=vz+k*e+i*sum;
	vz=vz+g*imu.ea.z*dt;
	VAL_LIMIT(vz,-500,500);
 
	 debug.aux1=(s16)vz;
	 debug.aux2=(s16)vm;
	 debug.aux3=(s16)va;
   h_east(dt);
 }
}


void NAV_V_::h_east(float dt)
{
	
 float k=0.01,i=0;
 float e=0;
 static float sum=0;

 if(upvm==1)
  e=h-eh;
	sum+=e;
 
  k=(fabs(imu.ea.z)/4096);
	//VAL_LIMIT(k,0.01,0.15);
	eh=eh+k*e+i*sum;
	eh=eh+vz*dt;
//   debug.aux1=(s16)vz;
//	 debug.aux2=(s16)eh;
//	 debug.aux3=(s16)h;
 


}





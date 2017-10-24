#include "uav_adrc.h"
#include "cal_adrc.h"
#include "fal_initialize.h"
#include "imu.h"
#include "dev_mpu6050.h"
#include "uav_rc.h"
#include "cal_math.h"
#include "dev_moto.h"
#include "dev_mavlink.h"
#include "control_v.h"

enum {x=0,y,z,};
_ADRC  Adrc_;
Tracking_Diff  td[3];
Tracking_Diff  tdr[3];
Tracking_Diff  eptd[3];
Tracking_Diff  ottd[3];

void _ADRC::Init(void)
{
	// ADRC 
	fal_initialize();
  //角速度输入
  TrackingDiffInit(td[x],1000);
	TrackingDiffInit(td[y],1000);
	TrackingDiffInit(td[z],1000);
	//角加速度输入
	TrackingDiffInit(tdr[x],90);
	TrackingDiffInit(tdr[y],90);
	TrackingDiffInit(tdr[z],90);
	//遥控输入
	TrackingDiffInit(eptd[x],5);
	TrackingDiffInit(eptd[y],5);
	TrackingDiffInit(eptd[z],5);
	//姿态输入
	TrackingDiffInit(ottd[x],5);
	TrackingDiffInit(ottd[y],5);
	TrackingDiffInit(ottd[z],5);

	
};

lpid aglp[3];
float sgt;
	float epag[3]={0};
void _ADRC::Angle(float dt )
{
	sgt=dt;

	aglp[x].e0=0;aglp[x].e1=0;aglp[x].e2=0;
	aglp[y].e0=0;aglp[y].e1=0;aglp[y].e2=0;
	aglp[z].e0=0;aglp[z].e1=0;aglp[z].e2=0;
	
	aglp[x].bo=3.5;aglp[x].b1=0.215;aglp[z].b2=80;
	aglp[y].bo=3.5;aglp[y].b1=0.215;aglp[z].b2=80;
	aglp[z].bo=35;aglp[z].b1=0.1; aglp[z].b2=80;
	
	epag[x]=40.0f*deathzoom(Rc.g_CH[rol],30)/500.f;
	epag[y]=40.0f*deathzoom(Rc.g_CH[pit],30)/500.f;
	//======================================================
  if(Rc.g_Thr_Low)
	epag[z]=imu.angle.z;
  else	
	epag[z]+=deathzoom(Rc.g_CH[yaw],30)*0.001;
	epag[z]=To_180_degrees(epag[z]);
	//======================================================
	TrackingDiffCal(eptd[x],epag[x],0.2);
	TrackingDiffCal(eptd[y],-epag[y],0.2);
	TrackingDiffCal(eptd[z],epag[z],0.2);

	
	TrackingDiffCal(ottd[x],imu.angle.x,0.3);
	TrackingDiffCal(ottd[y],imu.angle.y,0.3);
	TrackingDiffCal(ottd[z],imu.angle.z,0.3);
   //p
	aglp[x].e0=eptd[x].v1-ottd[x].v1;
	aglp[y].e0=eptd[y].v1-ottd[y].v1;
	aglp[z].e0=eptd[z].v1-ottd[z].v1;
	//d
	float t=dt/(0.005f);
	aglp[x].e2=(eptd[x].v2-ottd[x].v2)/t;
	aglp[y].e2=(eptd[y].v2-ottd[y].v2)/t;
	aglp[z].e2=(eptd[z].v2-ottd[z].v2)/t;
	//i
	aglp[x].e1+=(aglp[x].e0*t);
	aglp[y].e1+=(aglp[y].e0*t);
	aglp[z].e1+=(aglp[z].e0*t);
	
	aglp[x].u=aglp[x].bo*aglp[x].e0+aglp[x].b1*aglp[x].e1+aglp[x].b2*aglp[x].e2;
	aglp[y].u=aglp[y].bo*aglp[y].e0+aglp[y].b1*aglp[y].e1+aglp[y].b2*aglp[y].e2;
	aglp[z].u=aglp[z].bo*aglp[z].e0+aglp[z].b1*aglp[z].e1+aglp[z].b2*aglp[z].e2;
//  debug.aux1=(s16)imu.angle.x;
//	debug.aux2=(s16)ottd[x].v1;
}

  lpid agulp[3];
  float agut;
void _ADRC::Angular(float dt)
{
	 u16 motor[4]={0};
	 float postureVal[4]={0}; 
	 float posagVal[4]={0}; 
	 float posaguVal[4]={0}; 
	 
	agulp[x].bo=0.175; agulp[x].b1=0.0003125;agulp[x].b2=0.625;
	agulp[y].bo=0.175; agulp[y].b1=0.0003125;agulp[y].b2=0.625;
	agulp[z].bo=0.55;  agulp[z].b1=0.0003125;agulp[z].b2=1;
	
	static float fl[3]={0};

	TrackingDiffCal(td[x],Mpu6050.gyr.x+0.1*td[x].v2,0.08);
	TrackingDiffCal(td[y],-Mpu6050.gyr.y+0.1*td[y].v2,0.08);
	TrackingDiffCal(td[z],-Mpu6050.gyr.z+0.1*td[z].v2,0.08);
 //===============================================================  
    float fe[3]={0};
		fe[x]=Mpu6050.gyr.x-fl[x];
 	 	fe[y]=-(Mpu6050.gyr.y-fl[y]);
 	 	fe[z]=-(Mpu6050.gyr.z-fl[z]);
   
    TrackingDiffCal(tdr[x],fe[x],0.4);
    TrackingDiffCal(tdr[y],fe[y],0.4);
    TrackingDiffCal(tdr[z],fe[z],0.4);
 
	  fl[x]=Mpu6050.gyr.x;
	  fl[y]=Mpu6050.gyr.y;
	  fl[z]=Mpu6050.gyr.z;
//===============================================================	
	//p
	agulp[x].e0= aglp[x].u*16.0f-td[x].v1;
	agulp[y].e0= aglp[y].u*16.0f-td[y].v1;
	agulp[z].e0= aglp[z].u*16.0f-td[z].v1;
	//d
	agut=dt;
  float t=dt/(0.0020f);
 	agulp[x].e2= -tdr[x].v1/t;
	agulp[y].e2= -tdr[y].v1/t;
	agulp[z].e2= -tdr[z].v1/t;
	// i
	agulp[x].e1+= agulp[x].e0*t;
	agulp[y].e1+= agulp[x].e0*t;
	agulp[z].e1+= agulp[x].e0*t;
	VAL_LIMIT(agulp[x].e1,-30000,30000);
	VAL_LIMIT(agulp[y].e1,-30000,30000);
	VAL_LIMIT(agulp[z].e1,-30000,30000);
	if(Rc.g_Thr_Low)
    for(u8 j=0;j<3;j++) agulp[j].e1=0;
	
	
	agulp[x].u=agulp[x].bo*agulp[x].e0+agulp[x].b1*agulp[x].e1+agulp[x].b2*agulp[x].e2;
	agulp[y].u=agulp[y].bo*agulp[y].e0+agulp[y].b1*agulp[y].e1+agulp[y].b2*agulp[y].e2;
	agulp[z].u=agulp[z].bo*agulp[z].e0+agulp[z].b1*agulp[z].e1+agulp[z].b2*agulp[z].e2;
	
	agulp[z].u=0;
	
  posaguVal[0] = -agulp[x].u +agulp[y].u  + agulp[z].u ;
	posaguVal[1] = +agulp[x].u +agulp[y].u  - agulp[z].u ;
	posaguVal[2] = +agulp[x].u -agulp[y].u  + agulp[z].u ;
	posaguVal[3] = -agulp[x].u -agulp[y].u  - agulp[z].u ;	
	
	posagVal[0] = -aglp[x].u +aglp[y].u  + aglp[z].u ;
	posagVal[1] = +aglp[x].u +aglp[y].u  - aglp[z].u ;
	posagVal[2] = +aglp[x].u -aglp[y].u  + aglp[z].u ;
	posagVal[3] = -aglp[x].u -aglp[y].u  - aglp[z].u ;
	
	
	for(u8 i=0;i<4;i++)//小要稳大要快
	   postureVal[i]=(1.0f-Rc.g_Weight[thr])*posaguVal[i]+Rc.g_Weight[thr]*posagVal[i]*0.125;
	
	
//	debug.aux1=(s16)fe[x];
//	debug.aux2=(s16)td[x].v2;
	
//	motor[0] = Rc.g_ThrVAL + (s16)(Rc.g_Weight[thr] *postureVal[0]) ;//油门越大姿态输出越大
//	motor[1] = Rc.g_ThrVAL + (s16)(Rc.g_Weight[thr] *postureVal[1]) ;
//	motor[2] = Rc.g_ThrVAL + (s16)(Rc.g_Weight[thr] *postureVal[2]) ;
//	motor[3] = Rc.g_ThrVAL + (s16)(Rc.g_Weight[thr] *postureVal[3]) ;
	
		motor[0] = ctrl.u + (s16)(Rc.g_Weight[thr] *postureVal[0]) ;//油门越大姿态输出越大
		motor[1] = ctrl.u + (s16)(Rc.g_Weight[thr] *postureVal[1]) ;
		motor[2] = ctrl.u + (s16)(Rc.g_Weight[thr] *postureVal[2]) ;
		motor[3] = ctrl.u + (s16)(Rc.g_Weight[thr] *postureVal[3]) ;
	
 for(u8 i=0;i<4;i++)
	{
    VAL_LIMIT(motor[i], 0,1500 );
	}

	if(Rc.g_CH[ksr]!=1)
	{
		 for(u8 i=0;i<4;i++)
		{
			motor[i]=0;
		}
	}
	Moto.SetPwm(motor);
}	

void AngularRote(float dt)
{
  static float lx=0;  // 90 h=0.4
	float e=Mpu6050.gyr.x-lx;
	lx=Mpu6050.gyr.x;
	TrackingDiffCal(tdr[x],e,0.4);
	
//	debug.aux1=(s16)e;
//	debug.aux2=(s16)tdr[x].v1;
};
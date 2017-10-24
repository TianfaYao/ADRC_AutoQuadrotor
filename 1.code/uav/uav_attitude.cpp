/*
  ******************************************************************************
  * @file    
  * @author  ytf
  * @version 
  * @date   
  * @brief  互补滤波实现高度融合
  *    
  *****************************************************************************
*/
#include "uav_attitude.h"
#include "cal_math.h"
#include "dev_mpu6050.h"
#include "dev_us100.h"
#include "uav_config.h"
#include "uav_rc.h"
#include "uav_pid.h"
#include "cal_math.h"
#include "imu.h"

_PID  z_dis[2]; 
_PID  z_spe[2];
#define   MAX_Z_DIS      10.0       //dM 
#define   MAX_Z_SPE      2        // 1ds/S
//以下所有参数可调
#define SPE_UL_KP            1
#define SPE_UL_KI           0.1
//超声波
#define DIS_UL_KP            1
#define DIS_UL_KI           0.1
//气压计
#define SPE_MS_KP            1
#define SPE_MS_KI           0.1

#define ACC_MS_KP            1
#define ACC_MS_KI           0.1

#define DIS_MS_KP            1
#define DIS_MS_KI           0.1

_ATTITUDE Attitude; 
void _ATTITUDE::Init(void)
{
  z_dis[ul].Init(z_dis[ul].PID.Kp,z_dis[ul].PID.Ki,z_dis[ul].PID.Kd);
  z_dis[ms].Init(z_dis[ms].PID.Kp,z_dis[ms].PID.Ki,z_dis[ms].PID.Kd);
	z_spe[ul].Init(z_spe[ul].PID.Kp,z_spe[ul].PID.Ki,z_spe[ul].PID.Kd);
  z_spe[ms].Init(z_spe[ms].PID.Kp,z_spe[ms].PID.Ki,z_spe[ms].PID.Kd);
	
}

void _ATTITUDE::GetDIS_SPE(float dt)
{
 		//加速度得到的加速度短时有很大的参考价值刷新频率较快           //预测
		//超生波得到的位置有很大的意义                                 //观测
    //气压计观测高度 准确但是刷新频率较慢                          //观测
	
  float SPE_Z_ul_dFactor =dt / (dt + 1 / (2 * M_PI * 20));
	float SPE_Z_ms_dFactor =dt / (dt + 1 / (2 * M_PI * 2));
	//气压计
  static 	float body_ms_DIS_z=0;
	static 	float lastbody_ms_DIS_z=0;
	static 	float lastbody_ms_DIS_z_fc=0;
	        float speErr_ms;
	static  float speErr_ms_I=0;
  static  float lastbody_ms_SPE_z=0;
		      float disErr_ms;
	static  float disErr_ms_I=0;
//交叉加速度
          float acccal_ms;//计算加速度
	        float accErr_ms;
	static  float accErr_ms_I=0;
  static  float lastbody_ms_ACC_z=0;
  static 	float lastbody_ms_SPE_fc=0;

	
	//超声波
	static float body_ul_DIS_z=0;
	static float lastbody_ul_DIS_z=0;
	
	static float lastbody_ul_SPE_z=0;
	       float speErr_ul;
	static float speErr_ul_I=0;
	       float disErr_ul;
	static float disErr_ul_I=0;
	

	//超生波
  body_ul_DIS_z=Us100.disFilt/10.0;                                      //M
	body_ul_SPE_z=(body_ul_DIS_z-lastbody_ul_DIS_z)/(dt*10.0);             // M/s
  body_ul_SPE_z=body_ul_SPE_z*SPE_Z_ul_dFactor+(1-SPE_Z_ul_dFactor)*lastbody_ul_SPE_z;
	lastbody_ul_SPE_z=body_ul_SPE_z;
	lastbody_ul_DIS_z=body_ul_DIS_z;
	//稳
	speErr_ul=body_ul_SPE_z-body_ul_SPE_z_fc;
	speErr_ul_I+=SPE_UL_KI*speErr_ul;
	body_ul_SPE_z_fc=body_ul_SPE_z_fc+SPE_UL_KP*speErr_ul+speErr_ul_I;  //误差修正积分
	
	body_ul_SPE_z_fc=body_ul_SPE_z_fc+imu.ea.z*dt;                       //加计直接积分
	
	//参数可调 时间可以往前移动毕竟加速计快超声很多倍
	disErr_ul=body_ul_DIS_z-body_ul_DIS_z_fc;
  disErr_ul_I+=DIS_UL_KI*disErr_ul_I;
	body_ul_DIS_z_fc=body_ul_DIS_z_fc+DIS_UL_KP*disErr_ul+disErr_ul_I;
	body_ul_DIS_z_fc=body_ul_DIS_z_fc+body_ul_SPE_z_fc*dt+imu.ea.z*dt*dt*0.5;
	
	
	//气压计 参数可以有很大的调整                 交叉融合相互抑制
	body_ms_DIS_z=Ms5611.BaroAlt/10.0;         //气压数据一定要低通
	body_ms_SPE_z=(body_ms_DIS_z-lastbody_ms_DIS_z)/(dt*10.0);   //dm/s
	body_ms_SPE_z=body_ms_SPE_z*SPE_Z_ms_dFactor+(1-SPE_Z_ms_dFactor)*lastbody_ms_SPE_z;
  lastbody_ms_DIS_z=body_ms_DIS_z;
	lastbody_ms_SPE_z=body_ms_SPE_z;
	
	//速度
	speErr_ms=body_ms_SPE_z-body_ms_SPE_z_fc;
	speErr_ms_I+=SPE_MS_KI*speErr_ms_I;
	body_ms_SPE_z_fc=body_ms_SPE_z_fc+SPE_MS_KP*speErr_ms+speErr_ms_I;//误差修正积分
	body_ms_SPE_z_fc=body_ms_SPE_z_fc+imu.ea.z*dt;  
	//body_ms_SPE_z_fc=body_ms_SPE_z_fc+(imu.ea.z+accErr_ms)*dt; 
	
//加速度
	acccal_ms=(body_ms_SPE_z_fc-lastbody_ms_SPE_fc);   // m/s
	accErr_ms=acccal_ms-imu.ea.z;                       //本来应该是0但是由于气压计的游走这个会发生该变
//	accErr_ms_I=accErr_ms_I+ ACC_MS_KI*accErr_ms*dt;      //对误差加速积分得到误差速度反馈到计算速度上去
	lastbody_ms_SPE_fc =body_ms_SPE_z_fc;                 
	//高度
	disErr_ms=body_ms_DIS_z-body_ms_DIS_z_fc;
	disErr_ms_I+=DIS_MS_KI*disErr_ms;
	body_ms_DIS_z_fc=body_ms_DIS_z_fc+DIS_MS_KP*disErr_ms+disErr_ms_I;
	body_ms_DIS_z_fc=body_ms_DIS_z_fc+body_ms_SPE_z_fc*dt+imu.ea.z*dt*dt*0.5;
	
	
	
if(fabs(body_ms_DIS_z_fc-lastbody_ms_DIS_z_fc)<2)//2dm
{
 ms_flag++;
	if(ms_flag>=200) //1s
	{
		ms_flag=200;  //安全
//	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	GPIO_ResetBits(GPIOE,GPIO_Pin_3); 
	}
}
else
{
	
 ms_flag=0;
}
	
}
//位置环
void _ATTITUDE::DISControl(float dt)
{
  static u16 count=0;
	static u8  ms_flag=0;
	if(Rc.g_CH[ksl]==dis_ul_z) //定高模式
	{
		//位置pid
	 z_dis[ul].PID.expect=MAX_Z_DIS;
	 z_dis[ul].GetErr(body_ul_DIS_z_fc);
//	 VAL_LIMIT(z_dis[ul].PID.err,-MAX_Z_DIS/4,MAX_Z_SPE/4);
//	 z_dis.Err_d(dt);
//	 VAL_LIMIT(z_dis.PID.ec,-MAX_Z_SPE,MAX_Z_SPE);
	 z_dis[ul].Err_i(1);
	 VAL_LIMIT(z_dis[ul].PID.err_i,-MAX_Z_DIS/4.0,MAX_Z_DIS/4.0);
	 z_dis[ul].Cal();
	}  else{z_dis[ul].PID.output=0;}
	
	
	if(Rc.g_CH[ksl]==dis_ms_z) //气压定高
	{
//位置pid
	 if(fabs(Rc.g_CH[thr])<30)	 
	 {
		 count++;
		 if(count>=200)
		 {
			 if(ms_flag==0)
			 z_dis[ms].PID.expect=body_ul_DIS_z_fc;
			 ms_flag=1;
			
		 }
	 }
	 else
	 {
	 ms_flag=0; //为下次定高做准备
	 z_dis[ms].PID.expect+=MAX_Z_SPE*(Rc.g_Weight[thr]-0.5)*dt;
	 }
	 z_dis[ms].GetErr(body_ul_DIS_z_fc);
	 z_dis[ms].Err_i(1);
	 VAL_LIMIT(z_dis[ms].PID.err_i,-MAX_Z_DIS/4.0,MAX_Z_DIS/4.0);
	 z_dis[ms].Cal();
	} else{
	z_dis[ms].PID.output=0;
	
	}
}

//速度环
void _ATTITUDE::SPEControl(float dt)
{ 
	
//超声定高
if(Rc.g_CH[ksl]==dis_ul_z)
{
	z_spe[ul].PID.expect=z_dis[ul].PID.output/100;
//	z_spe[ul].PID.expect=MAX_Z_SPE*(Rc.g_Weight[thr]-0.5)*2;
//	
//	VAL_LIMIT(z_spe[ul].PID.expect,-MAX_Z_SPE,MAX_Z_SPE);
	z_spe[ul].GetErr(body_ul_SPE_z_fc);
	z_spe[ul].PID.err=z_spe[ul].PID.err*100;
	z_spe[ul].PID.ec=imu.ea.z*100;             //加速度的测量值直接作为阻尼
	//z_spe.Err_d(dt);
	z_spe[ul].Cal();
	//混合输出
	z_spe[ul].PID.output=0.5*z_spe[ul].PID.output+0.5*z_dis[ul].PID.output;
	VAL_LIMIT(z_spe[ul].PID.output,-500,500);
  Rc.g_ThrVAL=Rc.g_ThrVAL+((1-Rc.g_Weight[thr])*z_spe[ul].PID.output);
}else{z_spe[ul].PID.output=0;}

//气压定高
if(Rc.g_CH[ksl]==dis_ms_z) 
{
	//z_spe[ms].PID.expect=z_dis[ms].PID.output/100;
	
	z_spe[ms].PID.expect=MAX_Z_SPE*(Rc.g_Weight[thr]-0.5)*2;
  VAL_LIMIT(z_spe[ms].PID.expect,-MAX_Z_SPE,MAX_Z_SPE);
	z_spe[ms].GetErr(body_ms_SPE_z_fc);
	z_spe[ms].PID.err=z_spe[ms].PID.err*100;
	z_spe[ms].PID.ec=imu.ea.z*100;             //加速度的测量值直接作为阻尼
	z_spe[ms].disErr_i(dt,MAX_Z_SPE/3.0);
	z_spe[ms].PID.offsetKi= z_spe[ms].PID.Ki*0.4*(1-fabs((Rc.g_Weight[thr]-0.5))*2);//
	z_spe[ms].Cal();
	//混合输出
	VAL_LIMIT(z_spe[ms].PID.output,-500,500);
  Rc.g_ThrVAL=Rc.g_ThrVAL+((1-Rc.g_Weight[thr])*z_spe[ms].PID.output);
}else{z_spe[ms].PID.output=0;}
//裸飞
if(Rc.g_CH[ksl]!=dis_ms_z&&Rc.g_CH[ksl]!=dis_ul_z)
{
//float fd=imu.ea.z*z_spe[ms].PID.Kd;
//VAL_LIMIT(fd,-100,100);
//Rc.g_ThrVAL=Rc.g_ThrVAL+((1-Rc.g_Weight[thr])*fd);
Rc.g_ThrVAL=Rc.g_ThrVAL+((1-Rc.g_Weight[thr]));
}
}
	

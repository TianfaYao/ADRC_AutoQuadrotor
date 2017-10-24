
/*
  ******************************************************************************
  * @file   
  * @author  ytf
  * @version 
  * @date   
  * @brief   任务执行文件，所有的任务都在这儿执行
  *    
  *****************************************************************************
*/
#include "uav_control.h"
#include "uav_pid.h"
#include "imu.h"
#include "dev_mpu6050.h"
#include "uav_rc.h"
#include "cal_math.h"
#include "dev_moto.h"
#include "dev_mavlink.h"



#define MAX_ANGLE              30
#define MAX_YAW_ANGULAR        150
#define MAX_ROLL_PITCH_ANGULAR 400
#define MAX_ANGLE              30
#define DATA_MIDLE             30//控制数据所做的死区


_PID 			qAngle[3];
_PID      qAngular[3];

_CONTROL Control;
void _CONTROL::Init(void)
{
//角度环控制参数初始化 从flash读取
	qAngle[x].Init(qAngle[x].PID.Kp,qAngle[x].PID.Ki,qAngle[x].PID.Kd);  //roll
	qAngle[y].Init(qAngle[y].PID.Kp,qAngle[y].PID.Ki,qAngle[y].PID.Kd);  //pit
	qAngle[z].Init(qAngle[z].PID.Kp,qAngle[z].PID.Ki,qAngle[z].PID.Kd);  //yaw
	
//角速度环控制参数初始化 从flash读取
	qAngular[x].Init(qAngular[x].PID.Kp,qAngular[x].PID.Ki,qAngular[x].PID.Kd);  //roll
	qAngular[y].Init(qAngular[y].PID.Kp,qAngular[y].PID.Ki,qAngular[y].PID.Kd);  //pit
	qAngular[z].Init(qAngular[z].PID.Kp,qAngular[z].PID.Ki,qAngular[z].PID.Kd);  //yaw
}

//角度环
void _CONTROL::Angle(float dt)
{
	qAngle[x].PID.expect=MAX_ANGLE*deathzoom(Rc.g_CH[rol],30)/500.f;
	qAngle[y].PID.expect=-MAX_ANGLE*deathzoom(Rc.g_CH[pit],30)/500.f;
	if(!Rc.g_Thr_Low)//油门值不低
	{
	 qAngle[z].PID.expect+=MAX_YAW_ANGULAR*deathzoom(Rc.g_CH[yaw],50)/500.f*dt;
	}
	else
	{
   qAngle[z].PID.expect += 1 *3.14 *dt *( imu.angle.z -qAngle[z].PID.expect );           //低通滤波
	}
		qAngle[x].PID.expect = To_180_degrees(qAngle[x].PID.expect);      //角度归到正负180
		qAngle[y].PID.expect = To_180_degrees(qAngle[y].PID.expect);      //角度归到正负180
	  qAngle[z].PID.expect = To_180_degrees(qAngle[z].PID.expect);      //角度归到正负180
	
	
	//得到误差
	 qAngle[x].GetErr(imu.angle.x);
	 qAngle[y].GetErr(imu.angle.y);
	 qAngle[z].GetErr(imu.angle.z);
	
	 qAngle[x].PID.err = To_180_degrees(qAngle[x].PID.err);      //角度误差归到正负180
	 qAngle[y].PID.err = To_180_degrees(qAngle[y].PID.err);      //角度误差归到正负180
	 qAngle[z].PID.err = To_180_degrees(qAngle[z].PID.err);      //角度误差归到正负180
		 
	//误差比例
	qAngle[x].PID.errWgight=abs(qAngle[x].PID.err/MAX_ANGLE);//0- 1
  qAngle[y].PID.errWgight=abs(qAngle[y].PID.err/MAX_ANGLE);
	qAngle[z].PID.errWgight=abs(qAngle[z].PID.err/MAX_ANGLE);
	
	 VAL_LIMIT(qAngle[x].PID.errWgight,0,1);
	 VAL_LIMIT(qAngle[y].PID.errWgight,0,1);
	 VAL_LIMIT(qAngle[z].PID.errWgight,0,1);
	 
//===========================================================================================
	 //比例动态参数调整 误差越大权值越接近1 然后将比例参数调大
//   qAngle[x].PID.offsetKp= qAngle[x].PID.Kp*0.1*qAngle[x].PID.errWgight;     //10%
//	 qAngle[y].PID.offsetKp= qAngle[y].PID.Kp*0.1*qAngle[y].PID.errWgight;     //10%
//	 qAngle[z].PID.offsetKp= qAngle[z].PID.Kp*0.1*qAngle[z].PID.errWgight;     //10%
	//限幅
	 //积分动态参数调整 误差越大积分的参数要越小 //外环I 需要消除静差所以其变化需范围需要相对大些
//	 qAngle[x].PID.offsetKi= qAngle[x].PID.Ki*0.4*(1-qAngle[x].PID.errWgight);     //10%
//	 qAngle[y].PID.offsetKi= qAngle[x].PID.Ki*0.4*(1-qAngle[x].PID.errWgight);     //10%
//	 qAngle[z].PID.offsetKi= qAngle[x].PID.Ki*0.4*(1-qAngle[x].PID.errWgight);     //10%
	 
	 
	//积分限幅
	qAngle[x].PID.limitI=MAX_ANGLE*0.5*Rc.g_Weight[thr];
	qAngle[y].PID.limitI=MAX_ANGLE*0.5*Rc.g_Weight[thr];
	qAngle[z].PID.limitI=MAX_ANGLE*0.5*Rc.g_Weight[thr];
	//微分
	qAngle[x].Err_d(dt); 
	qAngle[y].Err_d(dt);
	qAngle[z].Err_d(dt);
	
	if(Rc.g_CH[ksr]!=1)
	{
		for(u8 i=0;i<3;i++)
		{
			qAngle[i].PID.err_i=0;
		}
	}
	//反向积分
	qAngle[x].disErr_i(dt,qAngle[x].PID.limitI/3);
	qAngle[x].disErr_i(dt,qAngle[x].PID.limitI/3);
	qAngle[x].disErr_i(dt,qAngle[x].PID.limitI/3);
	//角度积分限幅
	VAL_LIMIT(qAngle[x].PID.err_i,-qAngle[x].PID.limitI,qAngle[x].PID.limitI);
	VAL_LIMIT(qAngle[y].PID.err_i,-qAngle[y].PID.limitI,qAngle[y].PID.limitI);
	VAL_LIMIT(qAngle[z].PID.err_i,-qAngle[z].PID.limitI,qAngle[z].PID.limitI);
	//误差限幅
  VAL_LIMIT(qAngle[x].PID.err,-90,90);
  VAL_LIMIT(qAngle[y].PID.err,-90,90);
  VAL_LIMIT(qAngle[z].PID.err,-90,90);
	//
	qAngle[x].Cal();
	qAngle[y].Cal();
	qAngle[z].Cal();	

}

//角速度环
void _CONTROL::Angular(float dt)
{

   float gyr_Err[3]={0};
	 float gyr_Err_d[3]={0};
   static float gyr_Err_i[3]={0};
	 static float last_gyr_Err[3]={0};
	 	//抗扰动
   float Anti_interference[3]={0};
	 gyr_Err[x]=Mpu6050.gyrdeg.x-last_gyr_Err[x];
	 gyr_Err[y]=Mpu6050.gyrdeg.y-last_gyr_Err[y];
	 gyr_Err[z]=Mpu6050.gyrdeg.z-last_gyr_Err[z];
	 
	 gyr_Err_i[x]+=gyr_Err[x];
	 gyr_Err_i[y]+=gyr_Err[y];
	 gyr_Err_i[z]+=gyr_Err[z];
	 
	 last_gyr_Err[x]=Mpu6050.gyrdeg.x;
	 last_gyr_Err[y]=Mpu6050.gyrdeg.y;
	 last_gyr_Err[z]=Mpu6050.gyrdeg.z;
	
	 for(u8 i=0;i<3;i++)
	   Anti_interference[i]=qAngular[x].PID.Ki*gyr_Err_i[i]+ qAngular[x].PID.Kd*gyr_Err[i];
	 
	 
	
	//内环期望
   qAngular[x].PID.expect=MAX_ROLL_PITCH_ANGULAR*(qAngle[x].PID.output/MAX_ANGLE);
	 qAngular[y].PID.expect=MAX_ROLL_PITCH_ANGULAR*(qAngle[y].PID.output/MAX_ANGLE);
	 qAngular[z].PID.expect=MAX_ROLL_PITCH_ANGULAR*(qAngle[z].PID.output/MAX_ANGLE);
	//内环期望限幅
	 VAL_LIMIT(  qAngular[x].PID.expect, -MAX_ROLL_PITCH_ANGULAR,MAX_ROLL_PITCH_ANGULAR);
	 VAL_LIMIT(  qAngular[x].PID.expect, -MAX_ROLL_PITCH_ANGULAR,MAX_ROLL_PITCH_ANGULAR);
	 VAL_LIMIT(  qAngular[x].PID.expect, -MAX_ROLL_PITCH_ANGULAR,MAX_ROLL_PITCH_ANGULAR);
	
	//内环误差
	//机体坐标系和传感器坐标系不一样
	 qAngular[x].GetErr(Mpu6050.gyrdeg.x*100);     // max 2000*0.017=34 ./s  =34   
	 qAngular[y].GetErr(-Mpu6050.gyrdeg.y*100);     //单位太小放大100倍方便条参数
	 qAngular[z].GetErr(-Mpu6050.gyrdeg.z*100);
	//内环误差权重
	 qAngular[x].PID.errWgight=abs(qAngular[x].PID.err)/MAX_ROLL_PITCH_ANGULAR;
	 qAngular[y].PID.errWgight=abs(qAngular[y].PID.err)/MAX_ROLL_PITCH_ANGULAR;
	 qAngular[z].PID.errWgight=abs(qAngular[z].PID.err)/MAX_YAW_ANGULAR;
	 //内环积分限幅
	//===============================================================================
	
	 VAL_LIMIT(qAngular[x].PID.errWgight,0,1);
	 VAL_LIMIT(qAngular[y].PID.errWgight,0,1);
	 VAL_LIMIT(qAngular[z].PID.errWgight,0,1);
	
//	 //比例动态参数调整 误差越大权值越接近1 然后将比例参数调大
//   qAngular[x].PID.offsetKp= qAngular[x].PID.Kp*0.1*qAngular[x].PID.errWgight;     //10%
//	 qAngular[y].PID.offsetKp= qAngular[y].PID.Kp*0.1*qAngular[y].PID.errWgight;     //10%
//	 qAngular[z].PID.offsetKp= qAngular[z].PID.Kp*0.1*qAngular[z].PID.errWgight;     //10%
//	//限幅
//	 //积分动态参数调整 误差越大积分的参数要越小
//	 qAngular[x].PID.offsetKi= qAngular[x].PID.Ki*0.1*(1-qAngular[x].PID.errWgight);     //10%
//	 qAngular[y].PID.offsetKi= qAngular[x].PID.Ki*0.1*(1-qAngular[x].PID.errWgight);     //10%
//	 qAngular[z].PID.offsetKi= qAngular[x].PID.Ki*0.1*(1-qAngular[x].PID.errWgight);     //10%
	 
	 
	 //油门越大惯性越大，需要更大的阻尼来抑制系统震荡
   qAngular[x].PID.offsetKd= (0.55+0.45*Rc.g_Weight[thr]);
   qAngular[y].PID.offsetKd= (0.55+0.45*Rc.g_Weight[thr]);
   qAngular[z].PID.offsetKd= (0.55+0.45*Rc.g_Weight[thr]);
	 //===============================================这样乱七八糟的玩早晚要玩坏 ========================
	 
	 qAngular[x].PID.limitI=MAX_ROLL_PITCH_ANGULAR*0.5*Rc.g_Weight[thr];
	 qAngular[y].PID.limitI=MAX_ROLL_PITCH_ANGULAR*0.5*Rc.g_Weight[thr];
	 qAngular[z].PID.limitI=MAX_YAW_ANGULAR*0.5*Rc.g_Weight[thr];
	
	 //内环微分
	 qAngular[x].Err_d(dt);
	 qAngular[y].Err_d(dt);
	 qAngular[z].Err_d(dt);
	//内环积分
	//内环积分限幅 
	qAngle[x].disErr_i(dt,qAngular[x].PID.limitI);
	qAngle[y].disErr_i(dt,qAngular[y].PID.limitI);
	qAngle[z].disErr_i(dt,qAngular[z].PID.limitI);	 
	//
   VAL_LIMIT( qAngular[x].PID.err_i, -qAngular[x].PID.limitI,qAngular[x].PID.limitI);
   VAL_LIMIT( qAngular[y].PID.err_i, -qAngular[y].PID.limitI,qAngular[y].PID.limitI);
   VAL_LIMIT( qAngular[z].PID.err_i, -qAngular[z].PID.limitI,qAngular[z].PID.limitI);
	 	if(Rc.g_CH[ksr]!=1)
	{
		for(u8 i=0;i<3;i++)
		{
			qAngular[i].PID.err_i=0;
		}
	}
	//内环计算
	
	 qAngular[x].Cal();
	 qAngular[y].Cal();
 	 qAngular[z].Cal();
  //混合输出
	 qAngular[x].PID.output= qAngular[x].PID.expect*(0.45+0.55*Rc.g_Weight[thr])*0.2+
	                          0.8*(qAngular[x].PID.output+Anti_interference[x]); //0-300
	 qAngular[y].PID.output= qAngular[y].PID.expect*(0.45+0.55*Rc.g_Weight[thr])*0.2+
	                            0.8*(qAngular[y].PID.output-Anti_interference[y]);
	 qAngular[z].PID.output= qAngular[z].PID.expect*(0.45+0.55*Rc.g_Weight[thr])*0.2+
	                             0.8*(qAngular[z].PID.output-Anti_interference[z]);
  //姿态油门分解输出映射到四个电机pwm
	 DecompositionOutput();
	//抗扰动
  //Anti-interference
	
}

void _CONTROL::DecompositionOutput(void)
{
  float postureVal[4];
	float curve[4];
	u16 motor[4]={0};

 	postureVal[0] = -qAngular[x].PID.output +qAngular[y].PID.output + qAngular[z].PID.output ;
	postureVal[1] = +qAngular[x].PID.output +qAngular[y].PID.output - qAngular[z].PID.output ;
	postureVal[2] = +qAngular[x].PID.output -qAngular[y].PID.output + qAngular[z].PID.output ;
	postureVal[3] = -qAngular[x].PID.output -qAngular[y].PID.output - qAngular[z].PID.output ;	
	
	for(u8 i=0;i<4;i++)
	{
    VAL_LIMIT(postureVal[i], -1000,1000 );
	}
	
	
	curve[0] = (0.55f + 0.45f *abs(postureVal[0])/1000.0f) *postureVal[0] ;    
	curve[1] = (0.55f + 0.45f *abs(postureVal[1])/1000.0f) *postureVal[1] ;
	curve[2] = (0.55f + 0.45f *abs(postureVal[2])/1000.0f) *postureVal[2] ;
	curve[3] = (0.55f + 0.45f *abs(postureVal[3])/1000.0f) *postureVal[3] ;
	
	motor[0] = Rc.g_ThrVAL + Rc.g_Weight[thr] *curve[0] ;//油门越大姿态输出越大
	motor[1] = Rc.g_ThrVAL + Rc.g_Weight[thr] *curve[1] ;
	motor[2] = Rc.g_ThrVAL + Rc.g_Weight[thr] *curve[2] ;
	motor[3] = Rc.g_ThrVAL + Rc.g_Weight[thr] *curve[3] ;



		for(u8 i=0;i<4;i++)
	{
    VAL_LIMIT(motor[i], 0,1500 );
	}
	
//	for(u8 i=0;i<4;i++)
//	{
//		motor[i] = constrain_uint16(postureVal[i],0,1500 );
//	}
//	if(Rc.qudrotorState==lock)
//	{
//		for(u8 i=0;i<4;i++)
//		{
//			motor[i] = 0;
//		}
//		for(u8 i=0;i<3;i++)
//		{
//		 //未解锁时去积分效应
//		 qAngle[i].PID.err=0;
//		 qAngular[i].PID.err=0;
//		}
//	}

	debug.aux6=Rc.qudrotorState;
	
	if(Rc.g_CH[ksr]!=1)
	{

		for(u8 i=0;i<3;i++)
		{
		 //未解锁时去积分效应
		 qAngle[i].PID.err=0;
		 qAngular[i].PID.err=0;
		}
		 for(u8 i=0;i<4;i++)
		{
			motor[i] = 0;
		}
	}
	debug.aux1=motor[0];
	debug.aux2=motor[1];
	debug.aux3=motor[2];
	debug.aux4=motor[3];
  Moto.SetPwm(motor);
}

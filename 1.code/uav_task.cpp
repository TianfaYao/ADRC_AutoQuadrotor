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
#include "includes.h"
#include "imu.h"
#include "stdio.h"
#include "dev_clock.h"
#include "dev_mavlink.h"
#include  "drv_w25qxx.h"
#include "cal_vector3.h"
#include "uav_parameter.h"
#include "dev_sd.h"
#include "dev_flash.h"	
#include "uav_rc.h"
#include "dev_gps.h"
#include "uav_control.h"
#include "uav_attitude.h"
#include "dev_us100.h"
#include "uav_parameter_sd.h"
#include "uav_adrc.h"
#include "nav_speed.h"
#include "nav_h.h"
#include "control_v.h"
//#include "ff.h"
//#include "stdlib.h"

#include "AttitudePositionEstimatorEKF.h"
/*
1.机体是北东地坐标系
2.6050是东北天坐标系
3.ak8975是北东地坐标系
*/

#define scaf   9.086/4096

Vector3f UAV;


//==================================
void task_1_ms(void)
{
	

	
}
extern void AngularRote(float dt);
void task_2_ms(void)
{
		//数据传输
   Mavlink.Data_Exchange();

  float dt=Get_Cycle_T(1);
  //传感器数据更新
	 imu.SensorSample();
	 //adrc 内环
   Adrc_.Angular(dt);
		//计算速度 位置
}
void task_5_ms(void)
{
	 float dt=Get_Cycle_T(2);
 //遥控数据管理
	Rc.DataMange(dt);
  //姿态解算
	// imu.Attitude(dt);
	 att_ekf.loop_up();
	 Adrc_.Angle(dt);
	 nav_v.get_a_v(dt);
	 nav_v.east(dt);
	 ctrl.V();
  
	
}

void task_8_ms()
{
 float dt=Get_Cycle_T(0);
	

};
void task_10_ms(void)
{
	 float dt=Get_Cycle_T(3);
  	  //气压计数据跟新
	 if( Ms5611.DataUpdate()){
	  nav_v.get_ms_v(dt);
	 }

	 //罗盘数据更新
	 Ak8975.DataSample();
  //角度控制
		 

	// Control.Angle(dt);
		 
	//位置控制
   Attitude.DISControl(dt);
}
void task_20_ms(void)
{

	//GPS 数据更新
	GPS_Call();
}

void task_50_ms(void)
{
 //超声波数据采样触发
	Us100.SampleTriger(1);
	
}


void TaskLoop(void)
{
		//1ms 任务
	  task_1_ms();

	 if(loop.cnt_2ms>=2)
	{
	 //2ms 任务
	 task_2_ms();
		loop.cnt_2ms=0;
	}
	if(loop.cnt_5ms>=5)
	{
		//5ms 任务
	  //task_5_ms();
		loop.cnt_5ms=0  ;
	}
		if(loop.cnt_8ms>=8)
	{
		//5ms 任务
	  task_8_ms();
		loop.cnt_8ms=0;
	}
	
	if(loop.cnt_10ms>=10)
	{
		//10ms 任务
	  task_10_ms();
		loop.cnt_10ms=0;
	}
  if(loop.cnt_20ms>=20)
	{
		 //20ms 任务
	  task_20_ms();
		loop.cnt_20ms=0;
	}
  if(loop.cnt_50ms>=50)
	{
		 //50ms 任务
	  task_50_ms();
		loop.cnt_50ms=0;
	}

}


int counT=0;
int up=1;
int main(void )
{ 
 	SysTick_Configuration();
  Uav.Init();
  imu.Init();
  //parameter_sd.SaveDataInit();
	while(1)
	{
	//任务轮询	
	 TaskLoop();
		
		if(counT>=10&&up==1)
		{
			//counT--;
		  up=0;
		}
		if(counT<=-10&&up==0)
		{
		// counT++;
			up=1;
		}
		if(up==1)
			  counT++;
			if(up==0)
			  counT--;
	}
}
	
	
	
	
	
	
	
	
	





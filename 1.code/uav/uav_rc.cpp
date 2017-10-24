/*
  ******************************************************************************
  * @file   
  * @author  ytf
  * @version 
  * @date   
  * @brief  
  *    
  *****************************************************************************
*/

#include "uav_rc.h"
#include "dev_debus.h"
#include "math.h"
#include "imu.h"
_RC Rc;
void _RC::DataMange(float dt )
{
 //摇杆量传递给控制量
 g_CH[yaw]=(Debus.remot[ch0]-1024)* 0.7575f; //+-500
 g_CH[thr]=(Debus.remot[ch1]-1024)* 0.7575f;   
 g_CH[rol]=(Debus.remot[ch2]-1024)* 0.7575f;
 g_CH[pit]=(Debus.remot[ch3]-1024)* 0.7575f;
 g_CH[ksr]=(Debus.remot[ch4]);
 g_CH[ksl]=(Debus.remot[ch5]);
	
//油门定义
 Thr();
//HoveringFlight();
//解锁
 Deblock(dt);
	//模式
 Mode();
}

void _RC::Deblock(float dt)
{
 static u16 unlockcnt=0;
		if(Debus.remot[ch1]==364)  	//内八解锁
		{
			if(Debus.remot[ch0]==1684)
				{
						if(Debus.remot[ch2]==364)
							{
							 if(Debus.remot[ch3]==364)
								 {
									 //解锁
								 qudrotorState=unlock;

							   }
						  }
			  }
		}
//反锁
		if( qudrotorState==lock)
		{
			 if(Debus.remot[ch1]==364)
				{
					if(abs(Debus.remot[ch0]-1024)<=3)
					unlockcnt+=1000 *dt;
				}		
				if(unlockcnt>=1000)//1S
				{
					qudrotorState = lock;
					unlockcnt=0;
				}
		}
}


//油门控制
void _RC::Thr(void)
{
	
  g_ThrVAL=( g_CH[thr]+500)*0.8; //0-800
  g_Weight[thr]=g_ThrVAL/800.0;
	
	g_ThrVAL=1.15*g_ThrVAL;
	
  VAL_LIMIT(g_ThrVAL,0,800);
	
	if(Debus.remot[ch1]>400)
	{
	  g_Thr_Low=0;
	}else
	{
	 g_Thr_Low=1;
	}
//后面添加定高控制
}
//模式控制
void _RC::Mode(void)
{
 if(qudrotorState==unlock)
 {
 		if(g_CH[ksl]==1 )
		{
			g_MODE= 0; //裸飞
		}
		else if(  g_CH[ksl]==3 )
		{
			g_MODE = 1; //气压
		}
		else
		{

		}
	}
	else
	{
	g_MODE = 0;
		
	}
}
//姿态期望控制 脱控悬停
void _RC::HoveringFlight(void)
{
	 float x_kd=0.125 ;
	 float y_kd=0.125;
	 float z_kd;
	
	//加速度直接反馈到输出，抑制飞机四处游荡
	if(abs(g_CH[rol])<30)
	{
	//rol的大小改变飞机沿y轴飞行
		if(imu.ea.y>0)			
			g_CH[rol]=30+y_kd*imu.ea.y;
		
	 if(imu.ea.y<=0)
			g_CH[rol]=-30+y_kd*imu.ea.y;
		
	}

	if(abs(g_CH[pit])<30)
	{
		//pit的大小改变飞机沿x轴飞行	
	 if(imu.ea.x>0)	
	 g_CH[pit]=30+x_kd*imu.ea.x;
	 
	 if(imu.ea.x<=0)	
	 g_CH[pit]=30+x_kd*imu.ea.x;
	
	}
	
}


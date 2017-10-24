#include "imu.h"
#include "uav_filter.h"
#include "uav_rc.h"
#include "imu_dcm.h"
#include "imu_gradient.h"
#include "dev_ak8975.h"

#define     USEDCM
//#define     USEGD
//#define     USEEKF

#define accVelScale      9.80665f / ACC_1G / 10000.0f;
#define angle_z_offfset  15
IMU imu;

extern void  EKF(Vector3f gyr,Vector3f acc,Vector3f mag,uint8_t update_vect[3]	,float dt );
//姿态解算初始化
void IMU::Init(void)
{
  	//加速度二阶低通滤波器系数计算
	LPF_2nd_Factor_Cal(IMU_LOOP_TIME * 1e-6, ACC_LPF_CUT, &Acc_lpf_2nd);
	AccSpeedfactor=LPF_1st_Factor_Cal(IMU_LOOP_TIME * 1e-6,2);
	//互补滤波器系数计算
	factor = CF_Factor_Cal(IMU_LOOP_TIME * 1e-6, GYRO_CF_TAU);	
}

void IMU::SensorSample(void)
{
	//陀螺仪加速度计数据采样
  Mpu6050.DataSample();
	
	//加速度二阶低通滤波
	Acc_lpf=filter.LPF_2nd(&Acc_lpf_2nd,Mpu6050.acc );
	//计算实际测量的加速度和重力加速度的比值
	accRatio = Acc_lpf.length_squared() * 100 / (ACC_1G * ACC_1G);	

}



void IMU::Attitude( float dt)
{
	//DCM 更新姿态
	#ifdef USEDCM
  dcm.DirectionConsineMatrix(Mpu6050.gyrdeg,Acc_lpf,accRatio,dt );
	#endif
	
	#ifdef USEGD
	static u8 initcount=0;
	initcount++;
	if(initcount<10)
	MadgwickAHRSinit(Acc_lpf.x,Acc_lpf.y,Acc_lpf.z,  //加快开机快速到位
		               Ak8975.mag.x,	Ak8975.mag.y,	Ak8975.mag.z);
	else
		initcount=12;

//  MadgwickAHRSupdate(Mpu6050.gyrdeg.x,Mpu6050.gyrdeg.y,Mpu6050.gyrdeg.z,
//  Acc_lpf.x,Acc_lpf.y,Acc_lpf.z,
//	Ak8975.mag.x,	Ak8975.mag.y,	Ak8975.mag.z,dt);	
	  MadgwickAHRSupdate(Mpu6050.gyrdeg.x,Mpu6050.gyrdeg.y,Mpu6050.gyrdeg.z,
  Acc_lpf.x,Acc_lpf.y,Acc_lpf.z,
	0,	0,	0,dt);	
  Quaternion_to_euler(&angle.x,&angle.y,&angle.z);
	#endif 
		
	#ifdef USEEKF  
	uint8_t update_vect[3];
	update_vect[0]=1;
	update_vect[1]=1;
	update_vect[2]=Ak8975.update;
	//update_vect[2]=0;
  EKF(Mpu6050.gyrdeg,Acc_lpf,Ak8975.mag,update_vect, dt );
	Ak8975.update=0;
	#endif 
	//去除罗盘自身偏差
	imu.angle.z=imu.angle.z-angle_z_offfset;
 	
	
}





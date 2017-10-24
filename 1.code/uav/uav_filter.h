#ifndef UAV_FILTER_H
#define UAV_FILTER_H
#include "includes.h"

	
class Filter
{
	
public:

	Filter(){
	}
	
	struct LPF2ndData_t{
		float b0;
		float a1;
		float a2;
		Vector3f preout;
		Vector3f lastout;
	};

	
	//一阶低通滤波器系数计算
	float LPF_1st_Factor_Cal(float deltaT, float Fcut);
	
	//二阶低通滤波器系数计算
	void LPF_2nd_Factor_Cal(float deltaT, float Fcut, LPF2ndData_t* lpf_data);
	
	//互补滤波器系数计算
	float CF_Factor_Cal(float deltaT, float tau);
	
	//一阶低通滤波器
	Vector3f LPF_1st(Vector3f oldData, Vector3f newData, float lpf_factor);
	
	//二阶低通滤波器
	Vector3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData);

	
	//一阶互补滤波器
	Vector3f CF_1st(Vector3f gyroData, Vector3f accData, float cf_factor);
	
	//滑动窗口滤波
void MoveWindow(Vector3i GyroInput,Vector3i *GyroOutput,Vector3i AccInput,Vector3i *AccOutput,u8 num);
// 
 void  Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out);
 //滑动均值
 float MeanValue(float fifin  );

};


extern Filter  filter;
#endif
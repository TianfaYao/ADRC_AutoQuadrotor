#include "uav_filter.h"

Filter  filter;
/*----------------------一阶低通滤波器系数计算-------------------------*/
float Filter::LPF_1st_Factor_Cal(float deltaT, float Fcut)
{
	return deltaT / (deltaT + 1 / (2 * M_PI * Fcut));
}

/*----------------------一阶低通滤波器------------------------*/
Vector3f Filter::LPF_1st(Vector3f oldData, Vector3f newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;
}

/*----------------------二阶低通滤波器系数计算-------------------------*/
void Filter::LPF_2nd_Factor_Cal(float deltaT, float Fcut, LPF2ndData_t* lpf_data)
{
	float a = 1 / (2 * M_PI * Fcut * deltaT);
	lpf_data->b0 = 1 / (a*a + 3*a + 1);
	lpf_data->a1 = (2*a*a + 3*a) / (a*a + 3*a + 1);
	lpf_data->a2 = (a*a) / (a*a + 3*a + 1);
}

/*----------------------二阶低通滤波器------------------------*/
Vector3f Filter::LPF_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData)
{
	Vector3f lpf_2nd_data;
	
	lpf_2nd_data = newData * lpf_2nd->b0 + lpf_2nd->lastout * lpf_2nd->a1 - lpf_2nd->preout * lpf_2nd->a2;
	lpf_2nd->preout = lpf_2nd->lastout;
	lpf_2nd->lastout = lpf_2nd_data;
	
	return lpf_2nd_data;
}



/*----------------------互补滤波器系数计算-------------------------*/
float Filter::CF_Factor_Cal(float deltaT, float tau)
{
	return tau / (deltaT + tau);
}

/*----------------------一阶互补滤波器-----------------------------*/
Vector3f Filter::CF_1st(Vector3f gyroData, Vector3f accData, float cf_factor)
{ 
	return (gyroData * cf_factor + accData *(1 - cf_factor));	
}


void Filter::MoveWindow(Vector3i GyroInput,Vector3i *GyroOutput,Vector3i AccInput,Vector3i *AccOutput,u8 num)
{
  static u8 FilterNumbers=num;
	static u8 count=0;
	 
	static Vector3l   temp_gyro;
	static Vector3i    gyro[10];
	static Vector3l   temp_acc;
	static Vector3i    acc[10];
	if(++count>FilterNumbers)
	{
	 count=0;
	}
  gyro[count]=GyroInput;
	acc[count]=AccInput;
	for(u8 i=0;i<FilterNumbers;i++)
	{
	 temp_gyro.x+=gyro[i].x;
	 temp_gyro.y+=gyro[i].y;
	 temp_gyro.z+=gyro[i].z;
		
	 temp_acc.x+=acc[i].x;
	 temp_acc.y+=acc[i].y;
	 temp_acc.z+=acc[i].z;		
	}
	(*GyroOutput).x=	(float)temp_gyro.x/(float)FilterNumbers;
	(*GyroOutput).y=	(float)temp_gyro.y/(float)FilterNumbers;
	(*GyroOutput).z=	(float)temp_gyro.z/(float)FilterNumbers;
	
	(*AccOutput).x=	(float)temp_acc.x/(float)FilterNumbers;
	(*AccOutput).y=	(float)temp_acc.y/(float)FilterNumbers;
	(*AccOutput).z=	(float)temp_acc.z/(float)FilterNumbers;
}
	
 void Filter:: Moving_Average(float in,float moavarray[],u16 len ,u16 fil_cnt[2],float *out)
{
	u16 width_num;
	
	width_num = len ;
	
	if( ++fil_cnt[0] > width_num )	
	{
		fil_cnt[0] = 0; //now
		fil_cnt[1] = 1; //old
	}
	else
	{
		fil_cnt[1] = (fil_cnt[0] == width_num)? 0 : (fil_cnt[0] + 1);
	}
	
	moavarray[ fil_cnt[0] ] = in;
	*out += ( in - ( moavarray[ fil_cnt[1] ]  ) )/(float)( width_num ) ;
	
}



float Filter::MeanValue(float fifin  )
{
 static float fiftem[10]={0};
 float  fifRET=0;
 static u8 fifcount=0;
 fiftem[ fifcount++]=fifin;
 if(fifcount>=10) fifcount=0;//每次更新一个减少数组移动

 
	for(u8 i=0;i<10;i++)
 {
   fifRET+= fiftem[i];
 }
 fifRET=fifRET/10.0;
	return fifRET;
}




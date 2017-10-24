#include "dev_ak8975.h"
#include "uav_parameter.h"
#include "uav_filter.h"
/*
//北东地
                y
								|
                |
x<______________

*/
_AK8975 Ak8975;
//采样触发
void _AK8975::SampleTriger(void)
{
  SingleWrite(AK8975_ADDRESS,AK8975_CNTL,0x01);
}
//采样
void _AK8975::DataSample(void)
{
 
	Vector3i Tmag;
	SingleRead(AK8975_ADDRESS,AK8975_HXL,&buffer[0]); 
	SingleRead(AK8975_ADDRESS,AK8975_HXH,&buffer[1]);
	Tmag.y= ((((int16_t)buffer[1]) << 8) | buffer[0]) ;  //罗盘X

	SingleRead(AK8975_ADDRESS,AK8975_HYL,&buffer[2]);
	SingleRead(AK8975_ADDRESS,AK8975_HYH,&buffer[3]);
	Tmag.x = ((((int16_t)buffer[3]) << 8) | buffer[2]) ;  //罗盘Y

	SingleRead(AK8975_ADDRESS,AK8975_HZL,&buffer[4]);
	SingleRead(AK8975_ADDRESS,AK8975_HZH,&buffer[5]);
	Tmag.z = -((((int16_t)buffer[5]) << 8) | buffer[4]) ;  //罗盘Z	
	
	magTem.x=(float)Tmag.x;  
	magTem.y=(float)Tmag.y;
	magTem.z=(float)Tmag.z;
	//矫正后
  mag=magTem-magOffset;
	mag=magTem;

	
 // printf("mag %f  %f  %f \n",magTem.x,magTem.y,magTem.z);
 //数据低通
 //罗盘低通滤波系数
 
 	float Ak8975factor=0.02 / (0.02 + 1 / (2 * M_PI * 0.5));  //采样时间是20毫秒5H低通
 	mag=mag*Ak8975factor+lastmag*(1-Ak8975factor);
	lastmag=mag;
//	
//	mag.x=filter.MeanValue(magTem.x);
//	mag.y=filter.MeanValue(magTem.y);
//	mag.z=filter.MeanValue(magTem.z);
	MagCalOffset();
	update=1;
	SampleTriger();//再次采样触发
}

void _AK8975:: MagCalOffset(void)
{

  Vector3f  magMAX(-1000,-1000,-1000), minMAX(1000,1000,1000);//避免将初值变成最值
	static u16  cnt_m=0;
	
	if(g_CallSetMagOffset==1)
	{
		static Vector3f temMag(0,0,0);
		//周围磁场正常
	 if(fabs(mag.x)<400&&fabs(mag.x)<400&&fabs(mag.x)<400)
	 {
		 //寻找最大
		 if((magTem.x-magMAX.x)>0.1)  magMAX.x=magTem.x;
		 if((magTem.y-magMAX.y)>0.1)  magMAX.x=magTem.y;
		 if((magTem.z-magMAX.z)>0.1)  magMAX.x=magTem.z;
		 
		 //寻找最小
		 if((magTem.x-minMAX.x)<0.1)  minMAX.x=magTem.x;
		 if((magTem.y-minMAX.y)<0.1)  minMAX.x=magTem.y;
		 if((magTem.z-minMAX.z)<0.1)  minMAX.x=magTem.z;
	 }
	 cnt_m++;
	 if(cnt_m==2000)//这儿可以加陀螺仪旋转积分判断矫正过程中是否每个轴选装360，开机飞行前
	 {
		//椭圆矫正
		temMag=(magMAX+minMAX)*0.5f;
		 //存入零飘数据
		Pamameter.SaveMagOfferset(temMag);
		//Delay_ms(2);
		 //更新零飘数据
		Pamameter.ReadMagOfferset(); 
		 //清除残留数据保证下次矫正正常进行
	  magMAX(-1000,-1000,-1000), minMAX(1000,1000,1000),temMag(0,0,0);
	  cnt_m=0;
		g_CallSetMagOffset=0; 
	 }
	
	
	}
	
	
}





/*
  ******************************************************************************
  * @file   
  * @author  TianfaYao
  * @version 
  * @date   
  * @brief  
  *    
  *****************************************************************************
*/

#include "includes.h"
#include "uav_parameter.h"
#include "string.h"
#include "dev_mpu6050.h"
#include "cal_vector3.h"
#include "stdio.h"
#include  "drv_w25qxx.h"
#include "dev_flash.h"
#include "uav_pid.h"
#include "uav_control.h"
#include "uav_config.h"
_PARAMETER Pamameter;

/*
每次较矫正结束延时一会就读取一次数据 开机的时候读取一次数据
*/

void _PARAMETER::Configuration(void )
{  
 //地址初始化
	FlashAdrr[acc]=0;                  //地址的移动是字节为单位
	FlashAdrr[gyr]=FlashAdrr[gyr-1]+12;
	FlashAdrr[mag]=FlashAdrr[mag-1]+12;
  FlashAdrr[anglepid]=FlashAdrr[anglepid-1]+12*3;
  FlashAdrr[angularpid]=FlashAdrr[angularpid-1]+12*3;
	
  FlashAdrr[ul_dis_spepid]=FlashAdrr[ul_dis_spepid-1]+12*2;
	FlashAdrr[ms_dis_spepid]=FlashAdrr[ms_dis_spepid-1]+12*2;

	//传感器零飘初始化
	ReadGyrOfferset();
	Delay_ms(1);
	ReadAccOfferset();
	Delay_ms(1);
	ReadMagOfferset();
	Delay_ms(1);
	//pid 参数初始化
	ReadAnglePID();
	Delay_ms(1);
	ReadAngularPID();
	Delay_ms(1);
	Read_UL_DIS_SPEPID();
	Delay_ms(1);
	Read_MS_DIS_SPEPID();
}

//加速度计零飘缓存
void _PARAMETER::SaveAccOfferset(Vector3f soure )
{
	float acctem[3];
	printf("d %d\n",sizeof(acctem));
	acctem[0]=soure.x;acctem[1]=soure.y;acctem[2]=soure.z;
	
  flash25q32.SPI_Flash_Write_f(FlashAdrr[acc],acctem,sizeof(acctem));
	
	printf("加计存入数据成功.....\n");
}
//加速度计零飘读取

void _PARAMETER::ReadAccOfferset(void )
{
  float acctem[3];
	flash25q32.SPI_Flash_Read_f(FlashAdrr[acc],acctem,sizeof(acctem));
	Mpu6050.accOffset.x=acctem[0];Mpu6050.accOffset.y=acctem[1];Mpu6050.accOffset.z=acctem[2];
	printf("acc x %f\n",Mpu6050.accOffset.x);
	printf("acc y %f\n",Mpu6050.accOffset.y);
	printf("acc z %f\n",Mpu6050.accOffset.z);
	printf("加计读取数据成功.....\n");
}



//陀螺仪零飘缓存
void  _PARAMETER::SaveGyrOfferset(Vector3f soure)
{
	float gyrtem[3];
	gyrtem[0]=soure.x;gyrtem[1]=soure.y;gyrtem[2]=soure.z;
	
  flash25q32.SPI_Flash_Write_f(FlashAdrr[gyr],gyrtem,sizeof(gyrtem));
	
	printf("陀螺仪存入数据成功.....\n");
}
//陀螺仪零飘读取
void _PARAMETER::ReadGyrOfferset(void )
{
  float gyrtem[3];
	flash25q32.SPI_Flash_Read_f(FlashAdrr[gyr],gyrtem,sizeof(gyrtem));
	
	Mpu6050.gyrOffset.x=gyrtem[0];Mpu6050.gyrOffset.y=gyrtem[1];Mpu6050.gyrOffset.z=gyrtem[2];
	printf("gyr x %f\n",Mpu6050.gyrOffset.x);
	printf("gyr y %f\n",Mpu6050.gyrOffset.y);
	printf("gyr z %f\n",Mpu6050.gyrOffset.z);
	printf("陀螺仪读取数据成功.....\n");
}

//罗盘零飘存储
void  _PARAMETER::SaveMagOfferset(Vector3f soure)
{
 float magtem[3];
	magtem[0]=soure.x;magtem[1]=soure.y;magtem[2]=soure.z;
  flash25q32.SPI_Flash_Write_f(FlashAdrr[mag],magtem,sizeof(magtem));
	printf("罗盘数据存入成功....\n");
}

//罗盘零飘读取
void _PARAMETER::ReadMagOfferset(void )
{
  float magtem[3];
	flash25q32.SPI_Flash_Read_f(FlashAdrr[mag],magtem,sizeof(magtem));
	
	Ak8975.magOffset.x=magtem[0];Ak8975.magOffset.y=magtem[1];Ak8975.magOffset.z=magtem[2];
	
	printf("mag x %f\n",Ak8975.magOffset.x);
	printf("mag y %f\n",Ak8975.magOffset.y);
	printf("mag z %f\n",Ak8975.magOffset.z);
	printf("罗盘零飘缓存数据读取成功.....\n");
}
//存下角度环pid 参数
void _PARAMETER::SaveAnglePID(void)
{
	 float magtem[9];
	 //x
	 magtem[0]=qAngle[x].PID.Kp; magtem[1]=qAngle[x].PID.Ki; magtem[2]=qAngle[x].PID.Kd;
	 //y
	 magtem[3]=qAngle[y].PID.Kp; magtem[4]=qAngle[y].PID.Ki; magtem[5]=qAngle[y].PID.Kd;
	 //z
	 magtem[6]=qAngle[z].PID.Kp; magtem[7]=qAngle[z].PID.Ki; magtem[8]=qAngle[z].PID.Kd;
	
   flash25q32.SPI_Flash_Write_f(FlashAdrr[anglepid],magtem,sizeof(magtem));
	
	 printf("角度环pid 存入成功...\n");

}
//存下角速度环参数
void _PARAMETER::SaveAngularPID(void)
{
  float magtem[9];
	 //x
	 magtem[0]=qAngular[x].PID.Kp; magtem[1]=qAngular[x].PID.Ki; magtem[2]=qAngular[x].PID.Kd;
	 //y
	 magtem[3]=qAngular[y].PID.Kp; magtem[4]=qAngular[y].PID.Ki; magtem[5]=qAngular[y].PID.Kd;
	 //z
	 magtem[6]=qAngular[z].PID.Kp; magtem[7]=qAngular[z].PID.Ki; magtem[8]=qAngular[z].PID.Kd;
	
   flash25q32.SPI_Flash_Write_f(FlashAdrr[angularpid],magtem,sizeof(magtem));
	
	 printf("角度环pid 存入成功...\n");
}

void _PARAMETER::ReadAnglePID(void)
{
	float magtem[9];
	flash25q32.SPI_Flash_Read_f(FlashAdrr[anglepid],magtem,sizeof(magtem));
	//x
   qAngle[x].PID.Kp=magtem[0]; qAngle[x].PID.Ki=magtem[1]; qAngle[x].PID.Kd=magtem[2];
	 //y
	 qAngle[y].PID.Kp=magtem[3];qAngle[y].PID.Ki= magtem[4]; qAngle[y].PID.Kd=magtem[5];
	 //z
	 qAngle[z].PID.Kp=magtem[6]; qAngle[z].PID.Ki=magtem[7]; qAngle[z].PID.Kd=magtem[8];
  
	 printf("角度环参数读取成功....\n");
}
	
void _PARAMETER::ReadAngularPID(void)
{

float magtem[9];
	flash25q32.SPI_Flash_Read_f(FlashAdrr[angularpid],magtem,sizeof(magtem));
	
	//x
   qAngular[x].PID.Kp=magtem[0]; qAngular[x].PID.Ki=magtem[1]; qAngular[x].PID.Kd=magtem[2];
	 //y
	 qAngular[y].PID.Kp=magtem[3];qAngular[y].PID.Ki= magtem[4]; qAngular[y].PID.Kd=magtem[5];
	 //z
	 qAngular[z].PID.Kp=magtem[6]; qAngular[z].PID.Ki=magtem[7]; qAngular[z].PID.Kd=magtem[8];
	
	printf("角速度环参数读取成功.....\n");
	
}

//
//存下超声波位移速度pid参数
void _PARAMETER::Save_UL_DIS_SPEPID(void)
{
  float magtem[6];
	 //超声波
	 magtem[0]=z_dis[ul].PID.Kp; magtem[1]=z_dis[ul].PID.Ki; magtem[2]=z_dis[ul].PID.Kd;
	 //
	 magtem[3]=z_spe[ul].PID.Kp; magtem[4]=z_spe[ul].PID.Ki; magtem[5]=z_spe[ul].PID.Kd;
	
   flash25q32.SPI_Flash_Write_f(FlashAdrr[ul_dis_spepid],magtem,sizeof(magtem));
	
	 printf("位置速度pid 存入成功...\n");
}
//读取超声波速度pid参数
void _PARAMETER::Read_UL_DIS_SPEPID(void)
{
  float magtem[6];
	
	 flash25q32.SPI_Flash_Read_f(FlashAdrr[ul_dis_spepid],magtem,sizeof(magtem));
		 //超声波
	 z_dis[ul].PID.Kp= magtem[0]; z_dis[ul].PID.Ki=magtem[1]; z_dis[ul].PID.Kd=magtem[2];
	 //
	 z_spe[ul].PID.Kp=magtem[3];z_spe[ul].PID.Ki= magtem[4]; z_spe[ul].PID.Kd=magtem[5];
 
	printf("位置速度pid 读取成功...\n");

}

//存下气压计位移速度pid参数
void _PARAMETER::Save_MS_DIS_SPEPID(void)
{
  float magtem[6];
	
	 //气压计
	 magtem[0]=z_dis[ms].PID.Kp; magtem[1]=z_dis[ms].PID.Ki; magtem[2]=z_dis[ms].PID.Kd;
	//
	 magtem[3]=z_spe[ms].PID.Kp; magtem[4]=z_spe[ms].PID.Ki; magtem[5]=z_spe[ms].PID.Kd;
	
   flash25q32.SPI_Flash_Write_f(FlashAdrr[ms_dis_spepid],magtem,sizeof(magtem));
	
	 printf("位置速度pid 存入成功...\n");
}
//读取气压计位移速度pid参数
void _PARAMETER::Read_MS_DIS_SPEPID(void)
{
  float magtem[6];
	
	 flash25q32.SPI_Flash_Read_f(FlashAdrr[ms_dis_spepid],magtem,sizeof(magtem));
	 //气压计
	z_dis[ms].PID.Kp= magtem[0];z_dis[ms].PID.Ki= magtem[1];z_dis[ms].PID.Kd= magtem[2];
	//
	z_spe[ms].PID.Kp= magtem[3];z_spe[ms].PID.Ki= magtem[4];z_spe[ms].PID.Kd= magtem[5];
	 
	printf("位置速度pid 读取成功...\n");
}
//参数复位
void _PARAMETER::Reset(void)
{
   //角度
		//x
   qAngle[x].PID.Kp=0.65; qAngle[x].PID.Ki=0.045; qAngle[x].PID.Kd=0;
	 //y
	 qAngle[y].PID.Kp=0.65;qAngle[y].PID.Ki= 0.045; qAngle[y].PID.Kd=0;
	 //z
	 qAngle[z].PID.Kp=0.68; qAngle[z].PID.Ki=0.05; qAngle[z].PID.Kd=0;
	//角速度
	//x
   qAngular[x].PID.Kp=0.55; qAngular[x].PID.Ki=0; qAngular[x].PID.Kd=0.7;
	 //y
	 qAngular[y].PID.Kp=0.55;qAngular[y].PID.Ki= 0; qAngular[y].PID.Kd=0.7;
	 //z
	 qAngular[z].PID.Kp=0.55; qAngular[z].PID.Ki=0; qAngular[z].PID.Kd=0.7;
   //超声定高
	z_dis[ul].PID.Kp= 0.08; z_dis[ul].PID.Ki=0.001; z_dis[ul].PID.Kd=0;
	  //
	z_spe[ul].PID.Kp=0.9;z_spe[ul].PID.Ki= 0; z_spe[ul].PID.Kd=6;
    //气压定高
	z_dis[ms].PID.Kp=0;z_dis[ms].PID.Ki= 0;z_dis[ms].PID.Kd= 0;
	//
	z_spe[ms].PID.Kp=0;z_spe[ms].PID.Ki= 0;z_spe[ms].PID.Kd= 0;	
}








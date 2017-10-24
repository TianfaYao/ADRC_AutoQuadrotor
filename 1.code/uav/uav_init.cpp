/*
  ******************************************************************************
  * @file   
  * @author  ytf
  * @version 
  * @date   
  * @brief  初始化函数
  *    
  *****************************************************************************
*/

#include "uav_init.h"
#include "drv_w25qxx.h"
#include "uav_parameter.h"
#include "dev_moto.h"
#include "uav_control.h"
#include "uav_rc.h"
#include "uav_attitude.h"
#include "uav_adrc.h"
#include "nav_h.h"

_UAV Uav;
void _UAV::Init(void)
{
	//时钟初始化
	Rcc.Configuration();
	//端口初始化
  Port.Configuration();
	//串口初始化
	Usart.Configuration();
	//中断初始化
	Nvic.Configuration();
 //DMA初始化
	Dma.Configuration();
	//LED初始化
  Led.Configuration();
	//Flash初始化
	flash25q32.Init_();
	//参数初始化
  Pamameter.Configuration();
	//控制参数初始化
	Control.Init();
	//ADRC
	Adrc_.Init();
	//
	nav_h.Init();
	//位置控制初始化
	Attitude.Init();
	//电机初始化
	Delay_ms(100);
	Moto.MotoInit();
	Delay_ms(100);
	//传感器初始化
	Mpu6050.Configuration(20);
	//气压计初始化
	Ms5611.Configuration();
	
	
//	while(Rc.g_CH[ksr]!=1&&Rc.g_CH[ksr]!=2&&Rc.g_CH[ksr]!=3);
//	while(Rc.g_CH[ksl]!=1&&Rc.g_CH[ksl]!=2&&Rc.g_CH[ksl]!=3);
	
};
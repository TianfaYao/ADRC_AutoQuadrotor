#include "dev_port.h"

/*
  ******************************************************************************
  * @file    
  * @author  姚天发
  * @version 
  * @date    
  * @brief   飞控端口初始化
  *    
  *****************************************************************************
*/
 _PORT  port;
void _PORT::Configuration(void)
{
//串口2
  GPIO_InitTypeDef  GPIO_InitStructure;	
	//LED
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1\
	                               |GPIO_Pin_2| GPIO_Pin_3 ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP ;
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType  = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL ;
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
//PD8 GPS  
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP ;
	//GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
//PD9 GPS
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType  = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL ;
	//GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
//配置PC6 作为超生波数据接收RX
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6; 
  GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
//配置PC7 作为超生波数据接收TX
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_7 ; 
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType  = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL ;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	 
//I2C
  GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL | I2C_Pin_SDA; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DEV_GPIO_I2C, &GPIO_InitStructure);		

//MOTO
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_9 | GPIO_Pin_11 \
	                                 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP ;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 

  GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); 
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
//debus
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA,&GPIO_InitStructure);
	//
}

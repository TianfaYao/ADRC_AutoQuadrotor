#include "dev_rcc.h"

/*
  ******************************************************************************
  * @file   
  * @author  ytf
  * @version 
  * @date   
  * @brief  飞控时钟初始化
  *    
  *****************************************************************************
*/
 _RCC   Rcc;
void _RCC::Configuration(void)
{

	//LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	//串口2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	//串口3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	//串口6
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);   //开启USART6时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	//I2C
	RCC_AHB1PeriphClockCmd(DEV_RCC_I2C , ENABLE );
	//MOTO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  //FLASH
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	//debus
	 RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA| RCC_AHB1Periph_DMA2,ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	
}

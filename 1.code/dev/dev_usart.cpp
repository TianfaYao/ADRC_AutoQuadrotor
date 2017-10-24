/*
  ******************************************************************************
  * @file   
  * @author ytf
  * @version 
  * @date   
  * @brief    飞控串口初始化
  *    
  *****************************************************************************
*/

#include "dev_usart.h"
#include "dev_mavlink.h"
#include "stdio.h"
#include "dev_us100.h"
#include "dev_gps.h"

 _USART Usart;
void _USART::Configuration(void)
{
	//USART2 数据传输
  USART_InitTypeDef USART_InitStructure;
	USART_DeInit(USART2);
  USART_InitStructure.USART_BaudRate     = 500000;
	USART_InitStructure.USART_WordLength   = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits     = USART_StopBits_1;  
	USART_InitStructure.USART_Parity       = USART_Parity_No;    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode         = USART_Mode_Tx | USART_Mode_Rx;  
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE); 
	//GPS 
	USART_DeInit(USART3);
  USART_InitStructure.USART_BaudRate     = 9600;
	USART_InitStructure.USART_WordLength   = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits     = USART_StopBits_1;  
	USART_InitStructure.USART_Parity       = USART_Parity_No;    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode         = USART_Mode_Tx | USART_Mode_Rx;  
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE); 
	//USART6
	USART_DeInit(USART6);
  USART_InitStructure.USART_BaudRate     = 9600;
	USART_InitStructure.USART_WordLength   = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits     = USART_StopBits_1;  
	USART_InitStructure.USART_Parity       = USART_Parity_No;    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	USART_InitStructure.USART_Mode         = USART_Mode_Tx | USART_Mode_Rx;  
	USART_Init(USART6, &USART_InitStructure);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART6, ENABLE);
	//debus
  USART_DeInit(USART1);
  USART_InitStructure.USART_BaudRate = 100000;          //SBUS 100K baudrate
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_Mode = USART_Mode_Rx;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1,&USART_InitStructure);
  USART_Cmd(USART1,ENABLE);
}






//重定向printf


extern "C"	int fputc(int ch, FILE *f)
	{
		while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE));
		USART_SendData(USART2,ch);
		return ch;
}






u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0;
static u8 RxBuffer[50];
static u8 RxState = 0;
void _USART::Usart2Call(void)
{
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)//??!????if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)???
    {
        USART_ReceiveData(USART2);
    }
		
	//发送中断
	if((USART2->SR & (1<<7))&&(USART2->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART2,USART_IT_TXE)!=RESET)
	{
		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
			//USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
		}
	}
	//接收中断 (接收寄存器非空) 
	if(USART2->SR & (1<<5))//if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART2->DR;
		static u8 _data_len = 0,_data_cnt = 0;
		if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Mavlink.Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	}
}
void _USART::Uart2_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
	for(u8 i=0;i<data_num;i++)
		TxBuffer[count++] = *(DataToSend+i);
	if(!(USART2->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
}
//串口3回调函数 GPS
void _USART::Usart3Call(void)
{
	u8 rxtemp=0;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		rxtemp=(u8)USART_ReceiveData(USART3);
    GPS_DataCacheCall(rxtemp);
	 USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}


//串口6 回调函数（超声波）
void _USART::Usart6Call(void)
{
if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)  
{
 Us100.disCall();
 USART_ClearITPendingBit(USART6, USART_IT_RXNE);
}
}


#ifndef DEV_USART_H
#define DEV_USART_H
#include "stm32f4xx.h"
class _USART
{
	 private:
	 public:	
	 static void Configuration(void);
	 void Usart2Call(void);
	 void Usart3Call(void);
	 void Usart6Call(void);
	 void Uart2_Put_Buf(unsigned char *DataToSend , u8 data_num);
	
};
extern _USART Usart;

#endif
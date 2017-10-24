#include "dev_debus.h"
 /*
  ******************************************************************************
  * @file   
  * @author  ytf
  * @version 
  * @date   
  * @brief  
  *    
  *****************************************************************************
*/
 
 _DEBUS Debus;
 
void _DEBUS::RemotCall(void)
{
	//后面加入状态检查函数
remot[ch2] = (Dma.buffer[0]| (Dma.buffer[1] << 8)) & 0x07ff;//右边左右
remot[ch3] = ((Dma.buffer[1] >> 3) | (Dma.buffer[2] << 5)) & 0x07ff;//右边上下
remot[ch0] = ((Dma.buffer[2] >> 6) | (Dma.buffer[3] << 2) | (Dma.buffer[4] << 10)) & 0x07ff;//左边左右
remot[ch1] = ((Dma.buffer[4] >> 1) | (Dma.buffer[5] << 7)) & 0x07ff;//左边上下
remot[ch5] = ((Dma.buffer[5] >> 4)& 0x000C) >> 2;//radio_switch_left
remot[ch4] = ((Dma.buffer[5] >> 4)& 0x0003);//radio_switch_right
}

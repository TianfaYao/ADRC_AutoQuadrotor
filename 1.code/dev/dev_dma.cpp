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

#include "dev_dma.h"
#include "dev_debus.h"
 _DMA Dma;
void _DMA::Configuration(void)
{
 DMA_InitTypeDef   DMA_InitStructure;
 USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
 DMA_DeInit(DMA2_Stream2);
 DMA_InitStructure.DMA_Channel= DMA_Channel_4;
 DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
 DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;
 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
 DMA_InitStructure.DMA_BufferSize = 18;
 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
 DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
 DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
 DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
 DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
 DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
 DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
 DMA_Init(DMA2_Stream2,&DMA_InitStructure);
 DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
 DMA_Cmd(DMA2_Stream2,ENABLE);	
}
//写成回调函数
extern "C" void DMA2_Stream2_IRQHandler(void)
{  
    if( DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
    {   

			    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
      	  DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2); 
          //遥控器数据处理回调函数
			   Debus.RemotCall();

		}
	}

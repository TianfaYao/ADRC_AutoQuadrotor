#ifndef DEV_DMA_H
#define DEV_DMA_H
#include "stm32f4xx.h"

class _DMA
{
	public:
		u8 buffer[18];
		void Configuration(void);
	private:

};

extern _DMA Dma;

#endif
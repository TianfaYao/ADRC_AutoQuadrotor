#ifndef DEV_DEBUS_H
#define DEV_DEBUS_H
#include "stm32f4xx.h"
#include "dev_dma.h"
enum{
ch0=0,
ch1,
ch2,
ch3,
ch4,
ch5,
};
class _DEBUS
{
	public:
	int16_t remot[6];
  void RemotCall(void);
	private:
};

extern _DEBUS Debus;


#endif

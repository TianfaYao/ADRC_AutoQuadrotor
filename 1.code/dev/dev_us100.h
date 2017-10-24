#ifndef DEV_US100_H
#define DEV_US100_H
#include "stm32f4xx.h"


class _US100
{
	
	public:
	u8 rxTem;
	
	float  dis;
	float  disFilt;
	float lastdis;
	u8  sendFLag;
  void SampleTriger(float dt);
	void disCall(void);
};

extern  _US100 Us100;

#endif
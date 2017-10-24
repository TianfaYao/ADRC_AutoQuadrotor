#ifndef DEV_MOTO_H
#define DEV_MOTO_H
#include "stm32f4xx.h"
class _MOTO
{
	public :
	void MotoInit(void);
	void SetPwm(u16 moto[4]);
	private:
	static void MotoBaseInit(TIM_TypeDef* TIMx,u8 CH );
	
};

extern _MOTO Moto;
#endif
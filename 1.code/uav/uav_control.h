#ifndef UAV_CONTROL_H
#define UAV_CONTROL_H
#include "stm32f4xx.h"
enum {
	x=0,
	y,
	z,

};
class _CONTROL
{
	public:
		void Init(void);
		void Angular(float dt);
	  void Angle(float dt);
	  void DecompositionOutput(void); 
	private:
};

extern _CONTROL Control;


#endif



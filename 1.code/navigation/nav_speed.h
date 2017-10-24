#ifndef NAV_SPEED_H
#define NAV_SPEED_H
#include "stm32f4xx.h"

class NAV_V_
{
	private:

   u8   upvm;
	
	public:
	float va;
	 float vm;
	 float vz;
	 float eh;
	void get_ms_v(float dt);
	void get_a_v(float dt);
	void east(float dt);
	void h_east(float dt);
};

extern NAV_V_ nav_v;
#endif

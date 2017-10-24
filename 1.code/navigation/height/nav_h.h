#ifndef NAV_H_H
#define NAV_H_H
#include "stm32f4xx.h"

class NAV_H_
{
	public:
  void Init(void);
	void cal( float ms,float a,u8 &up, float dt);
	float h,v,a;

	private :
   float Q[9];
   float R[4];
	 float I[9];
	 float H[6];
	 float F[9];
	 float Z[2];
	
};


extern NAV_H_ nav_h;

#endif
#ifndef UAV_CONFIG_H
#define UAV_CONFIG_H
#include "stm32f4xx.h"

enum
{
	yaw=0,
	thr,
	rol,
	pit,
	ksr,
	ksl,
};

enum
{
 lock=0,
 unlock,
};

enum 
{
dis_ul_z=1,
dis_ms_z=2,
speed,
	
};

enum 
{
ul,
ms,
};


#endif

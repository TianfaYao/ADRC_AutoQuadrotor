#ifndef IMU_DCM_H
#define IMU_DCM_H
#include "uav_filter.h"


class _DCM
{
	public :
		 void DirectionConsineMatrix(Vector3f gry,Vector3f acc,float accRatio ,float dt);

};

extern _DCM dcm;

#endif
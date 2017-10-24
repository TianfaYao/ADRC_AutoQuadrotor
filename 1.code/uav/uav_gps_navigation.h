#ifndef UAV_GPS_NAVIGATION_H
#define UAV_GPS_NAVIGATION_H
#include "dev_gps.h"

class  _GPS_NAVIGATION
{
	private:
	 float Ratio;//不同经度平面半径
	void  CalcRatio(int32_t lat);
	void  Distance_cm_Bearing(int32_t * lat1, int32_t * lon1, int32_t * lat2, int32_t * lon2, uint32_t * dist, int32_t * bearing);
	public:
		
};
#endif
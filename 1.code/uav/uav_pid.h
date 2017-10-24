#ifndef UAV_PID_H
#define UAV_PID_H
#include "stm32f4xx.h"

class _PID
{
	public:
	 struct {
 float	errWgight;
 float	limitI;
 float Kp,Ki,Kd;
 float offsetKp,offsetKi,offsetKd;
 float err,lasterr,ec,lastec;	
 float err_i;
 float expect;
 float current,output;
		 
}PID;
 
 void Init( const float kp,const float ki, const float kd);
 void GetErr(float current);
 void disErr_i(float dt, float max);
 void Err_i(float dt);
 void Err_d(float dt);
 void Cal(void);
	 
};

extern   _PID 			qAngle[3];
extern   _PID       qAngular[3];
extern   _PID       z_dis[2]; 
extern   _PID       z_spe[2];



#endif
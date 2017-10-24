#ifndef IMU_GRADIENT_H
#define IMU_GRADIENT_H

class _GGRADIENT
{


};




void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,
                       	float mx, float my, float mz,float dt) ;
void  Quaternion_to_euler(float *roll, float *pitch, float *yaw);

void MadgwickAHRSinit(float ax, float ay, float az, float mx, float my, float mz);
#endif
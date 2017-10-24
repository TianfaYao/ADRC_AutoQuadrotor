#include "cal_math.h"


float invSqrt(float x)  
{  
    float xhalf = 0.5f * x;  
    int i = *(int*)&x;          // get bits for floating value  
    i =  0x5f375a86 - (i>>1);    // gives initial guess  
    x = *(float*)&i;            // convert bits back to float  
    x = x * (1.5f - xhalf*x*x); // Newton step  
    return x;  
}  



//角度转弧度
float radians(float deg) {
	return deg * DEG_TO_RAD;
}

//弧度转角度
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

//求平方
float sq(float v) {
	return v*v;
}

//2维向量长度
float pythagorous2(float a, float b) {
	return sqrtf(sq(a)+sq(b));
}

//3维向量长度
float pythagorous3(float a, float b, float c) {
	return sqrtf(sq(a)+sq(b)+sq(c));
}

//4维向量长度
float pythagorous4(float a, float b, float c, float d) {
	return sqrtf(sq(a)+sq(b)+sq(c)+sq(d));
}
//数据死区
 float deathzoom(float x,float zoom)
{
	float t;
	if(x>0)
	{
		t = x - zoom;
		if(t<0)
		{
			t = 0;
		}
	}
	else
	{
		t = x + zoom;
		if(t>0)
		{
			t = 0;
		}
	}
  return (t);
}
//
float To_180_degrees(float x)//角度归到正负180
{
	return (x>180?(x-360):(x<-180?(x+360):x));
}



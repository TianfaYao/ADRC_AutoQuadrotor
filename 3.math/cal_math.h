#ifndef CAL_MATH_H
#define CAL_MATH_H
#include "stm32f4xx.h"

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "cal_vector3.h"
#include "cal_matrix3.h"
#include "cal_quaternion.h"


#define M_PI 3.141592653f
#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f

float safe_asin(float v);

//浮点数限幅
float constrain_float(float amt, float low, float high);

//16位整型数限幅
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);

//16位无符号整型数限幅
uint16_t constrain_uint16(uint16_t amt, uint16_t low, uint16_t high);

//32位整型数限幅
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);

int32_t applyDeadband(int32_t value, int32_t deadband);

//角度转弧度
float radians(float deg);

//弧度转角度
float degrees(float rad);

//求平方
float sq(float v);

//2维向量长度
float pythagorous2(float a, float b);

//3维向量长度
float pythagorous3(float a, float b, float c); 

//4维向量长度
float pythagorous4(float a, float b, float c, float d);
////数据死区 并且连续化
float deathzoom(float x,float zoom);
//
float To_180_degrees(float x);//角度归到正负180
//
float invSqrt(float x);
#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

#endif


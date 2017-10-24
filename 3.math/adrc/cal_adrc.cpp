#include "cal_adrc.h"
#include "fal_initialize.h"
#include "fal.h"
#include "fhan.h"
#include "cal_math.h"
/*
  ******************************************************************************
  * @file     cal_adrc.cpp
  * @author   ytf
  * @version  v1
  * @date     2017/6/28
  * @brief    this file include TD and ESO
  *    
  *****************************************************************************
*/
//跟踪微分器参数初始化
void TrackingDiffInit(Tracking_Diff &td,float r)
{
	td.h=0;
	td.r=0;
	td.v1=0;
	td.v2=0;
  td.r=r;
}
//跟踪微分器实现
void TrackingDiffCal(Tracking_Diff &td,float vt,float dt)
{
	//采样步长
	td.h=dt;  
  float fh=fhan(td.v1-vt,td.v2,td.r,td.h);
	td.v1=td.v1+ td.h*td.v2;
	td.v2=td.v2+ td.h*fh;
}

//扩张状态观测器参数初始化
void ExtendStObsInit(Extend_St_Obs &eso,float r,float c1)
{
 eso.r=r;
 eso.c1=c1;
	
}
void ExtendStObsInit2(Extend_St_Obs2 &eso,float h)
{
 eso.h=h;
	
}
//扩张状态观测器实现
void  ExtendStObsCal(Extend_St_Obs &eso,float v1,float v2,float x,float h)
{
	//采样步长
	eso.h=0.05;
	eso.B1=1.0f/eso.h;
	eso.B2=1.0f/(3.0f*eso.h*eso.h);
	eso.B3=1.0f/(64.0f*eso.h*eso.h*eso.h);
	
  float e=eso.z1-x;
	float fe=fal(e,0.5,eso.h);
	float fe1=fal(e,0.25,eso.h);
	
	eso.z1+=eso.h*(eso.z2-eso.B1*e);
	eso.z2+=eso.h*(eso.z3-eso.B2*fe+eso.u);
	eso.z3+=eso.h*(-eso.B3*fe1);
	
	float e1=v1-eso.z1;
	float e2=v2-eso.z2;
	
	float u0=fhan(e1,eso.c1*e2,eso.r,eso.h);
	eso.u=(u0-eso.z3);
	
}
// 两个跟踪微分器实现线性pid 
void dt2dtalp(float i_r,float i_h,float &i,float o_r,float o_h,float &o,lpid &lp )
{
  static Tracking_Diff td_i,td_o;
	TrackingDiffCal(td_i,i,i_h);
	TrackingDiffCal(td_i,o,i_h);
	
	lp.e1=td_i.v1-td_o.v1;
	lp.e2=td_i.v2-td_o.v2;
	lp.e0=lp.e1;
	lp.u=lp.bo*lp.e0+lp.b1*lp.e1+lp.b2*lp.e2;
}

float eu;
float ez;
float fa;
void ExtendStObsCal2( Extend_St_Obs2 &eso, float i,float u,float B1,float B2,float drt,float eso_b,float eso_h)
{
	eso.h=eso_h;
	eso.B=eso_b;
	float e= eso.z1-i;
	float fe=fal(e,0.5,eso.h); //+-60 e 3000	
	fa=fe;
  eso.z1=eso.z1+eso.h*(eso.z2-B1*e+eso.u);
	eso.z2=eso.z2-eso.h*B2*fe;
  eso.e=u-eso.z1;
	eu=eso.B*fal(eso.e,0.5,drt);
	ez=eso.z2;
  //VAL_LIMIT(eso.z2,-500,500);
	
	
	eso.u=(eso.B*fal(eso.e,0.5,drt)-eso.z2);
}





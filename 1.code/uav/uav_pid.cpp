/*
  ******************************************************************************
  * @file   
  * @author  ytf
  * @version 
  * @date   
  * @brief    位置式pid
  *    
  *****************************************************************************
*/

#include "uav_pid.h"
#include "cal_math.h"

 
void _PID::Init( const float kp, const float ki,const float kd)
{
    PID.Kp=kp ; PID.Ki=ki;      PID.Kd=kd;\
	  PID.offsetKp=0 ;  PID.offsetKi=0 ;      PID.offsetKd=0 ;\
	  PID.err=0;   PID.lasterr=0;  PID.ec=0; PID.lastec=0;\
	  PID.current=0; PID.output=0;   PID.err_i=0;\
	  PID.errWgight=0;
}

//得到误差
void _PID::GetErr(float current)
{
  PID.err=(PID.expect-current);
}
//得到误差微分
//必须放在err_i 函数前面，不然会一直更新出来是不变的
void _PID::Err_d(float dt)
{
	  float pid_dFactor =dt / (dt + 1 / (2 * M_PI * 40));//低通对系统的稳定至关重要,参考陀螺仪采样低通频率
    PID.ec=( PID.err- PID.lasterr)*(0.01/dt);         //采样但是现在的单位是s归算一下参数不会特别夸张
	  PID.ec= PID.ec*pid_dFactor+ (1-pid_dFactor)*PID.lastec;
	  PID.lastec=PID.ec;
    
}
//得到误差积分
void _PID::Err_i(float dt)
{
   PID.err_i+=  (PID.err*dt);
	 PID.lasterr= PID.err;
}
//反向积分
void _PID::disErr_i(float dt, float max)
{
	if(PID.err>max)
	{
	  PID.err_i+=((PID.Ki+PID.offsetKi)*max*dt);
	}
	else if(PID.err<-max)
	{
	 PID.err_i+=(-(PID.Ki+PID.offsetKi)*max*dt);
	}
	else
	{
	 PID.err_i+=((PID.Ki+PID.offsetKi)*PID.err*dt);
	}
	 PID.lasterr= PID.err;
}

//计算实现
 void  _PID::Cal(void)
 {
	PID.output= ( PID.Kp+PID.offsetKp)* PID.err+PID.err_i+ (PID.Kd*PID.offsetKd)*PID.ec;
	                            
 }

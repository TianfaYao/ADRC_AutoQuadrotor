#include "control_v.h"
#include "cal_adrc.h"
#include "uav_rc.h"
#include "math.h"
#include "nav_speed.h"
#include "imu.h"
//#include "fal_initialize.h"
//#include "fal.h"
//#include "fhan.h"

 CONTROL_ ctrl;
 
 lpid  vzpid;
 void CONTROL_::V(void)
 {
	 static float leo=0;
   vzpid.bo=0;vzpid.b1=0;vzpid.b2=0;
	 
	 float a0=0.25;
	 float a1=0.75;
	 float a2=1.5;
	 float h=0.01;
	 
	 vzpid.e0=deathzoom(Rc.g_CH[thr],30)-nav_v.vz;
	 leo=vzpid.e0;
	 vzpid.e2=vzpid.e0-leo;
	 vzpid.e1 +=vzpid.e0;
	 VAL_LIMIT(vzpid.e1,-500,500);
	 vzpid.u=vzpid.bo*vzpid.e0+vzpid.b1*vzpid.e1+vzpid.b2*(-imu.ea.z);
	 wight=(1-fabs(deathzoom(Rc.g_CH[thr],30))/500.f);
	 
	 u=wight*vzpid.u+Rc.g_ThrVAL;
	 
  //vzpid.u= vzpid.bo*fal(vzpid.e0,a0,h)+vzpid.b1*fal(vzpid.e1,a1,h)+vzpid.b2*fal(vzpid.e2,a2,h);
	 
 }




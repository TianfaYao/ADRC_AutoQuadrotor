#include "nav_h.h"
#include "nav_h_ekf.h"
#include "dev_mavlink.h"
#define   gk 980.6f/4096.0f
 NAV_H_ nav_h;
 
 void NAV_H_::Init(void)
 {
   nav_h_ekf_init();

	   for(u8 j=0;j<9;j++)
		 {
		 Q[j]=0;
		 I[j]=0;
		 F[j]=0;
		 }
	
	 for(u8 j=0;j<6;j++)
	 {
		  H[j]=0;
   }
 	   for(u8 i=0;i<4;i++)
		 {
		  R[i]=0;
		 }
 
	 }
 //高度预估函数  ms 气压高度 ea 地理坐标系加速度
 void NAV_H_::cal( float ms,float ea,u8 &up, float dt)
 {

    Q[0]=0.01;
	  Q[4]=0.01;
	  Q[8]=0.01;

		R[0]=20;
		R[3]=1;
	 
    I[0]=1;
	  I[4]=1;
	  I[8]=1;

    
	  H[0]=1;
    H[5]=1;

	  F[0]=1;
		F[1]=dt;
		F[2]=0.5*dt*dt;
		F[4]=1;
		F[5]=dt;
		F[8]=1;

		Z[0]=ms;
		Z[1]=ea*gk; //cm/s
		static u8 run=0;
    if(fabs(ms)<50)
		{
			if(run==0)
			{
				run=1;
			}
		}
		if(run==1||run==100)
		{
			static s16 lms=0;
			s16 v;
			debug.aux3+=(s16)(Z[1]*dt);
			if(up==1)
			{
			v=(ms-lms)/dt*2;
			lms=ms;
			}
			//debug.aux5 =(s16)v;
			debug.aux1=(s16)ms;
			debug.aux2=(s16)v;
			
			run=100;
	    //nav_h_ekf(Q,H,R,F,I,Z,up,&h, &v, &a);
		  if(up==1)up=0;
			
		}
		
//		debug.aux1=(s16)h; 
//  	
//	  debug.aux3=(s16)a;
//		debug.aux4=(s16)Z[1];
//		debug.aux5=(s16)ms;
		
 };
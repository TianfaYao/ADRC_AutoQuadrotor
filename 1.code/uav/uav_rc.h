#ifndef  UAV_RC_H
#define  UAV_RC_H
#include "includes.h"
#include "uav_config.h"

class _RC
{
	public:
  float g_CH[6];
	u8    g_MODE;      //写的丑点容易认出来
	u8    g_Thr_Low;
	float g_ThrVAL;
	float g_Weight[4];
	u8    qudrotorState;
	void DataMange(float dt);
	private:
	void Deblock(float dt);
	void Mode(void);
	void Thr(void);
	void HoveringFlight(void );
};

extern _RC Rc;
#endif
#ifndef DEV_AK8975_H
#define DEV_AK8975_H

#include "includes.h"

class _AK8975 :public _I2C
{
	public:
	Vector3f  mag,lastmag;
	Vector3f  magTem;
	Vector3f  magOffset;
	u8 g_CallSetMagOffset;
	u8 update;
	void Configuration(void);
   void DataSample(void);
	 void SampleTriger(void);
	 void MagCalOffset(void);
	private:
	u8	buffer[6];
	
};

extern _AK8975 Ak8975;


#define AK8975_ADDRESS         0x0c	// 0x18

#define AK8975_WIA     0x00
#define AK8975_HXL     0x03
#define AK8975_HXH     0x04
#define AK8975_HYL     0x05
#define AK8975_HYH     0x06
#define AK8975_HZL     0x07
#define AK8975_HZH     0x08
#define AK8975_CNTL    0x0A




#endif

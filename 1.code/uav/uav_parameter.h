#ifndef UAV_PARAMETER_H
#define	UAV_PARAMETER_H
#include "stm32f4xx.h"
#include "cal_vector3.h"
	enum 
{
acc=1,
gyr,
mag,
anglepid,
angularpid,
ul_dis_spepid,
ms_dis_spepid,
};
class _PARAMETER{

	public :
		void Configuration(void);
	  void SaveAccOfferset(Vector3f soure);
	  void ReadAccOfferset(void);
	  void SaveGyrOfferset(Vector3f soure);
		void ReadGyrOfferset(void);
	  void SaveMagOfferset(Vector3f soure);
		void ReadMagOfferset(void);
	  void SaveAnglePID(void);
    void SaveAngularPID(void);
    void ReadAnglePID(void);
	  void ReadAngularPID(void);
	  void Save_UL_DIS_SPEPID(void);
	  void Read_UL_DIS_SPEPID(void);
	  void Save_MS_DIS_SPEPID(void);
	  void Read_MS_DIS_SPEPID(void);
	  void Reset(void);
	
	private:
		u32 FlashAdrr[40];

};

extern _PARAMETER Pamameter;
#endif


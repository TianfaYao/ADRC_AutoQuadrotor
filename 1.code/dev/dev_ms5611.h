#ifndef DEV_MS5611_H
#define DEV_MS5611_H
#include "stm32f4xx.h"
#include "dev_i2c.h"

#define MS5611_ADDR             0x77   //0xee //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096
#define BARO_CAL_CNT            200              
	#define BARO_SPEED_NUM 50
	
class _MS5611 :public _I2C
{
	private:
	uint8_t  TemperatureRx[3],PressureRx[3];
	uint16_t Prom[PROM_NB];
	int32_t  BaroOffset,BaroOffsetlast,BaroOffsetVAL;
	
	uint32_t MeasurTemperature;  // static result of temperature measurement
	uint32_t MeasurPressure;  // static result of pressure measurement
	

  float baro_speed_arr[BARO_SPEED_NUM + 1];
  u16 baro_cnt[2];
	
	void Reset(void);
	u8 ReadProm(void);
	//采样触发
	void TemperatureSampleTriger(void);
	void PressureSampleTriger(void);
	//数据采样
	void TemperatureSample(void);
  void PressureSample(void);
	//
	void BaroAltCalculate(void);
	public :
		//析构函数
		_MS5611(){
		for(u8 i=0;i<3;i++)
			{
				BaroOffset=0;
			TemperatureRx[i]=0;
			PressureRx[i]=0;
			}
		}
		
	u8 SensorReady;                  //传感器状态
  float    BaroAltSpeed,lastBaroAltSpeed,BaroAltSpeedMove;           //飞机速度
		
	int32_t  BaroAlt,BaroAltOld,BaroAltsou;     //飞机高度
	float    Temperature_5611;
	float    Temperature,Pressure;
	void     Configuration(void);
	u8       DataUpdate(void);
	u8       up;	
		
	
};

extern _MS5611 Ms5611;

#endif
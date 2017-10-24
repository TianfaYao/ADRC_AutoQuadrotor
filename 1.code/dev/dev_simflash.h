#ifndef DEV_SIMFLASH_H
#define DEV_SIMFLASH_H


#define STM32_FLASH_WREN 1              //使能FLASH写入(0，不是能;1，使能)
#define STM32_FLASH_SIZE 128	 		      //所选STM32的FLASH容量大小(单位为K)
#define STM32_FLASH_BASE 0x08000000 	  //STM32 FLASH的起始地址
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024            //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif

#define FLASH_SAVE_ADDR  0X08010000 	//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)


#include "stm32f4xx.h"

enum 
{
  gro,
	acc,
};
#define  SENSORNUM           2  										//传感器数目
#define  FASHCACHE_SIZE     sizeof(Vector3i)*SENSORNUM     //陀螺仪和加速度计的


class _FLSHSIMROM
{
	public:
		void flashInit(void);
	  void  sensorOffsetDataWrite(void);
	  void  sensorOffsetDataRead(void);

		void Test_Write(u32 WriteAddr,u16 WriteData);		
    void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead);   		//从指定地址开始读出指定长度的数据
    void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);		//从指定地址开始写入指定长度的数据	
	private:
		u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];                              //最多是2K字节
		u16 STMFLASH_ReadHalfWord(u32 faddr);		                          //读出半字  
  	void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);  

};

extern _FLSHSIMROM  flashsimrom;

#endif
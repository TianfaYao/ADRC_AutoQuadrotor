#ifndef DEV_FLASH_H
#define DEV_FLASH_H
#include "dev_spi.h"


class _FLASH25Q32 :public _SPI
{
	public :
		void Init_(void);
	
	// private:
		void SPI_Flash_Init(void);
		u16  SPI_Flash_ReadID(void);  	                                   
		u8	 SPI_Flash_ReadSR(void);       																 
		void SPI_FLASH_Write_SR(u8 sr);  																	
		void SPI_FLASH_Write_Enable(void);  															
		void SPI_FLASH_Write_Disable(void);																 
		void SPI_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);  
		void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
		void SPI_Flash_Erase_Chip(void);    	                            
		void SPI_Flash_Erase_Sector(u32 Dst_Addr);                         
		void SPI_Flash_Wait_Busy(void);                                   
		void SPI_Flash_PowerDown(void);                                    
		void SPI_Flash_WAKEUP(void);			                                
	  void SPI_Flash_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite) ;
	  void SPI_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
	  void  SPI_Flash_Read_f(u32 WriteAddr,float* pBuffer,u16 NumByteToWrite) ;
    void SPI_Flash_Write_f(u32 WriteAddr,float* pBuffer,u16 NumByteToWrite) ;
};
extern  _FLASH25Q32   flash25q32;

extern  u8 SPI_FLASH_BUF[4096];

#define W25Q80 	0XEF13 	
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16
 


#define SPI_FLASH_CS_EN    GPIO_ResetBits(GPIOA,  GPIO_Pin_4);
#define SPI_FLASH_CS_DIS   GPIO_SetBits(GPIOA,  GPIO_Pin_4);

//W25X16?¨¢D¡ä
#define FLASH_ID 0XEF14
//??¨¢?¡À¨ª
#define W25X_WriteEnable		0x06 
#define W25X_WriteDisable		0x04 
#define W25X_ReadStatusReg	0x05 
#define W25X_WriteStatusReg	0x01 
#define W25X_ReadData			  0x03 
#define W25X_FastReadData		0x0B 
#define W25X_FastReadDual		0x3B 
#define W25X_PageProgram		0x02 
#define W25X_BlockErase			0xD8 
#define W25X_SectorErase		0x20 
#define W25X_ChipErase			0xC7 
#define W25X_PowerDown			0xB9 
#define W25X_ReleasePowerDown	0xAB 
#define W25X_DeviceID			    0xAB 
#define W25X_ManufactDeviceID	0x90 
#define W25X_JedecDeviceID		0x9F 


#endif
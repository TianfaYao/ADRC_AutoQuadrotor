#include "stdio.h"
#include  "drv_w25qxx.h"

typedef struct 
{
    uint8_t Manufacturer;             /* Manufacturer ID */
    uint8_t Memory;               /* Density Code */
    uint8_t Capacity;                /* Family Code */
    uint8_t rev;               

}jedec_id_t;

/* Private define ------------------------------------------------------------*/

//#define W25QXX_DEBUG
#ifdef W25QXX_DEBUG
#define w25qxx_debug(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define w25qxx_debug(fmt, ...)
#endif

#define JEDEC_MANUFACTURER_ST       0x20
#define JEDEC_MANUFACTURER_MACRONIX 0xC2
#define JEDEC_MANUFACTURER_WINBOND  0xEF

/* JEDEC Device ID: Memory type and Capacity */
#define JEDEC_W25Q16_BV_CL_CV   (0x4015) /* W25Q16BV W25Q16CL W25Q16CV  */
#define JEDEC_W25Q16_DW         (0x6015) /* W25Q16DW  */
#define JEDEC_W25Q32_BV         (0x4016) /* W25Q32BV */
#define JEDEC_W25Q32_DW         (0x6016) /* W25Q32DW */
#define JEDEC_W25Q64_BV_CV      (0x4017) /* W25Q64BV W25Q64CV */
#define JEDEC_W25Q64_DW         (0x4017) /* W25Q64DW */
#define JEDEC_W25Q128_BV        (0x4018) /* W25Q128BV */


#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_SECTOR_ERASE           0x20

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80


#define DUMMY_BYTE     0xFF

#define W25QXX_CS_LOW()      GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET)
#define W25QXX_CS_HIGH()     GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET)

static flash_info_t flash_info;
static uint8_t Flash_SendByte(uint8_t byte);
static void Flash_WaitForEnd(void);
static uint8_t Flash_ReadID(jedec_id_t *id);
static void Flash_WriteEnable(void);
static void Flash_WriteDisable(void);

static uint8_t Flash_SendByte(uint8_t byte)
{
    /* Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

    /* Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI1, byte);

    /* Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    /* Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI1);
}
static uint32_t Flash_Transfer(uint8_t *send_buffer,uint8_t *recv_buffer,uint32_t len)
{
	uint8_t data;
	 while(len--)
	 {
     /* Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		 data = send_buffer ? *send_buffer++ : DUMMY_BYTE;

    /* Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI1, data);
		
    /* Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    /* Return the byte read from the SPI bus */
    data = SPI_I2S_ReceiveData(SPI1);
		if(recv_buffer)
		{
			*recv_buffer++ = data;
		}
	 } 
  return len;	 
}

jedec_id_t flash_id;
void Flash_Init(void)
{
	  
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    //SPI GPIO Configuration
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //flash SPI CS
    GPIO_InitStructure.GPIO_Pin             = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode            = GPIO_Mode_OUT ;   //推挽输出
    GPIO_InitStructure.GPIO_OType           = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd            = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed           = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //SPI configuration
    SPI_I2S_DeInit(SPI1);
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &SPI_InitStructure);
    SPI_Cmd(SPI1, ENABLE);
	  W25QXX_CS_HIGH();
		
	  /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send "0xff " instruction */
    Flash_SendByte(DUMMY_BYTE);
		
    W25QXX_CS_HIGH();
		
	  /* read flash id */
		Flash_ReadID(&flash_id);
		printf("%d\n",flash_id.Manufacturer);
		
    if (flash_id.Manufacturer == JEDEC_MANUFACTURER_WINBOND)
    {
			  flash_info.sector_size = 4096;                         /* Page Erase (4096 Bytes) */
        if (flash_id.Capacity == (JEDEC_W25Q128_BV&0xff))
        {
            w25qxx_debug("W25Q128_BV detection\r\n");
           
            flash_info.sector_count = 4096;                        /* 128Mbit / 8 / 4096 = 4096 */
        }
        else if (flash_id.Capacity == (JEDEC_W25Q64_DW&0xff))
        {
            w25qxx_debug("W25Q64_DW or W25Q64_BV or W25Q64_CV detection\r\n");
            flash_info.sector_count = 2048;                       /* 64Mbit / 8 / 4096 = 2048 */
        }
        else if (flash_id.Capacity == (JEDEC_W25Q32_DW&0xff))
				{
            w25qxx_debug("W25Q32_DW or W25Q32_BV detection\r\n");
            flash_info.sector_count = 1024;                       /* 32Mbit / 8 / 4096 = 1024 */
        }
        else if (flash_id.Capacity == (JEDEC_W25Q16_DW&0xff))
        {
            w25qxx_debug("W25Q16_DW or W25Q16_BV detection\r\n");
            flash_info.sector_count = 512;                       /* 16Mbit / 8 / 4096 = 512 */
        }
        else 
        {
            w25qxx_debug("error flash capacity\r\n");
            flash_info.sector_count = 0;                       
        }

        flash_info.capacity = flash_info.sector_size * flash_info.sector_count;
    }
    else
    {
        w25qxx_debug("Unknow Manufacturer ID!%02X\r\n", flash_id.Manufacturer);
			  flash_info.initialized = 0;
        return ;
    }

    flash_info.initialized = 1;
}
static void Flash_WriteEnable(void)
{
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send Write Enable instruction */
    Flash_SendByte(JEDEC_WRITE_ENABLE);
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();
}
static void Flash_WriteDisable(void)
{
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send Write Enable instruction */
    Flash_SendByte(JEDEC_WRITE_DISABLE);
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();
}
//Erases the specified FLASH sector.
void Flash_SectorErase(uint32_t address,uint8_t state)
{
	   Flash_WriteEnable();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send Sector Erase instruction */
    Flash_SendByte(JEDEC_SECTOR_ERASE);
    /* Send SectorAddr high nibble address byte */
    Flash_SendByte((address & 0xFF0000) >> 16);
    /* Send SectorAddr medium nibble address byte */
    Flash_SendByte((address & 0xFF00) >> 8);
    /* Send SectorAddr low nibble address byte */
    Flash_SendByte(address & 0xFF);
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    /* Wait the end of Flash writing */
    if(state)
    {
        Flash_WaitForEnd();
    }

}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE
  *         cycle(Page WRITE sequence). The number of byte can't exceed
  *         the FLASH page size.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *                  written to the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH,
  *                       must be equal or less than "SPI_FLASH_PageSize" value.
  * @retval : None
  */


void Flash_PageWrite(uint32_t address,uint8_t* buffer,  uint32_t lenght)
{
	   Flash_WriteEnable();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send "Write to Memory " instruction */
    Flash_SendByte(JEDEC_PAGE_WRITE);
    /* Send WriteAddr high nibble address byte to write to */
    Flash_SendByte((address & 0xFF0000) >> 16);
    /* Send WriteAddr medium nibble address byte to write to */
    Flash_SendByte((address & 0xFF00) >> 8);
    /* Send WriteAddr low nibble address byte to write to */
    Flash_SendByte(address & 0xFF);

    /* while there is data to be written on the FLASH */
    while (lenght--)
    {
        /* Send the current byte */
        Flash_SendByte(*buffer);
        /* Point on the next byte to be written */
        buffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    /* Wait the end of Flash writing */
    Flash_WaitForEnd();
   
}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param buffer : pointer to the buffer that receives the data read
  *                  from the FLASH.
  * @param address : FLASH's internal address to read from.
  * @param lenght : number of bytes to read from the FLASH.
  * @retval : None
  */
void Flash_PageRead(uint32_t address,uint8_t* buffer,  uint32_t lenght)
{

    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();

    /* Send "Read from Memory " instruction */
    Flash_SendByte(JEDEC_READ_DATA);

    /* Send ReadAddr high nibble address byte to read from */
    Flash_SendByte((address & 0xFF0000) >> 16);
    /* Send ReadAddr medium nibble address byte to read from */
    Flash_SendByte((address& 0xFF00) >> 8);
    /* Send ReadAddr low nibble address byte to read from */
    Flash_SendByte(address & 0xFF);

    while (lenght--) /* while there is data to be read */
    {
        /* Read a byte from the FLASH */
        *buffer = Flash_SendByte(DUMMY_BYTE);  //一个字节巴比特，一个浮点型32比特
        /* Point to the next location where the byte read will be saved */
        buffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

}

//Reads FLASH identification.
uint8_t Flash_ReadID(jedec_id_t *id)
{
    uint8_t *recv_buffer = (uint8_t*)id;

    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();

    /* Send "RDID " instruction */
    Flash_SendByte(JEDEC_DEVICE_ID);

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte(DUMMY_BYTE);

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte(DUMMY_BYTE);

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte(DUMMY_BYTE);

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();
		
		return id->Manufacturer;
}
 
static void Flash_WaitForEnd(void)
{
    u8 FLASH_Status = 0;

    /* Loop as long as the memory is busy with a write cycle */
    do
    {
			    /* Select the FLASH: Chip Select low */
        W25QXX_CS_LOW();
        /* Send "Read Status Register" instruction */
        Flash_SendByte(JEDEC_READ_STATUS);
        /* Send a dummy byte to generate the clock needed by the FLASH
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = Flash_SendByte(DUMMY_BYTE);
			 	/* Deselect the FLASH: Chip Select high */
       	W25QXX_CS_HIGH();
    }
    while (FLASH_Status & JEDEC_STATUS_BUSY);
		

}

//整个扇区的度
void Flash_SectorsRead(uint32_t address,uint8_t *buffer,uint16_t count)
{
	uint16_t i=0;
  printf("扇区大小 %d",flash_info.sector_size);
	for(i= 0;i<count;i++)
	{
//		  page=flash_info.sector_size/256;
//		  while(page--)
//			{
		  Flash_PageRead(address,buffer,flash_info.sector_size);
		  buffer += flash_info.sector_size;
		  address += flash_info.sector_size;
//			}
	}
}


//整个扇区的写
void Flash_SectorsWrite(uint32_t address,uint8_t *buffer,uint16_t count)
{
	uint16_t i=0,page=flash_info.sector_size/256;
  Flash_WriteEnable();
	for(i= 0;i<count;i++)
	{
		  Flash_SectorErase(address,1);
		  page=flash_info.sector_size/256;
		  while(page--)
			{
		  Flash_PageWrite(address,buffer,256);
		  buffer += 256;
		  address += 256;
			}
	}
	//Flash_WriteDisable();
}


flash_info_t *Flash_GetInfo(void)
{
	
 return &flash_info;
}


//====================================================================




/*
*********************************************************************************************************
*                                               STMFLASH_ReadByte
*
* 描述 : 从flash中读取一个字节
*
* 输入 : faddr：flash地址
*
* 输出 : 读取到的字节
*
* 调用 : 内部调用
*
* 说明 : 无
*********************************************************************************************************
*/
static u8 BSP_FLASH_ReadByte(u32 faddr)
{
	return *(vu8*)faddr; 
} 


/*
*********************************************************************************************************
*                                               STMFLASH_GetFlashSector
*
* 描述 : 获取指定地址所在flash的块首地址
*
* 输入 : addr：flash地址
*
* 输出 : 所在页首地址
*
* 调用 : 外部调用
*
* 说明 : 最高可支持1Mflash的STM32芯片
*********************************************************************************************************
*/
uint16_t BSP_FLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}


/*
*********************************************************************************************************
*                                               BSP_FLASH_Write
*
* 描述 : 从指定地址开始写入指定长度的数据
*
* 输入 : WriteAddr：起始地址
*        pBuffer：要写入数据存储的首地址
*        ByteToWrite：要写入的字节数，注意是字节数，如果写入字，需要用字数乘以4
*
* 输出 : 0:写入失败     1：写入成功
*
* 调用 : 外部调用
*
* 说明 : 因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
*        写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
*        写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
*        没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
*
*        本函数对OTP区域也有效!可以用来写OTP区!
*        OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
*********************************************************************************************************
*/
u8 BSP_FLASH_Write(u32 WriteAddr, u8 *pBuffer, u32 ByteToWrite)	
{ 
    FLASH_Status status = FLASH_COMPLETE;
    u8 res=1;
    u32 addrx=0;
    u32 endaddr=0;	
		int i = 0;
    
    u32 start_sector = 0;
    u32 end_sector = 0;
    
    if(WriteAddr<STM32_FLASH_BASE)return 0;	//非法地址
		FLASH_Unlock();									//解锁 
    FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
		addrx=WriteAddr;				//写入的起始地址
		endaddr=WriteAddr+ByteToWrite;	//写入的结束地址
    
    start_sector = BSP_FLASH_GetFlashSector(addrx);
    end_sector = BSP_FLASH_GetFlashSector(endaddr);
    
		if(addrx<0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
		{             
				 for(i = start_sector; i <= end_sector; i += 8)
				 {
						 status = FLASH_EraseSector(i, VoltageRange_3);
						 if(status!=FLASH_COMPLETE)
						 {
								 res = 0;	//发生错误了
								 break;
						 }           
				 }           
		}
			
		if(status == FLASH_COMPLETE)
		{
			while(WriteAddr < endaddr)//写数据
			{
				if(FLASH_ProgramByte(WriteAddr,*pBuffer) != FLASH_COMPLETE)//写入数据
				{ 
					res = 0;	//写入异常
									break;
				}
				WriteAddr+=1;
				pBuffer = (u8*)pBuffer+1;
			} 
		}
   
    FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
    FLASH_Lock();//上锁
    return res;
} 


/*
*********************************************************************************************************
*                                               BSP_FLASH_Write
*
* 描述 : 从指定地址开始读出指定长度的数据
*
* 输入 : ReadAddr：起始地址
*        pBuffer：要写入数据存储的首地址
*        ByteToWrite：要读出的字节数，注意是字节数，如果写入字，需要用字数乘以4
*
* 输出 : 无
*
* 调用 : 外部调用
*
* 说明 : 因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
*        写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
*        写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
*        没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
*
*        本函数对OTP区域也有效!可以用来写OTP区!
*        OTP区域地址范围:0X1FFF7800~0X1FFF7A0F
*********************************************************************************************************
*/
void BSP_FLASH_Read(u32 ReadAddr, u8 *pBuffer, u32 ByteToRead)   	
{
	u32 i;
//    u32 NumToRead = ((ByteToRead+3u)&(~3u))/4u;
	for(i=0;i<ByteToRead;i++)
	{
		pBuffer[i]=BSP_FLASH_ReadByte(ReadAddr);//读取1个字节.
		ReadAddr+=1;//偏移1个字节.	
	}
}




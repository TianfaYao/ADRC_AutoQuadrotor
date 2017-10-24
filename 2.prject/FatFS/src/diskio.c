/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "stm32f4xx.h"
#include "dev_sd.h"
#include  "stdio.h"
/* Definitions of physical drive number for each drive */
#define SD		 0	/*  Map spi nor flash to physical drive 0 */
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{

	switch (pdrv) {
	case SD :
		if(SD_Init()==RES_OK)
		{
     return RES_OK;
		}
    break;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{

	switch (pdrv) {
	case SD :
		printf("into disk_init fuction \n");
		int error;
	  while(error=SD_Init())
		{
      printf("init error \n");
		}
		return RES_OK;

	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	u8 res=1;
	switch (pdrv) {
	 case SD :
		printf("into disk_read fuction \n");
    res=SD_ReadDisk(buff,sector,count);			break;
	}
	if(!res)  	return RES_OK;
	else        return RES_PARERR;
  
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	u8 res=1;
	switch (pdrv) {
		case SD :
			res=SD_WriteDisk((u8*)buff,sector,count);		
		 // printf("into disk_write fuction [%d ]\n",res);break;
		}
  if(!res)  	return RES_OK;
	else        return RES_PARERR;
	}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	if (pdrv==SD) {
		printf("into disk_ioctl fuction \n");
							switch(cmd)
							{
								case CTRL_SYNC:
								res = RES_OK; 
										break;	 
								case GET_SECTOR_SIZE:
								*(DWORD*)buff = 512; 
										res = RES_OK;
										break;	 
								case GET_BLOCK_SIZE:
								*(WORD*)buff = SDCardInfo.CardBlockSize;
										res = RES_OK;
										break;	 
								case GET_SECTOR_COUNT:
										*(DWORD*)buff = SDCardInfo.CardCapacity/512;
										res = RES_OK;
										break;
								default:
										res = RES_PARERR;
										break;
							}
	}
	else
	{
		res=RES_ERROR;
		printf("not support this sd\n");
	}

	return res;
}
#endif

DWORD get_fattime (void)
{
	/* Pack date and time into a DWORD variable */
	return	 0;
}

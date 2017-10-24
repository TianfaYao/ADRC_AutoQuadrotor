#include "dev_i2c.h"




_I2C i2c; 
//模拟IIC初始化
void _I2C::Init(void)
{
  
	
}


void _I2C::Delay(void)
{
		__nop();	


	if(!I2C_FastMode)
	{
	u8 i=15;
	while(i--);
	}
}

int _I2C::Start(void)
{
	SDA_H;
	SCL_H;
	Delay();
	if(!SDA_read)return 0;	//SDA线为低电平则总线忙,退出
	SDA_L;
	Delay();
	if(SDA_read) return 0;	//SDA线为高电平则总线出错,退出
	SDA_L;
	Delay();
	return 1;	
}

void _I2C::Stop(void)
{
	SCL_L;
	Delay();
	SDA_L;
	Delay();
	SCL_H;
	Delay();
	SDA_H;
	Delay();
} 

void _I2C::Ack(void)
{	
	SCL_L;
	Delay();
	SDA_L;
	Delay();
	SCL_H;
	Delay();
	SCL_L;
	Delay();
}   

void _I2C::NoAck(void)
{	
	SCL_L;
	Delay();
	SDA_H;
	Delay();
	SCL_H;
	Delay();
	SCL_L;
	Delay();
} 

int _I2C::WaitAck(void) 	 
{
	u8 ErrTime = 0;
	SCL_L;
	Delay();
	SDA_H;			
	Delay();
	SCL_H;
	Delay();
	while(SDA_read)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			Stop();
			return 1;
		}
	}
	SCL_L;
	Delay();
	return 0;
}



void _I2C::SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        Delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        Delay();
				SCL_H;
				Delay();
    }
    SCL_L;
}  
//读1个字节，ack=1时，发送ACK，ack=0，发送NACK
u8 _I2C::ReadByte(u8 ask)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      Delay();
			SCL_H;
      Delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
   // return ReceiveByte;
		if (ask)
		Ack();
	else
		NoAck();  
    return ReceiveByte;
} 

//单字节写入*******************************************
int _I2C::SingleWrite(u8 SlaveAddress,u8 REG_Address,u8 REG_data)		
{
  	Start();
    SendByte(SlaveAddress<<1);   //
    if(WaitAck())
    {
		  Stop(); 
		  return 1;
		}
    SendByte(REG_Address );   //设置低起始地址      
    WaitAck();	
    SendByte(REG_data);
    WaitAck();   
    Stop(); 
    return 0;
}

//单字节读取
int _I2C::SingleRead(u8 SlaveAddress,u8 REG_Address,uint8_t *REG_data)
{   

    
		if(!Start())return 0;
   SendByte(SlaveAddress<<1); //SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
	
    if(WaitAck())
		{
			Stop();
			return 1;
		}
    SendByte(REG_Address);   //设置低起始地址      
    WaitAck();
    Start();
    SendByte(SlaveAddress<<1| 0x01);
    WaitAck();

		*REG_data= ReadByte(0);

    Stop();
	return 0;

}	


//多字节写入
int _I2C::MultWrite(u8 SlaveAddress, u8 REG_Address, u8 size, u8 *buf)
{	
	Start();
	SendByte(SlaveAddress<<1); 
	if(WaitAck())
	{
		Stop();
		return 1;
	}
	SendByte(REG_Address); 
	WaitAck();
	while(size--) 
	{
		SendByte(*buf++); 
		WaitAck();
	}
	Stop();
	return 0;
}




//多字节读取
int _I2C::MultRead(u8 SlaveAddress,u8 REG_Address,u8 size,u8 * buf)
{
    uint8_t i;
    Start();
    SendByte(SlaveAddress<<1);
    if(WaitAck())
		{
			Stop();
			return 1;
		}
    SendByte(REG_Address);    
    WaitAck();
    
    Start();
    SendByte(SlaveAddress<<1|0x01);
    WaitAck();
    
    for(i=1;i<size; i++)
    {
        *buf++ = ReadByte(1);
    }
    *buf++ = ReadByte(0);
    Stop();
    return 0;    
}	


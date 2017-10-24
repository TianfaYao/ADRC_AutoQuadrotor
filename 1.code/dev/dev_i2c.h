#ifndef DEV_I2C_H
#define DEV_I2C_H
#include "stm32f4xx.h"


#define SCL_H         DEV_GPIO_I2C->BSRRL = I2C_Pin_SCL
#define SCL_L         DEV_GPIO_I2C->BSRRH = I2C_Pin_SCL
#define SDA_H         DEV_GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         DEV_GPIO_I2C->BSRRH = I2C_Pin_SDA
#define SCL_read      DEV_GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      DEV_GPIO_I2C->IDR  & I2C_Pin_SDA

#define DEV_GPIO_I2C	 GPIOB
#define I2C_Pin_SCL		 GPIO_Pin_6
#define I2C_Pin_SDA		 GPIO_Pin_7
#define DEV_RCC_I2C		 RCC_AHB1Periph_GPIOB



class _I2C 
{
public:
  volatile u8 I2C_FastMode;
	 void Init(void);
	int SingleWrite(u8 SlaveAddress,u8 REG_Address,u8 REG_data);	
	int SingleRead(u8 SlaveAddress,u8 REG_Address,uint8_t *REG_data);
	int MultRead(u8 SlaveAddress,u8 REG_Address,u8 size,u8 * ptChar);
  int MultWrite(u8 SlaveAddress, u8 REG_Address, u8 size, u8 *buf);
private:
	void Delay(void);
	int Start(void);
	void Stop(void);
	void Ack(void); 
	void NoAck(void);
	int WaitAck(void); 	 //返回为:=1有ACK,=0无ACK
	void SendByte(u8 SendByte);
	u8 ReadByte(u8 ask);  //数据从高位到低位//
};
extern _I2C i2c; 


#endif
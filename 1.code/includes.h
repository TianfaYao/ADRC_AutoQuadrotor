#ifndef INCLUDES_H
#define INCLUDES_H
#include "stdio.h"
#include "stm32f4xx.h"

#define ACC_1G 			4096		//ÓÉ¼ÓËÙ¶È¼ÆµÄÁ¿³ÌÈ·¶
#define IMU_LOOP_TIME					2000	//µ¥Î»ÎªuS
#define ACC_LPF_CUT 10.0f		//¼ÓËÙ¶ÈµÍÍ¨ÂË²¨Æ÷½ØÖ¹ÆµÂÊ10H
#define GYRO_CF_TAU   1.2f
#define OFFSET_AV_NUM 50

#include "cal_math.h"
#include "dev_i2c.h"
#include "dev_usart.h"
#include "dev_port.h"
#include "dev_nvic.h"
#include "dev_port.h"
#include "dev_rcc.h"
#include "dev_clock.h"
#include "dev_mpu6050.h"
#include "dev_ms5611.h"
#include "dev_ak8975.h"
#include "dev_led.h"
#include "uav_init.h"
#include "dev_flash.h"
#include "dev_led.h"
#include "dev_dma.h"





#endif
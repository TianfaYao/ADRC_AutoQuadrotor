#ifndef DEV_CLOCK_H
#define DEV_CLOCK_H
#include "includes.h"

typedef struct
{
	u8 check_flag;
	u8 err_flag;
	s16 cnt_1ms;
	s16 cnt_2ms;
	s16 cnt_5ms;
	s16 cnt_10ms;
	s16 cnt_20ms;
	s16 cnt_50ms;
	s16 cnt_8ms;
	u16 time;
}loop_t;
typedef struct
{
u8  TheMachineRunningGood;
u32 TheMachineRunningTimeCount;	
	
}RuningStatus;

extern loop_t loop;
extern RuningStatus  MachineRuningStatus;
void  SysTick_Configuration(void);
void Clock_Config(void);
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
uint32_t GetSysTime_us(void) ;
void Loop_check(void);
float Get_Cycle_T(u8 item)	;



#endif
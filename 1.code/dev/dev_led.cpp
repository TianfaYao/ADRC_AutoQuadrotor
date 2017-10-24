#include "dev_led.h"
 _LED Led;
//led³õÊ¼»¯
void _LED::Configuration(void)
{
	//0 1 2 3                                            //23ºì¡¢13ÂÌ 12 À¶ 02 ×Ï  3°×
	 GPIO_SetBits(GPIOE,GPIO_Pin_2);
	 GPIO_SetBits(GPIOE,GPIO_Pin_3);
	 Delay_ms(20);
}
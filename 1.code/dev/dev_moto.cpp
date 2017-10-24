/*
  ******************************************************************************
  * @file   
  * @author  ytf
  * @version 
  * @date   
  * @brief  
  *    
  *****************************************************************************
*/
#include "dev_moto.h"
#define  Moto_TIM_Prescaler     (84-1)*2   
#define  Moto_TIM_Period        2500   
#define  Moto_TIM_Pulse         1000
_MOTO Moto;


 void _MOTO::MotoBaseInit(TIM_TypeDef* TIMx,u8 CH )
{
	 
    TIM_TimeBaseInitTypeDef         Moto_TIM;
    TIM_OCInitTypeDef               Moto_OC;

    Moto_TIM.TIM_Prescaler              = Moto_TIM_Prescaler;           
    Moto_TIM.TIM_CounterMode            = TIM_CounterMode_Up;         
    Moto_TIM.TIM_Period                 = Moto_TIM_Period;              
    Moto_TIM.TIM_ClockDivision          = TIM_CKD_DIV1;                 
    TIM_TimeBaseInit(TIMx, &Moto_TIM);
	
	
    Moto_OC.TIM_OCMode                  =TIM_OCMode_PWM2;
    Moto_OC.TIM_OutputState             = TIM_OutputState_Enable;
    Moto_OC.TIM_OutputNState            = TIM_OutputState_Disable;
    Moto_OC.TIM_Pulse                   =Moto_TIM_Pulse;                
    Moto_OC.TIM_OCPolarity              = TIM_OCPolarity_Low;
    Moto_OC.TIM_OCNPolarity             = TIM_OCPolarity_High;
    Moto_OC.TIM_OCIdleState             = TIM_OCIdleState_Reset;
    Moto_OC.TIM_OCNIdleState            = TIM_OCIdleState_Set;
	
    //  = Moto_TIM_Pulse/(Moto_TIM_Period+1)
																																					 
		switch (CH){
			
     case	1:		TIM_OC1Init(TIMx, &Moto_OC);
                  TIM_OC1PreloadConfig(TIMx,TIM_OCPreload_Enable); 
                    break;
		 case	2:	   	TIM_OC2Init(TIMx, &Moto_OC);
                  TIM_OC2PreloadConfig(TIMx,TIM_OCPreload_Enable); 
                    break;
		 case	3:	   	TIM_OC3Init(TIMx, &Moto_OC);
                  TIM_OC3PreloadConfig(TIMx,TIM_OCPreload_Enable); 
                    break;		
	 	 case	4:  	TIM_OC4Init(TIMx, &Moto_OC);
                 TIM_OC4PreloadConfig(TIMx,TIM_OCPreload_Enable); 
                    break;	
	
			}

	  TIM_CtrlPWMOutputs(TIMx, ENABLE);                                      
    TIM_ARRPreloadConfig(TIMx,ENABLE);
    TIM_Cmd(TIMx,ENABLE);
			
      TIM_SetCompare1(TIM1,Moto_TIM_Pulse);
			TIM_SetCompare2(TIM1,Moto_TIM_Pulse);
			TIM_SetCompare3(TIM1,Moto_TIM_Pulse);
		  TIM_SetCompare4(TIM1,Moto_TIM_Pulse);

		
}

void _MOTO::MotoInit(void)
{ 
	//M1
	  MotoBaseInit( TIM1,1);
	//M2
	  MotoBaseInit( TIM1,2);
	//M3
	  MotoBaseInit( TIM1,3);
	//M4
	  MotoBaseInit( TIM1,4);
}

void _MOTO::SetPwm(u16 moto[4])
{
 TIM_SetCompare1(TIM1, moto[3]+1000);
 TIM_SetCompare2(TIM1, moto[2]+1000);
 TIM_SetCompare3(TIM1, moto[1]+1000);
 TIM_SetCompare4(TIM1, moto[0]+1000);
};




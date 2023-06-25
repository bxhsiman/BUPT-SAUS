#include "servo.h"
/*
********************************************************************************************************
*                                               ʾ������
*                                             EXAMPLE  CODE                                             
*
*                             (c) Copyright 2021; SaiShu.Lcc.; Leo
*                                 ��Ȩ����[��������Ƽ����޹�˾]
*
*               The code is for internal use only, not for commercial transactions(��Դѧϰ,��������).
*               The code ADAPTS the corresponding hardware circuit board(����ʹ��CarDo�ǿذ�), 
*               the specific details consult the professional(��ӭ��ϵ����).
*********************************************************************************************************
*/

ServoStruct servoStr;


/**
* @brief        ������Ƴ�ʼ��
* @param        
* @ref          
* @author       Leo
* @note         
**/
void SERVO_Init(void)
{
    //PWM-IO��ʼ��
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; 			//PWM
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    //TIM��ʼ��
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef TIM_OCInitStructure;  
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure; 
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;  												// ϵͳ  72MHz    TIMʱ��= 72MHz/72=1M			
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseStructure.TIM_Period = 20000-1;   											// Frequency = 1000000 / 20000 = 50Hz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;   
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;     
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);    
      
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 									//����PWMģʽΪ���ϼ���ģʽ 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;  
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;  
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;  
    TIM_OCInitStructure.TIM_Pulse = 0;  

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  

    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;  
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;  
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;  
    TIM_OCInitStructure.TIM_Pulse = 0;  
		
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable; // Disable the Break function  
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;  
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable; //Enable Running State  
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable; //Enable Idle State  
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; //Set the lock level  
    TIM_BDTRInitStructure.TIM_DeadTime = 0x2B;  
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; //Enable the Auto Outputting.  
    TIM_BDTRConfig(TIM4, &TIM_BDTRInitStructure);  
      
    TIM_Cmd(TIM4, ENABLE);    
    TIM_CtrlPWMOutputs(TIM4, ENABLE);   
		
    SERVO_SetPwmValue(servoStr.thresholdMiddle);
}


/**
* @brief        ������PWM����
* @param        pwm��-20000~20000
* @ref          
* @author       Leo
* @note         
**/
void SERVO_SetPwmValue(signed int pwm)
{   
    pwm = 3000 - pwm;  //�����
        
    if(pwm < SERVO_PWM_MIN)
        pwm = SERVO_PWM_MIN;
    else if(pwm > SERVO_PWM_MAX)
        pwm = SERVO_PWM_MAX;
 
    TIM_SetCompare1(TIM4,pwm);
}


/**
* @brief        ������PWM���ã�������
* @param        pwm��500~2500
* @ref          
* @author       Leo
* @note         
**/
uint16_t pwm_Servo = 0;
void SERVO_SetPwmValueCorrect(signed int pwm)
{   
    pwm = 3000 - pwm;  //�����
    
    pwm -= servoStr.thresholdMiddle-SERVO_PWM_MIDDLE; //��ֵ����
	
	uint16_t pwmMax = 3000 - servoStr.thresholdLeft;
	uint16_t pwmMin = 3000 - servoStr.thresholdRight;
	if(pwm < pwmMin)
        pwm = pwmMin;
    else if(pwm > pwmMax)
        pwm = pwmMax;
	
	pwm_Servo = pwm;
    TIM_SetCompare1(TIM4,pwm);
}

/**
* @brief       ����Ƕȿ��� 
* @param        
* @ref          
* @author       
* @note         
**/
uint16_t ServoPwm = 1500;
void SERVO_AngleControl(float angle)
{
	uint16_t pwm = 1500;
	angle = -angle;
	if(angle > SERVO_ANGLE_MAX)
		angle = SERVO_ANGLE_MAX;
	else if(angle < -SERVO_ANGLE_MAX)
		angle = -SERVO_ANGLE_MAX;
	
	if(angle >= 0)  //��ת
		pwm = (float)angle/SERVO_ANGLE_MAX * (SERVO_PWM_MAX_R-servoStr.thresholdMiddle) + servoStr.thresholdMiddle;		//���ԽǶȼ���
	else if(angle < 0)  	//��ת
		pwm = (float)angle/SERVO_ANGLE_MAX * (servoStr.thresholdMiddle - SERVO_PWM_MAX_L) + servoStr.thresholdMiddle;		//���ԽǶȼ���
	
	ServoPwm = pwm;
	SERVO_SetPwmValue(pwm);
}


#include "motor.h"
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

MotorStruct motorStr;


/**
* @brief        ������Ƴ�ʼ��
* @param        
* @ref          
* @author       Leo
* @note         
**/
void MOTOR_Init(void)
{
    //PWM-IO��ʼ��
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);  
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8; 			//PWM
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 			//�������IO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_14);	
	
    //TIM��ʼ��
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
    TIM_OCInitTypeDef TIM_OCInitStructure;  
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure; 
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseStructure.TIM_Prescaler = 2;  										
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseStructure.TIM_Period = 2000-1;   //72M  3��Ƶ =  12KHz PWM
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;   
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;     
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);    
      
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //����PWMģʽΪ���ϼ���ģʽ 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;  
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;  
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;  
    TIM_OCInitStructure.TIM_Pulse = 0;  
    //Set the Channel 1 of TIMER1 
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);   

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
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);  
      
    TIM_Cmd(TIM1, ENABLE);    
    TIM_CtrlPWMOutputs(TIM1, ENABLE);   
		
    MOTOR_SetPwmValue(0);
    
    //���ģ�ͳ�ʼ��
    motorStr.EncoderLine = 512.0f; 							//����������=��դ��16*4				
    motorStr.ReductionRatio = 2.7f;							//������ٱ�								
    motorStr.EncoderValue = 0;
    motorStr.DiameterWheel = 0.064f;//68cm					//����ֱ��:m
    motorStr.CloseLoop = true;                              //Ĭ�ϱջ�ģʽ
}


/**
* @brief        ������PWM����
* @param        pwm��-2000~2000
* @ref          
* @author       Leo
* @note         
**/
void MOTOR_SetPwmValue(signed int pwm)
{   
    pwm = -pwm;
    if(pwm>=0)
    {
        GPIO_SetBits(GPIOB,GPIO_Pin_14);					
        if(pwm>MOTOR_PWM_MAX)
            pwm =MOTOR_PWM_MAX;
        
        TIM_SetCompare1(TIM1,pwm);
    }
    else if(pwm<0)
    {
        GPIO_ResetBits(GPIOB,GPIO_Pin_14);
        if(pwm<MOTOR_PWM_MIN)
            pwm=MOTOR_PWM_MIN;
        
        pwm = -pwm;

        TIM_SetCompare1(TIM1,pwm);
    }	
}


/**
* @brief        ����ջ��ٿ�
* @param        speed���ٶ�m/s
* @ref          
* @author       Leo
* @note         
**/
void MOTOR_ControlLoop(float speed)
{	
    if(speed > MOTOR_SPEED_MAX)
        speed = MOTOR_SPEED_MAX;
    else if(speed < -MOTOR_SPEED_MAX)
        speed = -MOTOR_SPEED_MAX;
    
    pidStr.vi_Ref = (float)(speed*MOTOR_CONTROL_CYCLE / motorStr.DiameterWheel / PI * motorStr.EncoderLine * 4.0f * motorStr.ReductionRatio);
    
    MOTOR_SetPwmValue(PID_MoveCalculate(&pidStr));
}


/**
* @brief        ��������߳�
* @param        
* @ref          
* @author       Leo
* @note         
**/
void MOTOR_Timer(void)
{
    motorStr.Counter++;
    if(motorStr.Counter >= 10)							    //�ٿ�:10ms
    {
        ENCODER_RevSample();								//����������

        if(icarStr.sprintEnable || usbStr.connected)        //ͨ�����ӻ������Բſ����ջ�������+ʡ�磩
        {
            if(motorStr.CloseLoop)
            {
                MOTOR_ControlLoop(icarStr.SpeedSet);		//�ջ��ٿ�
            }
            else//�����ٷֱȿ���
            {
                if(icarStr.SpeedSet > 100)
                    icarStr.SpeedSet = 100;
                else if(icarStr.SpeedSet < -100)
                    icarStr.SpeedSet = -100;
                signed int speedRate = MOTOR_PWM_MAX/100.f*icarStr.SpeedSet; //�������ٷֱ�%
                
                MOTOR_SetPwmValue(speedRate);		//�����ٿ�
            }
        }
        else
        {
            MOTOR_SetPwmValue(0);
        }
       
        motorStr.Counter = 0;
    }
}


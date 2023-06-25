#include "timer.h"
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


/**
* @brief        ��ʱ��TIM2��ʼ��
* @param        
* @ref          
* @author       Leo
* @note         
**/
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);              //ʱ��ʹ��

    TIM_TimeBaseStructure.TIM_Period = 1000-1;                        //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ  -> 10 * 1us = 10us
    TIM_TimeBaseStructure.TIM_Prescaler = 72-1;                       //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  -> 72M/(71+1) = 1us
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;           //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;       //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);                   //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE); 													//����ʱ��7�����ж�
    TIM_Cmd(TIM2, ENABLE);                                            //ʹ��TIM7

    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                   //TIM7�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;         //��ռ���ȼ�1��
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;                //�����ȼ�3��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                   //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);                                   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}


/**
* @brief        TIM2��ʱ�жϷ���
* @param        
* @ref          
* @author       Leo
* @note         
**/
void TIM2_IRQHandler(void)    //1ms����һ��
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)		//����ж�
    {	
        GPIO_Timer();			//GPIO�����߳�
        MOTOR_Timer();			//��������߳�
        SOC_Timer();			//�����Ƽ���߳�
        ICAR_Timer();           //���ܳ��ۺϴ����̼߳�����
        USB_Edgeboard_Timr();   //USBͨ���߳�
    }
    
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);				//����жϱ�־λ
}















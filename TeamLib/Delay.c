#include "delay.h"
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
* @brief        ��е��ʱ����
* @param        us��΢��
* @ref          2021��11��20�� 11:27:46
* @author       Leo
* @note         ����72MHz��Ƶ
**/
void Delay_Us(u32 us)
{
    int s,t = 0;
    for(s=0;s<us;s++)
    {
        for(t=0;t<10;t++)
        {
            __ASM("NOP");
        }
    }
}



/**
* @brief        ��е��ʱ����
* @param        ms������
* @ref          2021��11��20�� 11:27:43
* @author       Leo
* @note         ����72MHz��Ƶ
**/
void Delay_Ms(u16 ms)
{
    int t = 0;
    for(t=0;t<ms;t++)
    {
        Delay_Us(1000);		
    }
}




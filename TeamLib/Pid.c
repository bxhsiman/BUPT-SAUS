#include "pid.h"
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

PIDStruct pidStr;


/**
* @brief        PID������ʼ��
* @param        
* @ref          
* @author       Leo
* @note         
**/
void PID_Init(void)          
{	
    pidStr.vi_Ref = 0 ;	      	 
    pidStr.vi_FeedBack = 0 ;		
    pidStr.vi_PreError = 0 ;	  	
    pidStr.vi_PreDerror = 0 ;	  	    
    pidStr.v_Kp = PID_VKP;       
    pidStr.v_Ki = PID_VKI; 
    pidStr.v_Kd = PID_VKD;                
    pidStr.vl_PreU = 0;		
}


/**
* @brief        PID�ٿ�ģ��
* @param        
* @ref          
* @author       Leo
* @note         
**/
signed int PID_MoveCalculate(PIDStruct *pp)                        
{    
    float  error,d_error,dd_error; 
			 
    error = pp->vi_Ref - pp->vi_FeedBack;
    d_error = error - pp->vi_PreError;
    dd_error = d_error - pp->vi_PreDerror;          
        
    pp->vi_PreError = error;		                     
    pp->vi_PreDerror = d_error;
	
    if( ( error < VV_DEADLINE ) && ( error > -VV_DEADLINE ) )
    {
        ;             
    }
    else								
    { 
//PID���ְ��ͣ���Ҫʱ�����ã�����
//				I_error = pp -> v_Ki * error;
//				if(I_error >= (VV_MAX/5))
//				{
//				I_error = VV_MAX/5;
//				}
//				else if(I_error <= (VV_MIN/5))
//				{
//				I_error = VV_MIN/5;
//				}
//        pp->vl_PreU += (pp -> v_Kp * d_error + I_error + pp->v_Kd*dd_error)/3;
				  pp->vl_PreU += (pp -> v_Kp * d_error + pp -> v_Ki * error + pp->v_Kd*dd_error);
    } 
		pp->vl_PreU = pp->vl_PreU;
    if( pp->vl_PreU >= MOTOR_PWM_MAX )
    {
        pp->vl_PreU = MOTOR_PWM_MAX;
    }
    else if( pp->vl_PreU <= MOTOR_PWM_MIN )	
    {
        pp->vl_PreU = MOTOR_PWM_MIN;
    }
                                             
    return (pp->vl_PreU);
} 






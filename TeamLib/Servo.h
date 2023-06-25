#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"


#define  SERVO_PWM_MAX					2500						//����������PWM��180��
#define  SERVO_PWM_MIN					500							//���������СPWM��0��
#define  SERVO_PWM_MAX_L				1150						//�������ת�����ֵPWM
#define  SERVO_PWM_MAX_R				1850						//�������ת�����ֵPWM
#define  SERVO_PWM_MIDDLE				1500						//�����ֵPWM

#define  SERVO_ANGLE_MAX				38.0f						//���



/**
* @brief    ������
**/
typedef struct
{
	uint16_t thresholdMiddle;                   //�����ֵPWM
    uint16_t thresholdLeft;                     //�������ת�����ֵPWM
    uint16_t thresholdRight;                    //�������ת�����ֵPWM
}ServoStruct;

extern ServoStruct servoStr;

void SERVO_Init(void);
void SERVO_SetPwmValue(signed int pwm);
void SERVO_SetPwmValueCorrect(signed int pwm);
void SERVO_AngleControl(float angle);
#endif

//===========================================  End Of File  ===========================================//



#ifndef _IMU_H_
#define _IMU_H_

#include "main.h"


// ����MPU6050�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19		//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A		//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B		//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C		//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		  0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)


//****************************

#define	MPU6050_Addr   0xD0	  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

//************************************
/*ģ��IIC�˿�������붨��*/
#define SCL_H         GPIOB->BSHR = GPIO_Pin_14
#define SCL_L         GPIOB->BCR  = GPIO_Pin_14

#define SDA_H         GPIOB->BSHR = GPIO_Pin_13
#define SDA_L         GPIOB->BCR  = GPIO_Pin_13

#define SCL_read      GPIOB->INDR  & GPIO_Pin_14
#define SDA_read      GPIOB->INDR  & GPIO_Pin_13


typedef struct  
{
	uint16_t Counter;											//�̼߳�����
	short AacX;												    //X����ٶ�
	short AacY;												    //Y����ٶ�
	short AacZ;													//Z����ٶ�
	short GyroX;												//X����ٶ�
	short GyroY;												//Y����ٶ�
	short GyroZ;												//Z����ٶ�
}IMU_STA;


extern IMU_STA ImuStructure;


void IMU_Init(void);
void IMU_Handle(void);
void IMU_Timer(void);


#endif



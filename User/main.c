/* Includes ------------------------------------------------------------------*/
#include "main.h"


/**@mainpage                ��CarDO�������������� - ��Դ�ǿذ�
* <table>
* <tr><th>Project       <td>IntelligentCar
* <tr><th>Author        <td>Leo
* <tr><th>copyright     <td>Copyright@2016-2022 BjssTech, Ltd.All Rights Reserved.
**********************************************************************************
* </table>
* @section      CarDo�ǿذ�Ƕ��ʽ����
*
* @section      [�������][�������][USBͨ��][�˻�����][�Լ�]
*
* @section      ��������Ƽ����޹�˾��https://bjsstech.com/
* <table>
* <tr><th>Date          <th>Version     <th>Author      <th>Content         </tr>
* <tr><td>2022/02/09    <td>V1.0        <td>Leo         <td>InitialVersion  </tr>
* -#    Note:1
* -#    Note:2
* </tr>
* </table>
**********************************************************************************
*/


int main(void)
{
    Delay_Ms(100);                      //������ʱ���ȴ�ϵͳ�ȶ�
    FLASH_LoadAllConfig();              //����ϵͳ����
    
    ICAR_Init();                        //���ܳ�������ʼ��
    ENCODER_Init();                     //��������ʼ��
    GPIO_Initialize();                  //GPIO��ʼ��
    PID_Init();                         //PID������ʼ��
    MOTOR_Init();                       //�����ʼ��
    SERVO_Init();                       //�����ʼ��
    USB_Edgeboard_Init();               //USBͨ�ų�ʼ��
    SOC_Init();							//�����Ƴ�ʼ��
    RGB_Init();                         //RGB��ͨ�ų�ʼ��
    
    Delay_Ms(100);                      //������ʱ���ȴ�ϵͳ�ȶ�
    
    TIM2_Init();                        //ϵͳ���̶߳�ʱ����ʼ��
    GPIO_BuzzerEnable(BuzzerSysStart);  //������Ч
    
	while(1)
	{
        GPIO_Handle();                  //GPIO���ƣ�LED/������
        SOC_Handle();                   //�����Ʋ���
        ICAR_Handle();                  //���ܳ�����  
        FLASH_Handle();                 //Flash�洢
        USB_Edgeboard_Handle();         //USBͨ�ſ���
	}
    return 0;
}











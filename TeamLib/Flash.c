#include "flash.h"
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

bool flashSaveEnable = false;
volatile FLASH_Status FLASHStatus = FLASH_BUSY; // flash״̬

//��ʱ��ȡ����
uint8_t ArrayParams_ForFlash[FLASH_SIZE] = 
{
	/*1*/       0x00,0x00,0x00,0x00, 	//[Flag] [�����ֵ]
    /*2*/       0x00,0x00,0x00,0x00, 	//[Flag] [�����ת��ֵ]
    /*3*/       0x00,0x00,0x00,0x00 	//[Flag] [�����ת��ֵ]
};



/**
* @brief        �洢ϵͳ���ã�ȫ��/ALL��
* @param        
* @ref          
* @author       Leo
* @note         
**/
void FLASH_SaveAllConfig(void)
{
    int i = 0;
    uint8_t buff[4] = {0};
    Bint16_Union bint16_Union;
    
    memset(ArrayParams_ForFlash,0,sizeof(ArrayParams_ForFlash));
    
    /*1*/       buff[0] = FLASH_DATA_OK; 
                bint16_Union.U16 = servoStr.thresholdMiddle;//�����ֵPWM
                buff[1] = bint16_Union.U8_Buff[0];
                buff[2] = bint16_Union.U8_Buff[1];
                memcpy(ArrayParams_ForFlash+(i++)*4,buff,4);
    
    /*2*/       buff[0] = FLASH_DATA_OK; 
                bint16_Union.U16 = servoStr.thresholdLeft;//�����ת��ֵ
                buff[1] = bint16_Union.U8_Buff[0];
                buff[2] = bint16_Union.U8_Buff[1];
                memcpy(ArrayParams_ForFlash+(i++)*4,buff,4);
    
    /*3*/       buff[0] = FLASH_DATA_OK; 
                bint16_Union.U16 = servoStr.thresholdRight;//�����ת��ֵ
                buff[1] = bint16_Union.U8_Buff[0];
                buff[2] = bint16_Union.U8_Buff[1];
                memcpy(ArrayParams_ForFlash+(i++)*4,buff,4);
                
    FLASH_WriteBuffToFlash(0,ArrayParams_ForFlash,sizeof(ArrayParams_ForFlash));
}

/**
* @brief        ����ϵͳ���ã�ȫ��/ALL��
* @param        
* @ref          
* @author       Leo
* @note         
**/
void FLASH_LoadAllConfig(void)
{
    int i = 0;
    Bint16_Union bint16_Union;
	uint8_t buff[4];
    
    FLASH_ReadFlashNBtye(0,ArrayParams_ForFlash,sizeof(ArrayParams_ForFlash));//��ȡFlash����
    
    /*1*/		memcpy(buff,ArrayParams_ForFlash+(i++)*4,4);
                if(buff[0] == FLASH_DATA_OK)                //�����ֵPWM
                {
                    bint16_Union.U8_Buff[0] = buff[1];
                    bint16_Union.U8_Buff[1] = buff[2];
                    servoStr.thresholdMiddle = bint16_Union.U16;
                }
                else
                {
                    servoStr.thresholdMiddle = SERVO_PWM_MIDDLE;
                }
                
    /*2*/		memcpy(buff,ArrayParams_ForFlash+(i++)*4,4);
                if(buff[0] == FLASH_DATA_OK)                //�����ת��ֵ
                {
                    bint16_Union.U8_Buff[0] = buff[1];
                    bint16_Union.U8_Buff[1] = buff[2];
                    servoStr.thresholdLeft = bint16_Union.U16;
                }
                else
                {
                    servoStr.thresholdLeft = SERVO_PWM_MAX_L;
                }
                
    /*3*/		memcpy(buff,ArrayParams_ForFlash+(i++)*4,4);
                if(buff[0] == FLASH_DATA_OK)                //�����ת��ֵ
                {
                    bint16_Union.U8_Buff[0] = buff[1];
                    bint16_Union.U8_Buff[1] = buff[2];
                    servoStr.thresholdRight = bint16_Union.U16;
                }
                else
                {
                    servoStr.thresholdRight = SERVO_PWM_MAX_R;
                }
}


/**
* @brief        ��ȡflash��ָ��λ�õĲ���
* @param        *pch �洢�����ݵı���,iaddr �����ݵ�ƫ�Ƶ�ַ
* @ref          
* @author       Leo
* @note         
**/
void FLASH_ReadSpecifyParam(uint8_t *pch,int iaddr)
{
    FLASH_ReadFlashNBtye(0,ArrayParams_ForFlash,sizeof(ArrayParams_ForFlash));
}


/**
* @brief        ��Ƭ��flashд����
* @param        WriteAddress ƫ�Ƶ�ַ��pbuff д�������׵�ַ��num pbuff�Ĵ�С
* @ref          
* @author       Leo
* @note         
**/
void FLASH_WriteBuffToFlash(int WriteAddress,uint8_t * pbuff,int num)
{   
    int i = 0;
    uint16_t temp = 0;
    FLASH_UnlockBank1(); // ����flash
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    FLASHStatus = FLASH_BUSY;
    FLASHStatus = FLASH_ErasePage(FLASH_ADDR_START + WriteAddress);
    
    if(FLASHStatus == FLASH_COMPLETE)
    {
        FLASHStatus = FLASH_BUSY;
        for(i = 0;i < num;i ++)
        {
            temp = (uint16_t)pbuff[i];
            FLASHStatus = FLASH_ProgramHalfWord(FLASH_ADDR_START + WriteAddress + i*2,temp);
        }
    }
    FLASHStatus = FLASH_BUSY;
    FLASH_LockBank1();
}


/**
* @brief        ��flash�ж�����
* @param        ReadAddress ƫ�Ƶ�ַ��pbuff �洢�������飬ReadNum pbuff��С
* @ref          
* @author       Leo
* @note         
**/
void FLASH_ReadFlashNBtye(int ReadAddress, uint8_t *pbuff, int ReadNum)
{
    int i;
    int iDataNum = 0;
    uint8_t cReadBuff[2*ReadNum];
    ReadAddress = (unsigned int)FLASH_ADDR_START + ReadAddress;
    while(iDataNum < ReadNum*2)
    {
            *(cReadBuff + iDataNum) = *(__IO uint8_t*) ReadAddress++;
            iDataNum ++;
    }
    for(i = 0;i < ReadNum;i ++)
    {
        pbuff[i] = cReadBuff[i*2];
    }
}


/**
* @brief        Flash�����ݿ���
* @param        
* @ref          
* @author       Leo
* @note         
**/
void FLASH_Handle(void)
{
    if(flashSaveEnable)
    {		
        FLASH_SaveAllConfig();
        flashSaveEnable = false;
    }
}




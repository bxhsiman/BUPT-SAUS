#include "soc.h"
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

SocStruct socStr;

//----------------------------------------------[UNIT-SOC_IIC]----------------------------------------------------------
/********************************************************************************************************
Function Name: IIC_Init
Description  : IIC-IO��ʼ��
********************************************************************************************************/
void SOC_IIC_Init(void)
{			
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;		
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    SOC_IIC_SCL=1;
    SOC_IIC_SDA=1;
}


/**
* @brief				: ����IIC��ʼ�ź�
* @param        : None 
* @date					: None
* @author       : Fqy
* @note         : None
**/
void SOC_IIC_Start(void)
{
		SOC_SDA_OUT();     	//sda�����
		SOC_IIC_SDA=1;	  	  
		SOC_IIC_SCL=1;
		Delay_Us(4);
		SOC_IIC_SDA=0;				//START:when CLK is high,DATA change form high to low 
		Delay_Us(4);
		SOC_IIC_SCL=0;				//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  


/**
* @brief				: ����IICֹͣ�ź�
* @param        : None
* @date					: None
* @author       : Fqy
* @note         : None
**/
void SOC_IIC_Stop(void)
{
		SOC_SDA_OUT();				//sda�����
		SOC_IIC_SCL=0;
		SOC_IIC_SDA=0;				//STOP:when CLK is high DATA change form low to high
		Delay_Us(4);
		SOC_IIC_SCL=1; 
		SOC_IIC_SDA=1;				//����I2C���߽����ź�
		Delay_Us(4);							   	
}    


/**
* @brief				: �ȴ�Ӧ���źŵ���
* @param        : None
* @date					: None
* @author       : Fqy
* @note         : ����ֵ��1������Ӧ��ʧ�� ��0������Ӧ��ɹ�
**/

uint8_t SOC_IIC_Wait_Ack(void)
{
		uint8_t ucErrTime=0;
		SOC_SDA_IN();      					//SDA����Ϊ����  
		SOC_IIC_SDA=1;Delay_Us(1);	   
		SOC_IIC_SCL=1;Delay_Us(1);
	
		while(SOC_READ_SDA)
		{
			ucErrTime++;
			if(ucErrTime>250)
			{
				SOC_IIC_Stop();
				return 1;
			}
		}
		
		SOC_IIC_SCL=0;							//ʱ�����0 	 
		
		return 0;  
} 


/**
* @brief				: ����ACKӦ��
* @param        : None
* @date					: None
* @author       : Fqy
* @note         : None
**/
void SOC_IIC_Ack(void)
{
		SOC_IIC_SCL=0;
		SOC_SDA_OUT();
		SOC_IIC_SDA=0;
		Delay_Us(2);
		SOC_IIC_SCL=1;
		Delay_Us(2);
		SOC_IIC_SCL=0;
}


/**
* @brief				: ������ACKӦ��	
* @param        : None
* @date					: None
* @author       : Fqy
* @note         : None
**/
void SOC_IIC_NAck(void)
{
		SOC_IIC_SCL=0;
		SOC_SDA_OUT();
		SOC_IIC_SDA=1;
		Delay_Us(2);
		SOC_IIC_SCL=1;
		Delay_Us(2);
		SOC_IIC_SCL=0;
}	


/**
* @brief				: IIC����һ���ֽ�
* @param        : None
* @date					: None
* @author       : Fqy
* @note         : ���شӻ�����Ӧ��1����Ӧ��0����Ӧ��	
**/		  
void SOC_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
		SOC_SDA_OUT(); 	    
    SOC_IIC_SCL=0;					//����ʱ�ӿ�ʼ���ݴ���
	
    for(t=0;t<8;t++)
    {              
			SOC_IIC_SDA=(txd&0x80)>>7;
			txd<<=1; 	  
			Delay_Us(2);   				//��TEA5767��������ʱ���Ǳ����
			SOC_IIC_SCL=1;
			Delay_Us(2); 
			SOC_IIC_SCL=0;	
			Delay_Us(2);
    }	 
} 	    


/**
* @brief				: ��1���ֽ�
* @param        : None
* @date					: None
* @author       : Fqy
* @note         : ack=1ʱ������ACK��ack=0������nACK 
**/  
uint8_t SOC_IIC_Read_Byte(unsigned char ack)
{
		unsigned char i,receive=0;
		SOC_SDA_IN();								//SDA����Ϊ����
		
		for(i=0;i<8;i++ )
		{
			SOC_IIC_SCL=0; 
			Delay_Us(2);
			SOC_IIC_SCL=1;
			receive<<=1;
			
			if(SOC_READ_SDA)receive++;   
				Delay_Us(1); 
		}			
		
		if (!ack)
			SOC_IIC_NAck();//����nACK
		else
			SOC_IIC_Ack(); //����ACK  
		
		return receive;
}


/**
* @brief				: IIC����дָ����������
* @param        : reg:�Ĵ�����ַ��len:д�볤�ȣbbuff:����
* @date					: None
* @author       : Fqy
* @note         : ����ֵ:0,������,�������������
**/
uint8_t SOC_Write_Len(uint8_t reg,uint8_t len,uint8_t *buff)
{
		uint8_t i; 
		SOC_IIC_Start(); 
		SOC_IIC_Send_Byte(WRITE_CW2015);
		if(SOC_IIC_Wait_Ack())							//�ȴ�Ӧ��
		{
			SOC_IIC_Stop();		 
			return 1;		
		}
		
		SOC_IIC_Send_Byte(reg);						//д�Ĵ�����ַ
		SOC_IIC_Wait_Ack();								//�ȴ�Ӧ��
		
		for(i=0;i<len;i++)
		{
			SOC_IIC_Send_Byte(buff[i]);				//��������
			if(SOC_IIC_Wait_Ack())						//�ȴ�ACK
			{
				SOC_IIC_Stop();	 
				return 1;		 
			}		
		}    
		SOC_IIC_Stop();
		
		return 0;	
} 


/**
* @brief				: IICд������
* @param        : None
* @date					: None
* @author       : Fqy
* @note         : None
**/
uint8_t SOC_Write(uint8_t reg,uint8_t *buf)
{
		SOC_IIC_Start(); 
		SOC_IIC_Send_Byte(WRITE_CW2015);
		if(SOC_IIC_Wait_Ack())							//�ȴ�Ӧ��
		{
			SOC_IIC_Stop();		 
			return 1;		
		}
		
		SOC_IIC_Send_Byte(reg);						//д�Ĵ�����ַ
		SOC_IIC_Wait_Ack();								//�ȴ�Ӧ��
		
		SOC_IIC_Send_Byte(*buf);					//��������
		
		if(SOC_IIC_Wait_Ack())						//�ȴ�ACK
		{
			SOC_IIC_Stop();	 
			return 1;		 
		}  
		
		SOC_IIC_Stop();
		
		return 0;	
}


/**
* @brief				: IIC������ȡָ����������
* @param        : reg��Ҫ��ȡ�ļĴ�����ַ��len:Ҫ��ȡ�ĳ��ȣ�buff:��ȡ�������ݴ洢��
* @date					: None
* @author       : Fqy
* @note         : ����ֵ:0���������������������
**/
uint8_t SOC_Read_Len(uint8_t reg,uint8_t len,uint8_t *buff)
{ 
		SOC_IIC_Start(); 
		SOC_IIC_Send_Byte(WRITE_CW2015);
		if(SOC_IIC_Wait_Ack())									//�ȴ�Ӧ��
		{
			SOC_IIC_Stop();		 
			return 1;		
		}
		SOC_IIC_Send_Byte(reg);								//д�Ĵ�����ַ
		SOC_IIC_Wait_Ack();										//�ȴ�Ӧ��
		SOC_IIC_Start();
		SOC_IIC_Send_Byte(READ_CW2015);
		SOC_IIC_Wait_Ack();										//�ȴ�Ӧ�� 
		
		while(len)
		{
			if(len==1)*buff=SOC_IIC_Read_Byte(0);	//������,����nACK 
			else *buff=SOC_IIC_Read_Byte(1);			//������,����ACK  
			len--;
			buff++; 
		}    
		SOC_IIC_Stop();												//����һ��ֹͣ���� 
		
		return 0;	
}


/**
* @brief				: IIC��ȡָ����ַ������
* @param        : reg��Ҫ��ȡ�ļĴ�����ַ��buff:��ȡ�������ݴ洢��
* @date					: None
* @author       : Fqy
* @note         : None
**/
uint8_t SOC_Read(uint8_t reg,uint8_t *buff)
{ 
		SOC_IIC_Start(); 
		SOC_IIC_Send_Byte(WRITE_CW2015);
		if(SOC_IIC_Wait_Ack())									//�ȴ�Ӧ��
		{
			SOC_IIC_Stop();		 
			return 1;		
		}
		SOC_IIC_Send_Byte(reg);								//д�Ĵ�����ַ
		SOC_IIC_Wait_Ack();										//�ȴ�Ӧ��
		SOC_IIC_Start();
		SOC_IIC_Send_Byte(READ_CW2015);
		SOC_IIC_Wait_Ack();										//�ȴ�Ӧ�� 
		*buff = SOC_IIC_Read_Byte(0);					//������,����nACK 
		SOC_IIC_Stop();												//����һ��ֹͣ���� 
		
		return 0;	
}
//------------------------------------------------[END]-------------------------------------------------------------





//----------------------------------------------[UNIT-SOC-LOGIC]----------------------------------------------------------
int8_t no_charger_full_jump = 0;
unsigned int allow_no_charger_full =0;
unsigned int allow_charger_always_zero =0;
unsigned char if_quickstart =0;
unsigned char reset_loop =0;

/**
* @brief				: ���µ����Ϣ
* @param        : None
* @date					: None
* @author       : None
* @note         : ��������������Ǹ���ic�ڵĵ��profile��Ϣ��һ��ֻ����ic VDD��������ϵ�ʱ��ִ�� 
									return 1 : i2c��д�� return 2 : оƬ����sleepģʽ return 3 : д���profile��Ϣ������������еĲ�һ��
**/
unsigned char SOC_UpdataConfigInfo(void)
{
	int8_t ret = 0;
	unsigned char i;
	unsigned char reset_val;
	unsigned char reg_val;
	/* make sure no in sleep mode */
	ret = SOC_Read(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}
	if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP)
	{
		return 2;
	}
	/* update new battery info */
	for(i = 0; i < SIZE_BATINFO; i++)
	{
		reg_val = cw_bat_config_info[i];
		ret = SOC_Write(REG_BATINFO+i, &reg_val);
		if(ret)
		{
			return 1;
		}
	}

	/* readback & check */
	for(i = 0; i < SIZE_BATINFO; i++)
	{
		ret = SOC_Read(REG_BATINFO+i, &reg_val);
		if(ret)
		{
			return 1;
		}
		if(reg_val != cw_bat_config_info[i])
		{
			return 3;
		}
	}
	/* set cw2015/cw2013 to use new battery info */
	ret = SOC_Read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
	reg_val &= 0x07;                /* clear ATHD */
	reg_val |= ATHD;                /* set ATHD */
	ret = SOC_Write(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	/* reset */
	reset_val = MODE_NORMAL;
	reg_val = MODE_RESTART;
	ret = SOC_Write(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}
	Delay_Us(100);  //delay 100us      
	ret = SOC_Write(REG_MODE, &reset_val);
	if(ret)
	{
		return 1;
	}   
	return 0;
}


/**
* @brief				: �����Ƶײ��ʼ��
* @param        : None
* @date					: None
* @author       : None
* @note         : return 1 : i2c��д�� return 2 : оƬ����sleepģʽ return 3 : д���profile��Ϣ������������еĲ�һ�� return 4 : оƬ������30s�ڶ�����ֵһֱ�쳣
**/
unsigned char SOC_HardwareInit(void)
{
	unsigned ret;
	unsigned char i;
	unsigned char reg_val = MODE_NORMAL;

	/* wake up cw2015/13 from sleep mode */
	ret = SOC_Write(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}

	/* check ATHD if not right */
	ret = SOC_Read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	if((reg_val & 0xf8) != ATHD)
	{
		//"the new ATHD need set"
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = SOC_Write(REG_CONFIG, &reg_val);
		if(ret)
		{
			return 1;
		}
	}
	
	/* check config_update_flag if not right */
	ret = SOC_Read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	if(!(reg_val & CONFIG_UPDATE_FLG))
	{
		//"update flag for new battery info need set"
		ret = SOC_UpdataConfigInfo();
		if(ret)
		{
			return ret;
		}
	}
	else
	{
		for(i = 0; i < SIZE_BATINFO; i++)
		{ 
			ret = SOC_Read(REG_BATINFO +i, &reg_val);
			if(ret)
			{
				return 1;
			}
			if(cw_bat_config_info[i] != reg_val)
			{
				break;
			}
		}
		if(i != SIZE_BATINFO)
		{
			//"update flag for new battery info need set"
			ret = SOC_UpdataConfigInfo();
			if(ret)
			{
				return ret;
			}
		}
	}
	/* check SOC if not eqaul 255 */
	for (i = 0; i < 30; i++) {
		ret = SOC_Read(REG_SOC, &reg_val);
		if (ret)
			return 1;
		else if (reg_val <= 100) 
			break;
		Delay_Ms(100);//delay 100ms
    }
	
    if (i >=30){
        reg_val = MODE_SLEEP;
        ret = SOC_Write(REG_MODE, &reg_val);
        // "cw2015/cw2013 input unvalid power error_2\n";
        return 4;
    } 
	return 0;
}

#ifdef BAT_LOW_INTERRUPT


/**
* @brief				: �ͷ�alrt����
* @param        : None 
* @date					: None 
* @author       : None
* @note         : ��һ��alrt �¼�����ʱ��cw2015 ic������arlt pin�����жϣ���ʱ��Ҫ��06�Ĵ��������bitλ��0��������cw2015 ic�ͷ�alrt pin ���溯�����������ͷ�alrt pin
**/
unsigned char SOC_ReleaseAlrtPin(void)
{
		signed char ret = 0;
		unsigned int reg_val;
		unsigned char alrt;

		ret = SOC_Read(REG_RRT_ALERT, &reg_val);
		if(ret) 
		{
			return -1;
		}
		alrt = reg_val & 0x80;

		reg_val = reg_val & 0x7f;
		ret = SOC_Write(REG_RRT_ALERT, &reg_val);
		if(ret)
		{
			return -1;
		}

		return alrt;
}


/**
* @brief				: ���µ͵�������ֵ
* @param        : None
* @date					: None
* @author       : None
* @note         : �����������Ǹ����µĵ͵�澯ֵΪ�ϴε� -1�� �������ǵĴ��뿪ʼ��ʱ���趨�ĵ͵�澯ֵ��10���ǵ���������10��
									���ش������жϺ��Ұ��µĵ͵�澯ֵ9д���˶�Ӧ�ļĴ����С� ATHD��08�Ĵ�����ǰ5��bit
**/
int8_t SOC_UpdateAthd()
{
	int8_t ret = 0;
	unsigned char reg_val;
	char new_athd = 0;
	
	ret = SOC_Read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return -1;
	}
	new_athd = (reg_val >> 3) - 1;
	if(new_athd <= 0){
		new_athd = 0;
	}
	new_athd = new_athd << 3;

	//"the new ATHD need set"
	reg_val &= 0x07;    /* clear ATHD */
	reg_val |= new_athd;    /* set new ATHD */
	ret = SOC_Write(REG_CONFIG, &reg_val);
	if(ret)
	{
			return -1;
	}
	return 0;
}


/**
* @brief				: �жϷ�����
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
static void ALRT_ISR() //interrupt 
{
    /*User can do something when alrt */
	/*�ͻ�������������뵱�жϵ���ʱ�����Ĳ���*/
    SOC_ReleaseAlrtPin();
		SOC_UpdateAthd();
    /*User can write new alrt to CONFIG resiger*/
}
#endif


/**
* @brief				: �������ϵ縴λ
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
int SOC_Por(void)
{
	int8_t ret = 0;
	unsigned char reset_val = 0;
	reset_val = MODE_SLEEP;             
	ret = SOC_Write(REG_MODE, &reset_val);
	if (ret)
		return -1;
	Delay_Us(100); //delay 100us
	
	reset_val = MODE_NORMAL;
	ret = SOC_Write(REG_MODE, &reset_val);
	if (ret)
		return -1;
	Delay_Us(100); //delay 100us
	
	ret = SOC_HardwareInit();
	if (ret) 
		return ret;
	return 0;
}


/**
* @brief				: SOC��ȡ��ص���
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
int SOC_GetCapacity(void)
{
	int8_t ret = 0;
	unsigned char allow_capacity;
	unsigned char reg_val;
	//unsigned char reset_val;
	unsigned char cw_capacity;
	//int charge_time;

	ret = SOC_Read(REG_SOC, &reg_val);
	if(ret)
	{
		return -1;
	}
        
	cw_capacity = reg_val;
	/*����ic�������⣬��ȡ�������ں���ֵ��Χ��5�Σ�����ic������м������ȷ��ֵ����ô5�εļ�������0����ȷ��ʾ*/
	if (cw_capacity > 100)
		{
                // "get cw_capacity error; cw_capacity = %d\n"
        reset_loop++;
		if (reset_loop >5) { 
			ret = SOC_Por(); //por ic
			if(ret)
				return -1;
			reset_loop =0;               
		}                   
        return socStr.Capacity;
    }else {
        reset_loop =0;
    }
	
	/*����ǳ��״̬������Ӧ�����������ӵģ���������ʱ������ζ�ȡ�ĵ������ϴ�С����ʾ�ϴε�����*/
	/*ʲô����»�������������أ���Ϊ��˾��������С��λ�����籾�ε���0x04�Ĵ���������ֵ��9.01����Ϊ����ֻ��03�Ĵ��������Ը��ͻ���ʾ��ֵ��9�����´ζ�ȡ�ĵ�����8.99*/
	/*��ô�´���ʾ���ͻ��ĵ�����8���ӿͻ���ʾ��������̫�ÿ����������������*/
	/*�ŵ�ʱ����ͬ���Ĵ���*/
	if(((socStr.UsbOnline == 1) && (cw_capacity == (socStr.Capacity - 1)))
			|| ((socStr.UsbOnline == 0) && (cw_capacity == (socStr.Capacity + 1))))
	{
		// modify battery level swing
		if(!((cw_capacity == 0 && socStr.Capacity <= 2)||(cw_capacity == 100 && socStr.Capacity == 99)))
		{			
			cw_capacity = socStr.Capacity;
		}
	}
	
		/*����ǿͻ����������⣬�ͻ�ʹ�õ�charger ic�����ic�����Ȳ��Ǻܸߣ���Щicֻ�ܰѵ�س�絽4.1V�������ȡ�������ʱ�����ѹ�����4.2V����ô����Զ��ʾ����100%*/
		/*�����������������ַ�����1������д�Ĵ���������ʾ���ͻ�һ���ٵ�ֵ��һ������ӵ�100��
		2���������޸�profile��������ʾ100%ʱ�ĵ�ѹ�㣨��������������ǻ���Щcharger icֻ���õ�س䵽4.0�����������ˣ���
		��ǰ��oppo��һ��charger��������������
		*/
	if((socStr.UsbOnline == 1) && (cw_capacity >= 95) && (cw_capacity <= socStr.Capacity) )
	{     
		// avoid not charge full
		allow_no_charger_full++;
		if(allow_no_charger_full >= BATTERY_UP_MAX_CHANGE)
		{
			allow_capacity = socStr.Capacity + 1; 
			cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
			no_charger_full_jump =1;
			allow_no_charger_full =0;
		}
		else if(cw_capacity <= socStr.Capacity)
		{
			cw_capacity = socStr.Capacity; 
		}
	}
	/*��εĴ����ǵ�������ͨ���ϵ�����������ȥ�ļٵ�ֵ����ôҪ�üٵķ�����������������100ֱ���������ʵֵ�����*/
    else if((socStr.UsbOnline == 0) && (cw_capacity <= socStr.Capacity ) && (cw_capacity >= 90) && (no_charger_full_jump == 1))
	{
		// avoid battery level jump to CW_BAT
		if(socStr.UsbOnline == 0) 
		   allow_no_charger_full++;
		if(allow_no_charger_full >= BATTERY_DOWN_MIN_CHANGE)
		{
			allow_capacity = socStr.Capacity - 1;
			allow_no_charger_full =0; 
			if (cw_capacity >= allow_capacity)
			{
				no_charger_full_jump =0;
			}
			else
			{
				cw_capacity = (allow_capacity > 0) ? allow_capacity : 0;
			}
		}
		else if(cw_capacity <= socStr.Capacity)
		{
			cw_capacity = socStr.Capacity;
		}
	}
	else
    {
  		allow_no_charger_full =0;
    }
	
	/*ic�����ˣ����˺ܾ�һֱ����0%��һ�������ð��Сʱ����ô������ic*/
	if((socStr.UsbOnline > 0) && (cw_capacity == 0))
	{		  
		allow_charger_always_zero++;
		if((allow_charger_always_zero >= BATTERY_DOWN_MIN_CHANGE_SLEEP) && (if_quickstart == 0))
		{
      ret = SOC_Por(); //por ic
			if(ret){
				return -1;
			}
			if_quickstart = 1;
			allow_charger_always_zero =0;
		}
	}
	else if((if_quickstart == 1)&&(socStr.UsbOnline == 0))
	{
		if_quickstart = 0;
	}

	return(cw_capacity);
}


/**
* @brief				: ��ȡ��ѹֵ
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
unsigned int SOC_GetVol(void)
{
	int8_t ret = 0;
	unsigned char get_ad_times = 0;
	unsigned char reg_val[2] = {0 , 0};
	unsigned long ad_value = 0;
	unsigned int ad_buff = 0;
	unsigned int ad_value_min = 0;
	unsigned int ad_value_max = 0;

	for(get_ad_times = 0; get_ad_times < 3; get_ad_times++)
	{
		ret = SOC_Read(REG_VCELL, &reg_val[0]);
		if(ret)
		{
			return 1;
		}
		ret = SOC_Read(REG_VCELL + 1, &reg_val[1]);
		if(ret)
		{
			return 1;
		}
		ad_buff = (reg_val[0] << 8) + reg_val[1];

		if(get_ad_times == 0)
		{
			ad_value_min = ad_buff;
			ad_value_max = ad_buff;
		}
		if(ad_buff < ad_value_min)
		{
			ad_value_min = ad_buff;
		}
		if(ad_buff > ad_value_max)
		{
			ad_value_max = ad_buff;
		}
		ad_value += ad_buff;
	}
	ad_value -= ad_value_min;
	ad_value -= ad_value_max;
	ad_value = ad_value  * 305 / 1000;
	return(ad_value);       //14λADCת��ֵ
}


/**
* @brief				: SOC���µ�ص���
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
void SOC_UpdateCapacity(void)
{
	int cw_capacity;
	cw_capacity = SOC_GetCapacity();
	if((cw_capacity >= -5) && (cw_capacity <= 100) && (socStr.Capacity != cw_capacity))
	{       
		socStr.Capacity = cw_capacity;
		cw_capacity += 5;				//�Ż���ص����䲻��������
		if(cw_capacity>100)
			icarStr.Electricity = 100;
		else if(cw_capacity<0)
			icarStr.Electricity = 0;
		else
			icarStr.Electricity = cw_capacity;
	}
}


/**
* @brief				: SOC���µ�ص�ѹ
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
uint8_t Index = 0;
uint8_t uSocErrorCnt= 0;
void SOC_UpdateVol(void)
{
	unsigned int cw_voltage;
	cw_voltage = SOC_GetVol();
	if(cw_voltage == 1){
		//read voltage error
		socStr.voltage = socStr.voltage;
	}else if(socStr.voltage != cw_voltage)
	{
		socStr.voltage = cw_voltage;
		icarStr.Voltage = (float)cw_voltage*3.f/1000.f;
	}
	
	if(icarStr.Voltage < 11.3f)	//��ѹ<11.3v��ʼ����
	{
		uSocErrorCnt ++;
		if(uSocErrorCnt > 2)
		{
			GPIO_BuzzerEnable(BuzzerWarnning);
            uSocErrorCnt=0;
		}
	}
	else
	{
		uSocErrorCnt = 0;
	}
}


/**
* @brief				: USB����״̬���
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
void SOC_UpdateUsbOnline(void)
{
	if(0) 
	//������ע�⣬�����ǿͻ���Ҫ�Լ������޸ĵĵط�
	//���޸Ĵ��뱣֤DC����ʱ����soc_Struct.usb_onlineΪ 1��DC����ʱ����soc_Struct.usb_onlineΪ0
	{
		socStr.UsbOnline = 1;
	}else{
		socStr.UsbOnline = 0;
	}
}


/**
* @brief				: SOC������
* @param        : None
* @date					: None
* @author       : None
* @note         : None
**/
void SOC_BatWork(void)
{
	SOC_UpdateUsbOnline();
	SOC_UpdateVol();
    SOC_UpdateCapacity();
}


/**
* @brief				: SOC��ʼ��
* @param        : None
* @date					: None
* @author       : None
* @note         : return 1 : i2c��д�� return 2 : оƬ����sleepģʽ return 3 : д���profile��Ϣ������������еĲ�һ�� return 4 : оƬ������30s�ڶ�����ֵһֱ�쳣
**/
unsigned char SOC_Init(void)
{
	unsigned char ret;
	unsigned char loop = 0;
	//cw_bat_gpio_init();
	SOC_IIC_Init();
	
	ret = SOC_HardwareInit();
	
	while((loop++ < 200) && (ret != 0))
	{
		ret = SOC_HardwareInit();
	}	

	socStr.UsbOnline = 0;
	socStr.Capacity = 2;
	socStr.voltage = 0;
	
	return ret;	
}


/**
* @brief				: ������IC����ʱ��
* @param        : None
* @date					: None
* @author       : Leo
* @note         : None
**/
void SOC_Timer(void)
{
	socStr.Counter++;
	if(socStr.Counter>200000)
		socStr.Counter = 200000;
}


/**
* @brief				: ������IC�����߼�
* @param        : None
* @date					: None
* @author       : Leo
* @note         : None
**/
void SOC_Handle(void)
{
	if(socStr.Counter>1000)//1s
	{
		SOC_BatWork();
		socStr.Counter = 0;
	}
}
//------------------------------------------------[END]-------------------------------------------------------------





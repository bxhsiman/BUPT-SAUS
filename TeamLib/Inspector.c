#include "Inspector.h"


UsbStruct usbInspector;
InspectorStr inspectorStructure;

/**
* @brief        USB/UART��ʼ��
* @param        
* @ref          
* @author       Leo
* @note         
**/
void USB_Inspector_Init(void)
{  
    //UART2��ʼ��
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //PA2,PA3 ����IO�ڹ����������ô��ڡ����Ҫʹ��AFIO�����ù���IO��ʱ�ӡ�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		//��������ģʽ	   
    GPIO_Init(GPIOA, &GPIO_InitStructure);					 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;			  	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;	
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;			//����λ8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;					//ֹͣλ1λ
    USART_InitStructure.USART_Parity = USART_Parity_No;							//��У��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //��Ӳ������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure);//���ô��ڲ�������
    USART_Cmd(USART2, ENABLE); 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            			//ʹ�ܽ����ж�
        
    //USB���ݳ�ʼ��
    usbInspector.counter = 0;
    usbInspector.receiveFinished = false;
    usbInspector.receiveStart = false;
    usbInspector.receiveIndex = 0;
}

/**
* @brief        USB-TypeC(������)����һ���ֽ�
* @param        
* @ref          
* @author       Leo
* @note         
**/
void USB_Inspector_TransmitByte(uint8_t data)
{
    USART2->STATR;
    USART_SendData(USART2, data);
    while(USART_GetFlagStatus(USART2,USART_FLAG_TC) != SET);	//�ȴ����ͽ���
}


/**
* @brief        USB/UART�����жϺ���
* @param        
* @ref          
* @author       Leo
* @note         
**/
void USART2_IRQHandler(void)  
{	 
    uint8_t Uart1Res;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {   
        Uart1Res = USART_ReceiveData(USART2); 
        if(Uart1Res == USB_FRAME_HEAD && !usbInspector.receiveStart)//���֡ͷ
        {
            usbInspector.receiveStart = true;
            usbInspector.receiveBuff[0] = Uart1Res;
            usbInspector.receiveBuff[2] = USB_FRAME_LENMIN;
            usbInspector.receiveIndex = 1;
        }
        else if(usbInspector.receiveIndex == 2)	//����֡����
        {
            usbInspector.receiveBuff[usbInspector.receiveIndex] = Uart1Res;
            usbInspector.receiveIndex++;
            
            if(Uart1Res > USB_FRAME_LENMAX || Uart1Res < USB_FRAME_LENMIN) //֡������
            {
                usbInspector.receiveBuff[2] = USB_FRAME_LENMIN;
                usbInspector.receiveIndex = 0;
                usbInspector.receiveStart = false;
            }
        }
        else if(usbInspector.receiveStart && usbInspector.receiveIndex < USB_FRAME_LENMAX)
        {
            usbInspector.receiveBuff[usbInspector.receiveIndex] = Uart1Res;
            usbInspector.receiveIndex++;
        }
        
        //����֡���
        if((usbInspector.receiveIndex >= USB_FRAME_LENMAX || usbInspector.receiveIndex >= usbInspector.receiveBuff[2]) && usbInspector.receiveIndex > USB_FRAME_LENMIN)
        {
            uint8_t check = 0;
            uint8_t length = USB_FRAME_LENMIN;
        
            length = usbInspector.receiveBuff[2];
            for(int i=0;i<length-1;i++)
                check += usbInspector.receiveBuff[i];
            
            if(check == usbInspector.receiveBuff[length-1])//У��λ
            {
                memcpy(usbInspector.receiveBuffFinished,usbInspector.receiveBuff,USB_FRAME_LENMAX);	
                usbInspector.receiveFinished = true;
                
                //���ܳ�����ָ�����⴦��
                if(USB_ADDR_CONTROL  == usbInspector.receiveBuffFinished[1])
                {
                    Bint16_Union bint16_Union;
                    Bint32_Union bint32_Union;
                    for(int i=0;i<4;i++)
                        bint32_Union.U8_Buff[i] = usbInspector.receiveBuffFinished[3+i];
                    
                    bint16_Union.U8_Buff[0] = usbInspector.receiveBuffFinished[7];
                    bint16_Union.U8_Buff[1] = usbInspector.receiveBuffFinished[8];
                    
                    SERVO_SetPwmValueCorrect(bint16_Union.U16);
                    icarStr.ServoPwmSet = bint16_Union.U16;
                    if(motorStr.CloseLoop)//�ջ��ٿ�
                        icarStr.SpeedSet = bint32_Union.Float;   
                    else
                        icarStr.SpeedSet = MOTOR_PWM_MAX /100.f*bint32_Union.Float; //�������ٷֱ�%
                    GPIO_BuzzerEnable(BuzzerOk);
                }
            }
            
            usbInspector.receiveIndex = 0;
            usbInspector.receiveStart = false;
        }
    }

    USART_ClearFlag(USART2, USART_IT_RXNE); 
}



/**
* @brief        USBͨ�Ŵ�����
* @param        
* @ref          
* @author       Leo
* @note         
**/
void USB_Inspector_Handle(void)
{
    
    //-----------------------[���ݽ��մ���]-----------------------------
    if(usbInspector.receiveFinished)																//���ճɹ�
    {
        usbInspector.receiveFinished = false;
        Bint32_Union bint32_Union;
        Bint16_Union bint16_Union;
           
        if(usbInspector.receiveBuffFinished[1] & 0x80)	//������
        {
            uint8_t Addr = (uint8_t)(usbInspector.receiveBuffFinished[1] & 0x7F);
            switch(Addr)
            {
                case USB_ADDR_BATTERY :      //�����Ϣ
                    break;
                
                case USB_ADDR_SERVOTHRESHOLD :   //�����ֵ
                    break;
            }
        }
        else //д����
        {
            switch(usbInspector.receiveBuffFinished[1])
            {
                case USB_ADDR_HEART :    //����������
                    inspectorStructure.enable = true;
                    inspectorStructure.counterDrop = 0;
                    break;
                
                case USB_ADDR_SERVOTHRESHOLD :   //�����ֵ
                    if(usbInspector.receiveBuffFinished[3] == 1)          //��ת��ֵ
                    {
                        bint16_Union.U8_Buff[0] = usbInspector.receiveBuffFinished[4];
                        bint16_Union.U8_Buff[1] = usbInspector.receiveBuffFinished[5];
                        servoStr.thresholdLeft = bint16_Union.U16;
                        SERVO_SetPwmValue(servoStr.thresholdLeft);
                        GPIO_BuzzerEnable(BuzzerDing);
                        flashSaveEnable = true;
                    }
                    else if(usbInspector.receiveBuffFinished[3] == 2)     //��ת��ֵ
                    {
                        bint16_Union.U8_Buff[0] = usbInspector.receiveBuffFinished[4];
                        bint16_Union.U8_Buff[1] = usbInspector.receiveBuffFinished[5];
                        servoStr.thresholdRight = bint16_Union.U16;
                        SERVO_SetPwmValue(servoStr.thresholdRight);
                        GPIO_BuzzerEnable(BuzzerDing);
                        flashSaveEnable = true;
                    }
                    else if(usbInspector.receiveBuffFinished[3] == 3)     //��ֵ
                    {
                        bint16_Union.U8_Buff[0] = usbInspector.receiveBuffFinished[4];
                        bint16_Union.U8_Buff[1] = usbInspector.receiveBuffFinished[5];
                        servoStr.thresholdMiddle = bint16_Union.U16;
                        SERVO_SetPwmValue(servoStr.thresholdMiddle);
                        GPIO_BuzzerEnable(BuzzerDing);
                        flashSaveEnable = true;
                    }
                    break;
                
                case USB_ADDR_BUZZER :      //��������Ч
                    if(usbInspector.receiveBuffFinished[3] == 1)          //OK
                        GPIO_BuzzerEnable(BuzzerOk);
                    else if(usbInspector.receiveBuffFinished[3] == 1)     //Warnning
                        GPIO_BuzzerEnable(BuzzerWarnning);
                    else if(usbInspector.receiveBuffFinished[3] == 1)     //Finish
                        GPIO_BuzzerEnable(BuzzerFinish);
                    else if(usbInspector.receiveBuffFinished[3] == 1)     //Ding
                        GPIO_BuzzerEnable(BuzzerDing);
                    else if(usbInspector.receiveBuffFinished[3] == 1)     //SystemStart
                        GPIO_BuzzerEnable(BuzzerSysStart);
                    
                    break;
                
                case USB_ADDR_LIGHT :         //LED��Ч
                    for(int i=0;i<4;i++)
                        bint32_Union.U8_Buff[i] = usbInspector.receiveBuffFinished[i+3];
                
                    RGB_SetAllColor((unsigned long)bint32_Union.U32);
                    rgbStr.lastColor = (unsigned long)bint32_Union.U32;
                
                    break;
                
                case USB_ADDR_SPEEDMODE :        //�ٿ�ģʽ�л�
                    if(usbInspector.receiveBuffFinished[3] == 1)    //����ģʽ
                        motorStr.CloseLoop = false;                    
                    else
                        motorStr.CloseLoop = true;
                    
                    icarStr.SpeedSet = 0;
                    GPIO_BuzzerEnable(BuzzerDing);
                    break;
                    
                case USB_ADDR_SELFCHECK :     //���ܳ��Լ�
                {
                    ICAR_SelfcheckControl(usbInspector.receiveBuffFinished[3]);
                    break;
                }
            }
        }
    }
    
    //-----------------------[���ݷ��ʹ���]-----------------------------
    if(inspectorStructure.enable && inspectorStructure.counterSend > 150)//150ms
    {
        USB_SendToInspector_ServoThreshold(1);
        Delay_Ms(1);
        USB_SendToInspector_ServoThreshold(2);
        Delay_Ms(1);
        USB_SendToInspector_ServoThreshold(3);
        Delay_Ms(1);
        USB_SendToInspector_BatteryInfo();
        Delay_Ms(1);
        USB_SendToInspector_CarSpeed();
        inspectorStructure.counterSend = 0; 
    }
}
	


/**
* @brief        �������߳̿�����
* @param        
* @ref          
* @author       Leo
* @note         
**/
void USB_Inspector_Timer(void)
{
    if(inspectorStructure.enable)
    {
        inspectorStructure.counterSend++;
        inspectorStructure.counterDrop++;
        if(inspectorStructure.counterDrop >3000)//3s
        {
            inspectorStructure.enable = false;
            icarStr.selfcheckEnable = false;
        }
    }
    
}

//----------------------------------------------[UNIT-��λ���������ͨ��]----------------------------------------------------------

/**
* @brief        ���Ͷ����ֵ
* @param        chanel: 1/��ת��ֵ��2/��ת��ֵ��3/��ֵ
* @ref          
* @author       Leo
* @note         
**/
void USB_SendToInspector_ServoThreshold(uint8_t chanel)
{
    if(chanel<1 || chanel>3)
        return;
    
    Bint16_Union bint16_Union;
    uint8_t check = 0;
    uint8_t buff[9];
    buff[0] = 0x42; //֡ͷ
    buff[1] = USB_ADDR_SERVOTHRESHOLD ; //��ַ
    buff[2] = 0x07; //֡��
    buff[3] = chanel; //ͨ��
    
    switch(chanel)
    {
        case 1:
        {  
            bint16_Union.U16 = servoStr.thresholdLeft;
            buff[4] = bint16_Union.U8_Buff[0];
            buff[5] = bint16_Union.U8_Buff[1];
            break;
        }
        case 2:
        {
            bint16_Union.U16 = servoStr.thresholdRight;
            buff[4] = bint16_Union.U8_Buff[0];
            buff[5] = bint16_Union.U8_Buff[1];
            break;
        }
        case 3:
        {
            bint16_Union.U16 = servoStr.thresholdMiddle;
            buff[4] = bint16_Union.U8_Buff[0];
            buff[5] = bint16_Union.U8_Buff[1];
            break;
        }
    }
    
    for(int i=0;i<6;i++)
        check += buff[i];
    
    buff[6] = check;
    
    for(int i=0;i<9;i++)
        USB_Inspector_TransmitByte(buff[i]);
}

/**
* @brief        ���Ͱ�����Ӧ��Ϣ
* @param        time: ����ʱ��/ms
* @ref          
* @author       Leo
* @note         
**/
void USB_SendToInspector_KeyPress(uint16_t time)
{
    if(time<100)
        return;
    
    Bint16_Union bint16_Union;
    uint8_t check = 0;
    uint8_t buff[8];
    buff[0] = 0x42; //֡ͷ
    buff[1] = USB_ADDR_KEYINPUT ; //��ַ
    buff[2] = 0x06; //֡��
        
    bint16_Union.U16 = time;
    buff[3] = bint16_Union.U8_Buff[0];
    buff[4] = bint16_Union.U8_Buff[1];
    
    for(int i=0;i<5;i++)
        check += buff[i];
    
    buff[5] = check;
    
    for(int i=0;i<8;i++)
        USB_Inspector_TransmitByte(buff[i]);
}


/**
* @brief        ���͵����Ϣ
* @ref          
* @author       Leo
* @note         
**/
void USB_SendToInspector_BatteryInfo(void)
{
    Bint32_Union bint32_Union;
    uint8_t check = 0;
    uint8_t buff[11];
    buff[0] = 0x42; //֡ͷ
    buff[1] = USB_ADDR_BATTERY ; //��ַ
    buff[2] = 0x09; //֡��
        
    buff[3] = icarStr.Electricity; //����
    bint32_Union.Float = icarStr.Voltage;
    buff[4] = bint32_Union.U8_Buff[0];
    buff[5] = bint32_Union.U8_Buff[1];
    buff[6] = bint32_Union.U8_Buff[2];
    buff[7] = bint32_Union.U8_Buff[3];

    for(int i=0;i<8;i++)
        check += buff[i];
    
    buff[8] = check;
    
    for(int i=0;i<11;i++)
        USB_Inspector_TransmitByte(buff[i]);
}

/**
* @brief        �����Լ���Ϣ
* @ref          
* @author       Leo
* @note         
**/
void USB_SendToInspector_Selfcheck(uint8_t step)
{
    Bint16_Union bint16_Union;
    uint8_t check = 0;
    uint8_t buff[9];
    buff[0] = 0x42; //֡ͷ
    buff[1] = USB_ADDR_SELFCHECK ; //��ַ
    buff[2] = 0x07; //֡��
        
    buff[3] = step;
    
    bint16_Union.U16 = icarStr.errorCode;
    buff[4] = bint16_Union.U8_Buff[0];
    buff[5] = bint16_Union.U8_Buff[1];

    for(int i=0;i<6;i++)
        check += buff[i];
    
    buff[6] = check;
    
    for(int i=0;i<9;i++)
        USB_Inspector_TransmitByte(buff[i]);
}
//------------------------------------------------[END]-------------------------------------------------------------





#include "iic_drv.h"
#include "stm32f0xx_i2c.h"

/*******************************************************************************
* Function Name : I2C_HardwareInit
* Date          : 2025/02/21
* Creator       : shj       
* Description   : IIC�˿ڳ�ʼ��
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void I2C_HardwareInit(void)
{
    GPIO_InitTypeDef Struct_GPIOInit;
//    I2C_InitTypeDef I2C_InitStruct;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);   
//    RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE); 
    
    Struct_GPIOInit.GPIO_Pin = IIC_SCL | IIC_SDA;
    Struct_GPIOInit.GPIO_Mode = GPIO_Mode_OUT;//GPIO_Mode_AF;
    Struct_GPIOInit.GPIO_Speed = GPIO_Speed_Level_3;
    Struct_GPIOInit.GPIO_OType = GPIO_OType_PP;//GPIO_OType_OD;//��©
    Struct_GPIOInit.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(IIC_PORT,&Struct_GPIOInit);
}

/*******************************************************************************
* Function Name : delay_us
* Date          : 2025/02/21
* Creator       : shj       
* Description   : us��ʱ
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void delay_us(uint8_t us)
{
    #define COUNT_NUM 2
    uint8_t i = 0;
    uint16_t delay_cnt = COUNT_NUM;
    for(i=0; i < us; i++)
    {
        while(delay_cnt--);
        delay_cnt = COUNT_NUM;
    }
}

/*******************************************************************************
* Function Name : I2C_Start
* Date          : 2025/02/21
* Creator       : shj     
* Description   : ģ����ʼλ  
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void I2C_Start(void)
{
    IIC_SDA_OUT;
	IIC_SCL_HIGH();//������ʱ����
	IIC_SDA_HIGH();//������������

	delay_us(2);//�ӳ�2us���ӳ����б�Ҫ�ģ�����I2CЭ��д��
	IIC_SDA_LOW();//���������ߣ�����ͨѶ

	delay_us(2);//�ӳ�2us
	IIC_SCL_LOW();//����ʱ���ߣ������������ϵ����ݱ仯
//	delay_us(2);//�ӳ�2us
}

/*******************************************************************************
* Function Name : I2C_Stop
* Date          : 2025/02/21
* Creator       : shj       
* Description   : ģ��ֹͣλ
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void I2C_Stop(void)
{
    IIC_SDA_OUT;
    IIC_SDA_LOW();
    IIC_SCL_HIGH();

	delay_us(2);
	
    IIC_SDA_HIGH();
}

/*******************************************************************************
* Function Name : I2C_WaitAck
* Date          : 2025/02/21
* Creator       : shj       
* Description   : IIC�ȴ��ӻ���Ӧ
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
uint8_t I2C_WaitAck(void)
{
    uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA����Ϊ����  
	IIC_SDA_HIGH();
	delay_us(2);	   
	IIC_SCL_HIGH();
	delay_us(5);	 
	while(IIC_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			I2C_Stop();
			return 1;
		}
	}
	IIC_SCL_LOW();//ʱ�����0 	   
	return 0;
}

//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL_LOW();
	IIC_SDA_OUT;
	IIC_SDA_LOW();
	delay_us(5);
	IIC_SCL_HIGH();
	delay_us(5);
	IIC_SCL_LOW();
}

//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL_LOW();
	IIC_SDA_OUT;
	IIC_SDA_HIGH();
	delay_us(5);
	IIC_SCL_HIGH();
	delay_us(5);
	IIC_SCL_LOW();
}	

/*******************************************************************************
* Function Name : I2C_SendByte
* Date          : 2025/02/21
* Creator       : shj       
* Description   : IIC���͵��ֽ�����
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 0;
    IIC_SDA_OUT;
    IIC_SCL_LOW();
    for(i = 0; i < 8; i++)
    {
        if (byte & 0x80)
        {
            IIC_SDA_HIGH();
        }
        else
        {
            IIC_SDA_LOW();
        }
        byte <<= 1;
        delay_us(2);
        IIC_SCL_HIGH();
        delay_us(2);
        IIC_SCL_LOW();
        delay_us(2);
    }    
}

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t I2C_ReadByte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA����Ϊ����
	IIC_SCL_LOW();
	delay_us(5);
	for(i=0;i<8;i++ )
	{
		IIC_SCL_HIGH();//����ʱ��������[������],�ôӻ�׼��������
		delay_us(5);
		receive<<=1;
		if(IIC_SDA_STATE)
		{
			receive|=0x01; 
		}  
		IIC_SCL_LOW();//׼�����ٴν�������
		delay_us(5);//�ȴ�����׼���� 
	}					 
	if (!ack)
	{
		IIC_NAck();//����nACK
	}
	else
	{
		IIC_Ack(); //����ACK   
	}
	return receive;
}

unsigned char I2C_ByteWrite(uint8_t device_addr,uint8_t REG_Address,uint8_t REG_data)
{
    I2C_Start();                  //��ʼ�ź�
    
    I2C_SendByte(device_addr);   //�����豸��ַ+д�ź�
    I2C_WaitAck();	
    I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ�� //��ο�����pdf22ҳ
    I2C_WaitAck();	
    I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ� //��ο�����pdf22ҳ
    I2C_WaitAck();	
    I2C_Stop();                   //����ֹͣ�ź�
    return 1;
}

/*============================================================================
  ��������    :  
  ��������    :  
  �������    :
  ����ֵ      :
  ˵��        :      
=============================================================================*/
uint8_t I2C_ByteRead(uint8_t device_addr,uint8_t REG_Address,uint8_t* REG_data)
{

    I2C_Start();                          //��ʼ�ź�
    I2C_SendByte(device_addr);           //�����豸��ַ+д�ź�
    I2C_WaitAck(); 
    I2C_SendByte(REG_Address);            //���ʹ洢��Ԫ��ַ��//��0��ʼ
    I2C_WaitAck(); 
    I2C_Start();                          //��ʼ�ź�
    I2C_SendByte(device_addr+1);         //�����豸��ַ+���ź�
    I2C_WaitAck(); 
    *REG_data = I2C_ReadByte(0);	//�����Ĵ�������
    I2C_Stop();                           //ֹͣ�ź�
    return 0x01;
}

/*============================================================================
  ��������    :  
  ��������    :  
  �������    :
  ����ֵ      :
  ˵��        :      
=============================================================================*/
uint8_t I2C_BytesRead(uint8_t device_addr,uint8_t REG_Address,uint8_t* REG_data,uint8_t len)
{
    unsigned char i;
    if(!len)
        return 0;
    
    I2C_Start(); 
    
    I2C_SendByte(device_addr);//�����豸��ַ+д�ź�
    I2C_WaitAck();
    
    I2C_SendByte(REG_Address);  //���ʹ洢��Ԫ��ַ
    I2C_WaitAck();
    
    I2C_Start(); 
    
    I2C_SendByte(device_addr+1);//�����豸��ַ+���ź�
    I2C_WaitAck();
    
    for(i=0;i<(len-1);i++)
    {
        *REG_data = I2C_ReadByte(1);
        REG_data++;
    }
    *REG_data = I2C_ReadByte(0);  
    I2C_Stop();  
    return 1;
}





#include "iic_drv.h"
#include "stm32f0xx_i2c.h"

/*******************************************************************************
* Function Name : I2C_HardwareInit
* Date          : 2025/02/21
* Creator       : shj       
* Description   : IIC端口初始化
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
    Struct_GPIOInit.GPIO_OType = GPIO_OType_PP;//GPIO_OType_OD;//开漏
    Struct_GPIOInit.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(IIC_PORT,&Struct_GPIOInit);
}

/*******************************************************************************
* Function Name : delay_us
* Date          : 2025/02/21
* Creator       : shj       
* Description   : us延时
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
* Description   : 模拟起始位  
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void I2C_Start(void)
{
    IIC_SDA_OUT;
	IIC_SCL_HIGH();//先拉高时钟线
	IIC_SDA_HIGH();//再拉高数据线

	delay_us(2);//延迟2us，延迟是有必要的，参照I2C协议写的
	IIC_SDA_LOW();//拉低数据线，触发通讯

	delay_us(2);//延迟2us
	IIC_SCL_LOW();//拉低时钟线，方便数据线上的数据变化
//	delay_us(2);//延迟2us
}

/*******************************************************************************
* Function Name : I2C_Stop
* Date          : 2025/02/21
* Creator       : shj       
* Description   : 模拟停止位
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
* Description   : IIC等待从机响应
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
uint8_t I2C_WaitAck(void)
{
    uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA设置为输入  
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
	IIC_SCL_LOW();//时钟输出0 	   
	return 0;
}

//产生ACK应答
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

//不产生ACK应答		    
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
* Description   : IIC发送单字节数据
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

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t I2C_ReadByte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA设置为输入
	IIC_SCL_LOW();
	delay_us(5);
	for(i=0;i<8;i++ )
	{
		IIC_SCL_HIGH();//产生时钟上升沿[正脉冲],让从机准备好数据
		delay_us(5);
		receive<<=1;
		if(IIC_SDA_STATE)
		{
			receive|=0x01; 
		}  
		IIC_SCL_LOW();//准备好再次接收数据
		delay_us(5);//等待数据准备好 
	}					 
	if (!ack)
	{
		IIC_NAck();//发送nACK
	}
	else
	{
		IIC_Ack(); //发送ACK   
	}
	return receive;
}

unsigned char I2C_ByteWrite(uint8_t device_addr,uint8_t REG_Address,uint8_t REG_data)
{
    I2C_Start();                  //起始信号
    
    I2C_SendByte(device_addr);   //发送设备地址+写信号
    I2C_WaitAck();	
    I2C_SendByte(REG_Address);    //内部寄存器地址， //请参考中文pdf22页
    I2C_WaitAck();	
    I2C_SendByte(REG_data);       //内部寄存器数据， //请参考中文pdf22页
    I2C_WaitAck();	
    I2C_Stop();                   //发送停止信号
    return 1;
}

/*============================================================================
  函数名称    :  
  函数功能    :  
  输入参数    :
  返回值      :
  说明        :      
=============================================================================*/
uint8_t I2C_ByteRead(uint8_t device_addr,uint8_t REG_Address,uint8_t* REG_data)
{

    I2C_Start();                          //起始信号
    I2C_SendByte(device_addr);           //发送设备地址+写信号
    I2C_WaitAck(); 
    I2C_SendByte(REG_Address);            //发送存储单元地址，//从0开始
    I2C_WaitAck(); 
    I2C_Start();                          //起始信号
    I2C_SendByte(device_addr+1);         //发送设备地址+读信号
    I2C_WaitAck(); 
    *REG_data = I2C_ReadByte(0);	//读出寄存器数据
    I2C_Stop();                           //停止信号
    return 0x01;
}

/*============================================================================
  函数名称    :  
  函数功能    :  
  输入参数    :
  返回值      :
  说明        :      
=============================================================================*/
uint8_t I2C_BytesRead(uint8_t device_addr,uint8_t REG_Address,uint8_t* REG_data,uint8_t len)
{
    unsigned char i;
    if(!len)
        return 0;
    
    I2C_Start(); 
    
    I2C_SendByte(device_addr);//发送设备地址+写信号
    I2C_WaitAck();
    
    I2C_SendByte(REG_Address);  //发送存储单元地址
    I2C_WaitAck();
    
    I2C_Start(); 
    
    I2C_SendByte(device_addr+1);//发送设备地址+读信号
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





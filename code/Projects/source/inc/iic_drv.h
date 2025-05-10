#ifndef __IIC_DRV_H__
#define __IIC_DRV_H__
#include "stm32f0xx_gpio.h"

#define IIC_PORT GPIOA
#define IIC_SCL GPIO_Pin_7
#define IIC_SDA GPIO_Pin_9

#define IIC_SCL_HIGH() GPIO_SetBits(IIC_PORT, IIC_SCL)
#define IIC_SCL_LOW()  GPIO_ResetBits(IIC_PORT, IIC_SCL)
#define IIC_SDA_HIGH() GPIO_SetBits(IIC_PORT, IIC_SDA)
#define IIC_SDA_LOW()  GPIO_ResetBits(IIC_PORT, IIC_SDA)

#define IIC_SDA_OUT {IIC_PORT->MODER &= 0xfff3ffff;IIC_PORT->MODER |= (1<<18);}
#define IIC_SDA_IN  {IIC_PORT->MODER &= 0xfff3ffff;}

#define IIC_SDA_STATE (GPIO_ReadInputDataBit(IIC_PORT, IIC_SDA))

void I2C_Start(void);
void I2C_Stop(void);
uint8_t I2C_WaitAck(void);
void I2C_SendByte(uint8_t byte);
extern uint8_t I2C_ReadByte(unsigned char ack);
extern unsigned char I2C_ByteWrite(uint8_t device_addr,uint8_t REG_Address,uint8_t REG_data);
extern uint8_t I2C_ByteRead(uint8_t device_addr,uint8_t REG_Address,uint8_t* REG_data);
extern uint8_t I2C_BytesRead(uint8_t device_addr,uint8_t REG_Address,uint8_t* REG_data,uint8_t len);
void I2C_HardwareInit(void);

#endif

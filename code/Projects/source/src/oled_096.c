#include "oled_096.h"
#include "oled_font.h"
#include "ICM42670P.h"
#include "stdlib.h"

OLED_MENU_ menu = SHOW_GYRO;

/*******************************************************************************
* Function Name : OLED_WriteByte
* Date          : 2025/02/21
* Creator       : shj       
* Description   : 发送指令或数据
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_WriteByte(uint8_t dat, uint8_t mode)
{
    I2C_Start();
    I2C_SendByte(0x78);
    I2C_WaitAck();
    if (mode)
    {
        I2C_SendByte(0x40);
    }
    else
    {
        I2C_SendByte(0x00);
    }
    I2C_WaitAck();
    I2C_SendByte(dat);
    I2C_WaitAck();
    I2C_Stop();
}

/*******************************************************************************
* Function Name : OLED_SetCursor
* Date          : 2025/02/21
* Creator       : shj       
* Description   : 页地址模式，设定OLED起始光标位置
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteByte(0xB0 | Y, OLED_CMD);             //设置Y位置
    OLED_WriteByte(0x10 | (X & 0x0F), OLED_CMD);    //设置X位置高四位
    OLED_WriteByte(0x00 | (X & 0x0F), OLED_CMD);    //设置X位置低四位
}

/*******************************************************************************
* Function Name : OLED_SetColumnAddr
* Date          : 2025/02/21
* Creator       : shj       
* Description   : 水平/垂直地址模式，设定列开始和结束地址
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_SetColumnAddr(uint8_t start_addr, uint8_t end_addr)
{
    OLED_WriteByte(0x21, OLED_CMD);             
    OLED_WriteByte(start_addr, OLED_CMD);    
    OLED_WriteByte(end_addr, OLED_CMD);    
}
/*******************************************************************************
* Function Name : OLED_SetPageAddr
* Date          : 2025/02/21
* Creator       : shj       
* Description   : 水平/垂直地址模式，设定页开始和结束地址
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_SetPageAddr(uint8_t start_addr, uint8_t end_addr)
{
    OLED_WriteByte(0x22, OLED_CMD);             
    OLED_WriteByte(start_addr, OLED_CMD);    
    OLED_WriteByte(end_addr, OLED_CMD);    
}

/*******************************************************************************
* Function Name : OLED_Clear
* Date          : 2025/02/21
* Creator       : shj       
* Description   : 清除显示屏内容
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_Clear(void)
{
//    uint8_t i = 0, j = 0;
//	for (j = 0; j < 8; j++) 
//    {
//        /*0~7页*/
//		OLED_SetCursor(j, i);//选择起始光点
//		for(i = 0; i < 128; i++) 
//        {
//            /*0~127列*/
//			OLED_WriteByte(0x00, OLED_DATA);//写入数据
//		}
//	}   
  uint16_t i = 0;
  OLED_SetColumnAddr(0x00, 0x7f);
  OLED_SetPageAddr(0x00, 0x07);
  for (i = 0; i < 1024; i++)
  {
    OLED_WriteByte(0x00, OLED_DATA);
  }
}

/*******************************************************************************
* Function Name : OLED_Init
* Date          : 2025/02/21
* Creator       : shj       
* Description   : OLED屏初始化
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_Init(void)
{
    OLED_WriteByte(0xae, OLED_CMD);//关显示
    
    OLED_WriteByte(0x00, OLED_CMD);//设置低列地址
    OLED_WriteByte(0x10, OLED_CMD);//设置高列地址
    
    OLED_WriteByte(0x40, OLED_CMD);//设定GDDRAM起始行地址；40：第0行；7f：第63行
    
    OLED_WriteByte(0xa4, OLED_CMD);//全局显示开启；a4：正常；a5：无视命令全屏点亮
    
    OLED_WriteByte(0x81, OLED_CMD);//对比度设置（亮度）
    OLED_WriteByte(0xcf, OLED_CMD);//对比度值
    
    OLED_WriteByte(0xa1, OLED_CMD);//设置列扫描方向；a1:正常；a0：左右颠倒    
    OLED_WriteByte(0xc8, OLED_CMD);//设置行扫描方向；C8:正常；C0：上下颠倒
    
    OLED_WriteByte(0xa6, OLED_CMD);//设置颜色显示方式；a6:正常显示;a7:逆向显示
    
    OLED_WriteByte(0xa8, OLED_CMD);//设置多路复用率
    OLED_WriteByte(0x3f, OLED_CMD);//1/64多路复用
    
    OLED_WriteByte(0xd3, OLED_CMD);//设置显示偏移
    OLED_WriteByte(0x00, OLED_CMD);//无偏移
    
    OLED_WriteByte(0xd5, OLED_CMD);//设置显示时钟分频
    OLED_WriteByte(0x80, OLED_CMD);//推荐之
    
    OLED_WriteByte(0xd9, OLED_CMD);//设置预充电周期
    OLED_WriteByte(0xf1, OLED_CMD);//推荐之
    
    OLED_WriteByte(0xda, OLED_CMD);//设置COM引脚硬件配置
    OLED_WriteByte(0x12, OLED_CMD);//备用配置
    
    OLED_WriteByte(0xdb, OLED_CMD);//设定Vcomh
    OLED_WriteByte(0x40, OLED_CMD);//推荐值
    
    OLED_WriteByte(0x20, OLED_CMD);//设置内存寻址模式    
    OLED_WriteByte(0x00, OLED_CMD);//水平寻址模式
    
    OLED_WriteByte(0x8d, OLED_CMD);//设置电荷泵
    OLED_WriteByte(0x14, OLED_CMD);//启动电荷泵
    
    OLED_Clear();                  //清屏
    OLED_WriteByte(0xaf, OLED_CMD);//开显示
//    OLED_WriteByte(0xa5, OLED_CMD);//全屏点亮
}
/*******************************************************************************
* Function Name : Gyro_Menu_Show
* Date          : 2025/02/21
* Creator       : shj       
* Description   : Gyro 参数菜单显示
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void Gyro_Menu_Show(void)
{
    uint8_t i = 0, j = 0;
    OLED_SetColumnAddr(0x00, 0x37);
      OLED_SetPageAddr(0x00,0x01);
      for (i = 0; i < 7; i++)
      {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Gyrox_DataBuffer[i][j], OLED_DATA);
        }
      }
      for (i = 0; i < 7; i++)
      {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Gyrox_DataBuffer[i][j], OLED_DATA);
        }
      }
    OLED_SetColumnAddr(0x00, 0x37);
    OLED_SetPageAddr(0x02,0x03);
    for (i = 0; i < 7; i++)
    {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Gyroy_DataBuffer[i][j], OLED_DATA);
        }
    }
    for (i = 0; i < 7; i++)
    {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Gyroy_DataBuffer[i][j], OLED_DATA);
        }
    } 
    OLED_SetColumnAddr(0x00, 0x37);
    OLED_SetPageAddr(0x04,0x05);
    for (i = 0; i < 7; i++)
    {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Gyroz_DataBuffer[i][j], OLED_DATA);
        }
    }
    for (i = 0; i < 7; i++)
    {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Gyroz_DataBuffer[i][j], OLED_DATA);
        }
    }     
}
/*******************************************************************************
* Function Name : Accel_Menu_Show
* Date          : 2025/02/21
* Creator       : shj       
* Description   : Accel 参数菜单显示
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void Accel_Menu_Show(void)
{
    uint8_t i = 0, j = 0;
    OLED_SetColumnAddr(0x00, 0x3f);
    OLED_SetPageAddr(0x00,0x01);
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Accelx_DataBuffer[i][j], OLED_DATA);
        }
    }
    for (i = 0; i < 8; i++)
    {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Accelx_DataBuffer[i][j], OLED_DATA);
        }
    }
    OLED_SetColumnAddr(0x00, 0x3f);
    OLED_SetPageAddr(0x02,0x03);
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Accely_DataBuffer[i][j], OLED_DATA);
        }
    }
    for (i = 0; i < 8; i++)
    {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Accely_DataBuffer[i][j], OLED_DATA);
        }
    } 
    OLED_SetColumnAddr(0x00, 0x3f);
    OLED_SetPageAddr(0x04,0x05);
    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Accelz_DataBuffer[i][j], OLED_DATA);
        }
    }
    for (i = 0; i < 8; i++)
    {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Accelz_DataBuffer[i][j], OLED_DATA);
        }
    }    
}

/*******************************************************************************
* Function Name : Angel_Menu_Show
* Date          : 2025/02/21
* Creator       : shj       
* Description   : 角度 参数菜单显示
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void Angel_Menu_Show(void)
{
uint8_t i = 0, j = 0;
    OLED_SetColumnAddr(0x00, 0x5f);
    OLED_SetPageAddr(0x00,0x01);
    for (i = 0; i < 12; i++)
    {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Angle_Menu[i][j], OLED_DATA);
        }
    }
    for (i = 0; i < 12; i++)
    {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Angle_Menu[i][j], OLED_DATA);
        }
    }
    OLED_SetColumnAddr(0x00, 0x57);
    OLED_SetPageAddr(0x02,0x03);
    for (i = 0; i < 11; i++)
    {
        for (j = 0; j < 8; j++)
        {
            OLED_WriteByte(Roll_Menu[i][j], OLED_DATA);
        }
    }
    for (i = 0; i < 11; i++)
    {
        for (j = 8; j < 16; j++)
        {
            OLED_WriteByte(Roll_Menu[i][j], OLED_DATA);
        }
    }     
}

/*******************************************************************************
* Function Name : Test_OLEDShow
* Date          : 2025/02/21
* Creator       : shj    
* Description   : OLED显示测试   
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void Test_OLEDShow(void)
{
    uint8_t i = 0, j = 0;
    uint8_t ge = 0, shi = 0, bai = 0, qian = 0, wan = 0;
    uint32_t gyro_val = 0, accel_val = 0;
    switch(menu)
    {
    case SHOW_GYRO:     
      
      Gyro_Menu_Show();
      break;
    case SHOW_ACCEL:
        Accel_Menu_Show();
      break;
    case SHOW_PITCH:      
      Angel_Menu_Show();
      break;
    default:break;
    }
	 
}


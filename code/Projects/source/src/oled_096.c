#include "oled_096.h"
#include "oled_font.h"
#include "ICM42670P.h"
#include "stdlib.h"

OLED_MENU_ menu = SHOW_GYRO;

/*******************************************************************************
* Function Name : OLED_WriteByte
* Date          : 2025/02/21
* Creator       : shj       
* Description   : ����ָ�������
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
* Description   : �趨OLED��ʼ���λ��
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteByte(0xB0 | Y, OLED_CMD);             //����Yλ��
    OLED_WriteByte(0x10 | (X & 0xF0), OLED_CMD);    //����Xλ�ø���λ
    OLED_WriteByte(0x00 | (X & 0x0F), OLED_CMD);    //����Xλ�õ���λ
}

/*******************************************************************************
* Function Name : OLED_Clear
* Date          : 2025/02/21
* Creator       : shj       
* Description   : �����ʾ������
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_Clear(void)
{
    uint8_t i = 0, j = 0;
	for (j = 0; j < 8; j++) 
    {
        /*0~7ҳ*/
		OLED_SetCursor(j, i);//ѡ����ʼ���
		for(i = 0; i < 128; i++) 
        {
            /*0~127��*/
			OLED_WriteByte(0x00, OLED_DATA);//д������
		}
	}    
}

/*******************************************************************************
* Function Name : OLED_Init
* Date          : 2025/02/21
* Creator       : shj       
* Description   : OLED����ʼ��
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void OLED_Init(void)
{
    OLED_WriteByte(0xae, OLED_CMD);//����ʾ
    
    OLED_WriteByte(0x00, OLED_CMD);//���õ��е�ַ
    OLED_WriteByte(0x10, OLED_CMD);//���ø��е�ַ
    
    OLED_WriteByte(0x40, OLED_CMD);//�趨GDDRAM��ʼ�е�ַ��40����0�У�7f����63��
    
    OLED_WriteByte(0xa4, OLED_CMD);//ȫ����ʾ������a4��������a5����������ȫ������
    
    OLED_WriteByte(0x81, OLED_CMD);//�Աȶ����ã����ȣ�
    OLED_WriteByte(0xcf, OLED_CMD);//�Աȶ�ֵ
    
    OLED_WriteByte(0xa1, OLED_CMD);//������ɨ�跽��a1:������a0�����ҵߵ�    
    OLED_WriteByte(0xc8, OLED_CMD);//������ɨ�跽��C8:������C0�����µߵ�
    
    OLED_WriteByte(0xa6, OLED_CMD);//������ɫ��ʾ��ʽ��a6:������ʾ;a7:������ʾ
    
    OLED_WriteByte(0xa8, OLED_CMD);//���ö�·������
    OLED_WriteByte(0x3f, OLED_CMD);//1/64��·����
    
    OLED_WriteByte(0xd3, OLED_CMD);//������ʾƫ��
    OLED_WriteByte(0x00, OLED_CMD);//��ƫ��
    
    OLED_WriteByte(0xd5, OLED_CMD);//������ʾʱ�ӷ�Ƶ
    OLED_WriteByte(0x80, OLED_CMD);//�Ƽ�֮
    
    OLED_WriteByte(0xd9, OLED_CMD);//����Ԥ�������
    OLED_WriteByte(0xf1, OLED_CMD);//�Ƽ�֮
    
    OLED_WriteByte(0xda, OLED_CMD);//����COM����Ӳ������
    OLED_WriteByte(0x12, OLED_CMD);//��������
    
    OLED_WriteByte(0xdb, OLED_CMD);//�趨Vcomh
    OLED_WriteByte(0x40, OLED_CMD);//�Ƽ�ֵ
    
    OLED_WriteByte(0x20, OLED_CMD);//�����ڴ�Ѱַģʽ    
    OLED_WriteByte(0x02, OLED_CMD);//pageѰַģʽ
    
    OLED_WriteByte(0x8d, OLED_CMD);//���õ�ɱ�
    OLED_WriteByte(0x14, OLED_CMD);//������ɱ�
    
    OLED_Clear();                  //����
    OLED_WriteByte(0xaf, OLED_CMD);//����ʾ
//    OLED_WriteByte(0xa5, OLED_CMD);//ȫ������
}

/*******************************************************************************
* Function Name : Test_OLEDShow
* Date          : 2025/02/21
* Creator       : shj    
* Description   : OLED��ʾ����   
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
      
      for (j = 0; j < 6; j++) 
      {
          /*0~7ҳ*/
          i = 0;
		  OLED_SetCursor(j, i);//ѡ����ʼ���
		  for(i = 0; i < 56; i++) 
          {
              /*0~127��*/
			  OLED_WriteByte(Gyro_Menu[j][i], OLED_DATA);//д������
		  }
	  }
      
      
      break;
    case SHOW_ACCEL:
      for (j = 0; j < 6; j++) 
      {
          /*0~7ҳ*/
          i = 0;
		  OLED_SetCursor(j, i);//ѡ����ʼ���
		  for(i = 0; i < 64; i++) 
          {
              /*0~127��*/
			  OLED_WriteByte(Accel_Menu[j][i], OLED_DATA);//д������
		  }
	  }
      break;
    case SHOW_PITCH:      
      ge = abs(scooter_gyro.angle_pitch)%10;
      shi = abs(scooter_gyro.angle_pitch)/10;
      for (j = 0; j < 2; j++) 
      {
          /*0~7ҳ*/
          i = 0;
		  OLED_SetCursor(j, i);//ѡ����ʼ���
		  for (i = 0; i < 96; i++) 
          {
              /*0~127��*/
			  OLED_WriteByte(Angle_Menu[j][i], OLED_DATA);//д������
		  }
          /*��������*/
          if (j == 0)
          {                     
            /*������*/
              if (scooter_gyro.angle_pitch >= 0)
              {
                  for (i = 0; i < 8; i++)
                  {
                      OLED_WriteByte(Direction_Buffer[0][i], OLED_DATA);
                  }
              }
              else
              {
                  for (i = 0; i < 8; i++)
                  {
                      OLED_WriteByte(Direction_Buffer[1][i], OLED_DATA);
                  }
              }
              /*�Ƕȣ�ʮλ*/
              for (i = 0; i < 8; i++)
              {
                  OLED_WriteByte(Number_Buffer[shi][i], OLED_DATA);
              }  
              /*�Ƕȣ���λ*/
              for (i = 0; i < 8; i++)
              {
                  OLED_WriteByte(Number_Buffer[ge][i], OLED_DATA);
              } 
          }
          if (j == 1)
          {
              if (scooter_gyro.angle_pitch >= 0)
              {
                  for (i = 8; i < 16; i++)
                  {
                      OLED_WriteByte(Direction_Buffer[0][i], OLED_DATA);
                  }
              }
              else
              {
                  for (i = 8; i < 16; i++)
                  {
                      OLED_WriteByte(Direction_Buffer[1][i], OLED_DATA);
                  }
              }
              for (i = 8; i < 16; i++)
              {
                  OLED_WriteByte(Number_Buffer[shi][i], OLED_DATA);
              }
              for (i = 8; i < 16; i++)
              {
                  OLED_WriteByte(Number_Buffer[ge][i], OLED_DATA);
              }
          }
	  }
      break;
    default:break;
    }
	 
}


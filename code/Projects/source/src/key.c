#include "key.h"
#include "ws2812_app.h"
#include "oled_096.h"

key_struct__ key_struct={0};

/****************************************************************************** 
* Function Name    	: KeyGpio_Init
* Modify/Creat time	: 2024/08/02
* Modified/Creat By	: shj
* Description    	: device key gpio init
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void KeyGpio_Init(void)
{
    GPIO_InitTypeDef Struct_GPIOInit;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
    
//    GPIO_DeInit(KEY_PORT);
    Struct_GPIOInit.GPIO_Pin = KEY_GPIO_PIN;
    Struct_GPIOInit.GPIO_Mode = GPIO_Mode_IN;
    //Struct_GPIOInit.GPIO_PuPd = GPIO_PuPd_DOWN;
    Struct_GPIOInit.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_Init(KEY_PORT,&Struct_GPIOInit);
}

/****************************************************************************** 
* Function Name    	: CheckKey
* Modify/Creat time	: 2024/08/02
* Modified/Creat By	: shj
* Description    	: device key gpio input state check
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static uint16_t CheckKey(void)
{
    uint8_t key_val=0;
    if (GET_KEY_STA())          //获取按键GPIO电平状态
    {
	key_val=1;              //按键电平有效
    }
    return key_val;             //返回按键状态
}

/****************************************************************************** 
* Function Name    	: KeyState_Process
* Modify/Creat time	: 2024/08/02
* Modified/Creat By	: shj
* Description    	: deal with the key gpio input state information
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void KeyState_Process(key_struct__* key_param)
{
    if (key_param->keysta_current)                                      //当前案件信息有效处理
    {
        if (key_param->keysta_previous != key_param->keysta_current)    //对比之前的按键状态
        {
            key_param->keysta_previous = key_param->keysta_current;     //保存当前按键状态
        }
        if (key_param->time_cnt < (0xffff - \
          (KEY_PROCESS_STEP * KEY_PROCESS_CYCLE)))                      //按键有效时间判断
        {
            key_param->time_cnt += KEY_PROCESS_STEP * KEY_PROCESS_CYCLE;//计算按键有效时间
        }
    }
    else                                                                //按键松开状态逻辑处理
    {
        if (key_param->keysta_previous)                                 //检测到之前的按键有效
        {
            key_param->keysta_previous = 0;                             //清除之前按键状态
            if (key_param->time_cnt < 1000 && key_param->time_cnt >100) //按键按下时间在200~1000ms
            {
                key_param->key_longshort_chk = 1;                       //标志按键短按
            }
            else if (key_param->time_cnt > 2000)                        //按键按下时间超过2s
            { 
                key_param->key_longshort_chk = 2;                       //标志按键长按
            }
            else                                                        //噪声/误操作
            {
                key_param->key_longshort_chk = 0;                       //没有按键操作
            }
            key_param->time_cnt = 0;                                    //清除按键操作时间
        }
    }
}

/****************************************************************************** 
* Function Name    	: KeyState_Check
* Modify/Creat time	: 2024/08/02
* Modified/Creat By	: shj
* Description    	: check the key state ,up or down, and process it 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void KeyState_Check(key_struct__* key_param)
{
    uint16_t temp;
    #define CNT_TIMES 10
    
    if (key_param->sample_cnt < CNT_TIMES)              //按键信息滤波（时间10*时基）
    {		
	temp = CheckKey();                              //获取按键状态
	if (temp)                                       //检测到按键按下
	{
            key_param->key_filter_cnt++;                //按键按下次数累积
	}
	key_param->sample_cnt++;                        //采样次数累积
    }
    else                                                //滤波收按键数据处理
    {
	if (key_param->key_filter_cnt >= CNT_TIMES/3)   //有效数据判断
	{
            key_param->keysta_current = 1;		//按键有效	
	}
        else
        {
            key_param->keysta_current = 0;
        }
	key_param->key_filter_cnt = 0;	                //滤波次数清零
	key_param->key_process_flg = 1;                 //按键信息处理标志
        key_param->sample_cnt = 0;
    }
    
    if (key_param->key_process_flg)                     //案件信息处理接口
    {
        key_param->key_process_flg = 0;                 //按键信息处理完后清除处理标志
        KeyState_Process(key_param);                    //处理案件信息
    }
}

/****************************************************************************** 
* Function Name    	: KeyInfo_Process
* Modify/Creat time	: 2024/08/02
* Modified/Creat By	: shj
* Description    	: key state function remap
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void KeyInfo_Process(key_struct__* key_param)
{
    switch (key_param->key_longshort_chk)
    {
    case IDLE:
      
      break;;
    case SHORT_PRESS:
      OLED_Clear(); 
      key_param->key_longshort_chk = IDLE;
      if (menu == SHOW_GYRO)
      {
        menu = SHOW_ACCEL;
      }
      else if(menu == SHOW_ACCEL)
      {
        menu = SHOW_PITCH;
      }
      else
      {
        menu = SHOW_GYRO;
      }
      break;
    case LONG_PRESS:
      
      break;
    default:break;
    }
}

/****************************************************************************** 
* Function Name    	: HardwareInit_Key
* Modify/Creat time	: 2024/08/02
* Modified/Creat By	: shj
* Description    	: key hardware init
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void HardwareInit_Key(void)
{
    KeyGpio_Init();
}

/****************************************************************************** 
* Function Name    	: task_Key
* Modify/Creat time	: 2024/08/02
* Modified/Creat By	: shj
* Description    	: key task
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void task_Key(void)
{
    KeyState_Check(&key_struct);
    KeyInfo_Process(&key_struct);
}







#ifndef __KEY_H__
#define __KEY_H__
#include "stm32f0xx.h"

typedef enum
{
    IDLE=0,
    SHORT_PRESS,
    LONG_PRESS
}enum_KeySta_;

#define KEY_PORT GPIOA
#define KEY_GPIO_PIN GPIO_Pin_5

#define GET_KEY_STA() GPIO_ReadInputDataBit(KEY_PORT,KEY_GPIO_PIN)

#define KEY_PROCESS_STEP 10      //按键任务执行次数计数
#define KEY_PROCESS_CYCLE 1    //按键任务执行周期，单位:ms

typedef struct
{
    uint8_t sample_cnt;                 //总采样次数 
    uint8_t key_filter_cnt;             //有效按键次数
    uint8_t keysta_current;             //当前按键状态
    uint8_t keysta_previous;            //上次按键状态	
    uint8_t key_longshort_chk;
    uint8_t key_process_flg;            //按键信号处理标志；1：使能判断；0：等待判断
    uint8_t shortpress_cnt;             //按键短按次数
    uint16_t time_cnt;
}key_struct__;
extern key_struct__ key_struct;

extern void HardwareInit_Key(void);
extern void task_Key(void);
#endif

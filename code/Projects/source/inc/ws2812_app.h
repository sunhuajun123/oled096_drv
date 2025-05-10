#ifndef __WS2812_APP_H__
#define __WS2812_APP_H__
#include "ws2812_drv.h"
#include "main.h"

#define RGB_LED_NUM 10

#define  FRONT_RGB_LED_NUM	(10ul)
#define RGB_MAX_ELEMENT_NUM     40
#define RGB_PATTERN_LINEAR      0x01
#define RGB_PATTERN_ACCEL       0x02
#define RGB_PATTERN_DECEL       0x03
#define RGB_PATTERN_ACCEL_DECEL 0x04
#define RGB_PATTERN_DECEL_ACCEL 0x05
#define rgb_getTime() (sys_TimeGet())
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
}RGB;

typedef struct{
    uint8_t id;                 //显示元素的ID
    uint32_t start_time;        //时间(单位：ms)
    uint32_t end_time;
    RGB start_color;            //颜色
    RGB end_color;
    uint8_t start_position;     //位置
    uint8_t end_position;
    uint8_t start_brightness;   //亮度
    uint8_t end_brightness;
    uint8_t color_pattern;      //颜色渐变方式
    uint8_t position_pattern;   //位置渐变方式
    uint8_t brightness_pattern; //亮度渐变方式       
}RGB_Element_Typedef;

typedef struct{
    uint8_t id;
    uint32_t total_time;        //显示周期
    uint8_t element_num;        //动画元素数量
    uint8_t brightness;         //总亮度
}RGB_STLFileInfo_Typedef;

typedef struct
{
  uint8_t ch;
  uint8_t rgb_led_num;//此通道控制的RGB灯个数 
  int8_t light_set_stl; //要设置的模式
  uint32_t rgb_time;      
  uint32_t rgb_timestamp;
  uint32_t rgb_timeold;
  RGB light_set_color[8];   //要显示的颜色
  int8_t last_stl_id;    //上次动画的ID
  uint8_t update_flag;  
 // uint8_t logic_addr[RGB_LED_NUM];//此通道控制的RGB逻辑地址 = {0,1,2,3,4,5,6,7,8,9};
  RGB_STLFileInfo_Typedef current_stl;//当前显示的stl	    
  RGB Disp_RGB_Tab[FRONT_RGB_LED_NUM];//底层显示缓存
  RGB rgbs[FRONT_RGB_LED_NUM];        //应用层显示缓存
  RGB_Element_Typedef elements[RGB_MAX_ELEMENT_NUM];//显示元素列表 
  uint8_t RGB_TIM_CCR_OUT_BUF[FRONT_RGB_LED_NUM * 24+1];//DMA刷新缓存
}RGB_Controller;

typedef struct
{
  RGB light_blue;
  RGB light_green;
  RGB orange;
  RGB test_color;
  RGB light_clear;
}RGB_Color_Struct_;

extern RGB_Controller Front_RGB;
extern void bsp_ws2812_init(void);
extern void task_rgb_ch(RGB_Controller* rgb_ptr);
extern void Task_RGBTest(void);
#endif

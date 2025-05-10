#include "ws2812_app.h"
#include "stl.h"
#include "uart.h"

uint8_t TIM_CCR_OUT_BUF1[RGB_LED_NUM * 24 +1];
RGB Disp_RGB_Tab1[RGB_LED_NUM];
#if 0
RGB_Color_Struct_ struct_color = 
{
  {0,168,243},//浅蓝
  {0,255,40},//浅绿
  {255,97,0},//橙色
  {255,100,0},
  {0,0,0},
};
#endif

#if 0
RGB_Controller Front_RGB={
  .ch=1,
  .rgb_led_num=10,
  .light_set_stl=1,
  .light_set_color[0].r=255,
  .light_set_color[0].g=0,
  .light_set_color[0].b=0,
  
  .light_set_color[1].r=0,
  .light_set_color[1].g=255,
  .light_set_color[1].b=0, 
  
  .light_set_color[2].r=0,
  .light_set_color[2].g=0,
  .light_set_color[2].b=255,
  
  .light_set_color[3].r=128,
  .light_set_color[3].g=128,
  .light_set_color[3].b=0, 

  .light_set_color[4].r=0,
  .light_set_color[4].g=128,
  .light_set_color[4].b=128,

  .light_set_color[5].r=128,
  .light_set_color[5].g=0,
  .light_set_color[5].b=128,

  .light_set_color[6].r=128,
  .light_set_color[6].g=128,
  .light_set_color[6].b=128,
    
  .light_set_color[7].r=255,
  .light_set_color[7].g=100,
  .light_set_color[7].b=128,
        
  .update_flag=1,  
};
#endif
/***********************************************************
* name :rgb_update_stl_info
*
* describe :read stl file data buffer,and update paramater
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static void rgb_update_stl_info(RGB_Controller* rgb_ptr,uint8_t stl_id)
{
    int i;
    switch(stl_id)
    {
        case 0:
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 0;
                rgb_ptr->current_stl.element_num = 0;
                rgb_ptr->current_stl.brightness = 0;
                
                break; 
        case 1: //单色呼吸
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 3000;
                rgb_ptr->current_stl.element_num = sizeof(stl_one_color_breath)/sizeof(RGB_Element_Typedef);
                rgb_ptr->current_stl.brightness = 255;

                for(i=0; i<rgb_ptr->current_stl.element_num; i++)
                {
                    rgb_ptr->elements[i] = stl_one_color_breath[i];
                }
                for(int i=0; i<rgb_ptr->current_stl.element_num; i++)//设定呼终止颜色颜色
                {
                    rgb_ptr->elements[i].start_color = rgb_ptr->light_set_color[0];                  
                    rgb_ptr->elements[i].end_color = rgb_ptr->light_set_color[0];
                }

                break;                
        case 2: //单色水波纹
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 750;
                rgb_ptr->current_stl.element_num = sizeof(one_clolor_water_flow)/sizeof(RGB_Element_Typedef);
                rgb_ptr->current_stl.brightness = 255;

                for(i=0; i<rgb_ptr->current_stl.element_num; i++)
                {
                  rgb_ptr->elements[i] = one_clolor_water_flow[i];
                } 
                for(i=0; i<rgb_ptr->current_stl.element_num -1; i++)
                {
                  rgb_ptr->elements[i].start_color  = rgb_ptr->light_set_color[0];
                  rgb_ptr->elements[i].end_color  = rgb_ptr->light_set_color[0];                  
                }
                break;

        case 3: //流星
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 750;
                rgb_ptr->current_stl.element_num = sizeof(one_clolor_meteor)/sizeof(RGB_Element_Typedef);
                rgb_ptr->current_stl.brightness = 255;

                for(i=0; i<rgb_ptr->current_stl.element_num; i++)
                {
                  rgb_ptr->elements[i] = one_clolor_meteor[i];
                }
                for(i=0; i<rgb_ptr->current_stl.element_num; i++)
                {
                  switch(i)
                  {
                    case 10:
                    case 11:
                    case 13:
                    case 15:                      
                    case 16:
                    case 18:
                    case 19:
                    case 21:
                    case 22:
                    case 24:                      
                    case 25:
                    case 27:
                    case 28:
                    case 30:
                    case 31:
                    case 33:                      
                    case 34:
                    case 36:
                    case 38:
                    case 39:                      
                      rgb_ptr->elements[i].start_color  = rgb_ptr->light_set_color[0];
                      rgb_ptr->elements[i].end_color  = rgb_ptr->light_set_color[0]; 
                      break;
                  }
                }                  
                break;
        case 4: //堆叠
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 12000;
                rgb_ptr->current_stl.element_num = sizeof(one_clolor_duidie)/sizeof(RGB_Element_Typedef);
                rgb_ptr->current_stl.brightness = 255;

                for(i=0; i<rgb_ptr->current_stl.element_num; i++)
                {
                  rgb_ptr->elements[i] = one_clolor_duidie[i];
                }
                for(i=10; i<29; i++)
                {
                  rgb_ptr->elements[i].start_color  = rgb_ptr->light_set_color[0];
                  rgb_ptr->elements[i].end_color  = rgb_ptr->light_set_color[0];                  
                }                
                break;  
        case 5: //一只小强
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 1500;
                rgb_ptr->current_stl.element_num = sizeof(one_clolor_bug)/sizeof(RGB_Element_Typedef);
                rgb_ptr->current_stl.brightness = 255;

                for(i=0; i<rgb_ptr->current_stl.element_num; i++)
                {
                  rgb_ptr->elements[i] = one_clolor_bug[i];
                } 
                for(i=10; i<13; i++)
                {
                  rgb_ptr->elements[i].start_color  = rgb_ptr->light_set_color[0];
                  rgb_ptr->elements[i].end_color  = rgb_ptr->light_set_color[0];                  
                }                 
                break;
        case 6: //流水  亮度增强
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 14000;
                rgb_ptr->current_stl.element_num = sizeof(one_clolor_flow_brightness)/sizeof(RGB_Element_Typedef);
                rgb_ptr->current_stl.brightness = 255;

                for(i=0; i<rgb_ptr->current_stl.element_num; i++)
                {
                  rgb_ptr->elements[i] = one_clolor_flow_brightness[i];
                }
                for(i=9; i<29; i++)
                {
                  rgb_ptr->elements[i].start_color  = rgb_ptr->light_set_color[0];
                  rgb_ptr->elements[i].end_color  = rgb_ptr->light_set_color[0];                  
                }                
                break; 
        case 7: //双色呼吸
              rgb_ptr->current_stl.id = stl_id;
              rgb_ptr->current_stl.total_time = 5000;
              rgb_ptr->current_stl.element_num = sizeof(stl_two_color_breath)/sizeof(RGB_Element_Typedef);
              rgb_ptr->current_stl.brightness = 255;
              for(i=0; i<rgb_ptr->current_stl.element_num; i++)
              {
                  rgb_ptr->elements[i] = stl_two_color_breath[i];
              }
              for(int i=0; i<10; i++)
              {
                  rgb_ptr->elements[i].start_color = rgb_ptr->light_set_color[0];
                  rgb_ptr->elements[i].end_color = rgb_ptr->light_set_color[1];                
              }              
              for(int i=10; i<20; i++)
              {
                  rgb_ptr->elements[i].start_color = rgb_ptr->light_set_color[1];
                  rgb_ptr->elements[i].end_color = rgb_ptr->light_set_color[0];
              }               
              break; 
              
        case 8: //多色流水
              rgb_ptr->current_stl.id = stl_id;
              rgb_ptr->current_stl.total_time = 800;
              rgb_ptr->current_stl.element_num = sizeof(stl_eight_color_flow)/sizeof(RGB_Element_Typedef);
              rgb_ptr->current_stl.brightness = 255;
              for(i=0; i<rgb_ptr->current_stl.element_num; i++)
              {
                  rgb_ptr->elements[i] = stl_eight_color_flow[i];
              }
               for(int i=0; i<rgb_ptr->current_stl.element_num; i++)
              {
                  rgb_ptr->elements[i].start_color = rgb_ptr->light_set_color[i%8];
                  rgb_ptr->elements[i].end_color = rgb_ptr->light_set_color[i%8];                
              }                
              break;                 
        case 9: //红色
              rgb_ptr->current_stl.id = stl_id;
              rgb_ptr->current_stl.total_time = 3000;
              rgb_ptr->current_stl.element_num = sizeof(red_clolor)/sizeof(RGB_Element_Typedef);
              rgb_ptr->current_stl.brightness = 255;
              for(i=0; i<rgb_ptr->current_stl.element_num; i++)
              {
                  rgb_ptr->elements[i] = red_clolor[i];
              }  
              break;               
        case 10:
              rgb_ptr->current_stl.id = stl_id;
              rgb_ptr->current_stl.total_time = 3000;
              rgb_ptr->current_stl.element_num = sizeof(stl_full_color_breath)/sizeof(RGB_Element_Typedef);
              rgb_ptr->current_stl.brightness = 255;
              for(i=0; i<rgb_ptr->current_stl.element_num; i++)
              {
                  rgb_ptr->elements[i] = stl_full_color_breath[i];
              }
               for(i=0; i<rgb_ptr->current_stl.element_num; i++)
              {
//                  rgb_ptr->elements[i].start_color = rgb_ptr->light_set_color[0];
//                  rgb_ptr->elements[i].end_color = rgb_ptr->light_set_color[1];                
              }  
          break;
        default:
                rgb_ptr->current_stl.id = stl_id;
                rgb_ptr->current_stl.total_time = 0;
                rgb_ptr->current_stl.element_num = 0;
                rgb_ptr->current_stl.brightness = 0;
                break;
    }   
}

/***********************************************************
* name :rgb_pattern_linear
*
* describe :线性插值
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static float rgb_pattern_linear(uint32_t st,uint32_t et,float sv,float ev,uint32_t ct)//
{
    float temp;
    /*（当前时间-起始时间）*（截止位置-起始位置）/（截止时间-起始时间）+起始位置*/
    temp =(int32_t)((((float)(ct - st)) * (ev-sv) / (et-st)) + sv);     
    return temp;
}

/***********************************************************
* name :rgb_pattern_accel
*
* describe :加速插值
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static float rgb_pattern_accel(uint32_t st,uint32_t et,float sv,float ev,uint32_t ct)
{
    if(st > et)
        return 0;
    if((ct > et) || (ct < st))
        return 0;
    
    if(ev == sv)
        return sv;
    else
    {
        //加速算法为匀加速
        float t = (float)ct - (float)st;
        float s = ev - sv;
        float tt = (float)et - (float)st;
        float a = 2*s/tt/tt;
        float ret = sv + 0.5*a*t*t;
        
        return (ret);
    }
}


/***********************************************************
* name :rgb_pattern_decel
*
* describe :减速插值
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static float rgb_pattern_decel(uint32_t st,uint32_t et,float sv,float ev,uint32_t ct)
{
    if(st > et)
        return 0;
    if((ct > et) || (ct < st))
        return 0;
    
    if(ev == sv)
        return sv;
    else
    {
        //减速算法为匀减速
        float t = (float)et - (float)ct;
        float s = ev - sv;
        float tt = (float)et - (float)st;
        float a = 2*s/tt/tt;
        float ret = ev - 0.5*a*t*t;
        
        return (ret);
    }
}

/***********************************************************
* name :rgb_pattern_accel_decel
*
* describe :先加速后减速
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static float rgb_pattern_accel_decel(uint32_t st,uint32_t et,float sv,float ev,uint32_t ct)
{
    if(st > et)
        return 0;
    if((ct > et) || (ct < st))
        return 0;
    
    if(ev == sv)
        return sv;
    else
    {        
        if(ct <= ((et + st)/2))//前一半时间做加速运动
        {
            return (rgb_pattern_accel(st,(et+st)/2,sv,(ev+sv)/2,ct));
        }
        else
            return (rgb_pattern_decel((et+st)/2,et,(ev+sv)/2,ev,ct));
    }
}

/***********************************************************
* name :rgb_pattern_decel_accel
*
* describe :先减速后加速
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static float rgb_pattern_decel_accel(uint32_t st,uint32_t et,float sv,float ev,uint32_t ct)
{
    if(st > et)
        return 0;
    if((ct > et) || (ct < st))
        return 0;
    
    if(ev == sv)
        return sv;
    else
    {
        if(ct <= ((et + st)/2))//前一半时间做减速运动
        {
            return (rgb_pattern_decel(st,(et+st)/2,sv,(ev+sv)/2,ct));
        }
        else
            return (rgb_pattern_accel((et+st)/2,et,(ev+sv)/2,ev,ct));
    }
}

/***********************************************************
* name :rgb_interpolation
*
* describe :插值函数
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static float rgb_interpolation(uint32_t st,uint32_t et,float sv,float ev,uint32_t ct,uint8_t pattern)
{
    float ret=0;
    
    switch(pattern)
    {
    case RGB_PATTERN_LINEAR:
        ret = rgb_pattern_linear(st,et,sv,ev,ct);
        break;
    case RGB_PATTERN_ACCEL:
        ret = rgb_pattern_accel(st,et,sv,ev,ct);
        break;
    case RGB_PATTERN_DECEL:
        ret = rgb_pattern_decel(st,et,sv,ev,ct);
        break;
    case RGB_PATTERN_ACCEL_DECEL:
        ret = rgb_pattern_accel_decel(st,et,sv,ev,ct);
        break;
    case RGB_PATTERN_DECEL_ACCEL:
        ret = rgb_pattern_decel_accel(st,et,sv,ev,ct);
        break;
    default:
        ret = 0;
        break;
    }
    
    return ret;
}

/***********************************************************
* name :ws2812_Refresh
*
* describe :refresh timer compare value via DMA
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static void ws2812_Refresh(RGB_Controller* rgb_ptr)
{
    uint16_t i;
    uint8_t b;
    uint16_t n=0;

    for(i=0;i<(rgb_ptr->rgb_led_num); i++)
    {
        for( b = 0x80; b; b >>= 1, n++)
        {
          if(rgb_ptr->Disp_RGB_Tab[i].g & b){
                rgb_ptr->RGB_TIM_CCR_OUT_BUF[n] = WS2811_BIT1;
          }else{
                rgb_ptr->RGB_TIM_CCR_OUT_BUF[n] = WS2811_BIT0;
          }
        }
        for( b = 0x80; b; b >>= 1, n++)
        {
          if(rgb_ptr->Disp_RGB_Tab[i].r & b){
                rgb_ptr->RGB_TIM_CCR_OUT_BUF[n] = WS2811_BIT1;
          }else{
                rgb_ptr->RGB_TIM_CCR_OUT_BUF[n] = WS2811_BIT0;
          }
        }
        for( b = 0x80; b; b >>= 1, n++)
        {
          if(rgb_ptr->Disp_RGB_Tab[i].b & b){
            rgb_ptr->RGB_TIM_CCR_OUT_BUF[n] = WS2811_BIT1;
          }else{
                rgb_ptr->RGB_TIM_CCR_OUT_BUF[n] = WS2811_BIT0;
          }
        }
    }
    rgb_ptr->RGB_TIM_CCR_OUT_BUF[n] = 0;
    WS2812_DMAInit(rgb_ptr->RGB_TIM_CCR_OUT_BUF,sizeof(rgb_ptr->RGB_TIM_CCR_OUT_BUF));
    //刷新
//    switch(rgb_ptr->ch)
//    {
//      case 1:        
//        //tmr5ch1_dma_configer(rgb_ptr->RGB_TIM_CCR_OUT_BUF,sizeof(rgb_ptr->RGB_TIM_CCR_OUT_BUF));    
//        break;
//      case 2:  
//        //tmr5ch2_dma_configer(rgb_ptr->RGB_TIM_CCR_OUT_BUF,sizeof(rgb_ptr->RGB_TIM_CCR_OUT_BUF));
//        break;    
//    }


}

/***********************************************************
* name :ws_Set
*
* describe :set RGB map value
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static void ws_Set(RGB_Controller* rgb_ptr,uint16_t addr, const RGB* rgb)
{
    rgb_ptr->Disp_RGB_Tab[addr] = *rgb;
}

static void ws_Set2(RGB* rgb_ptr,uint16_t addr, const RGB* rgb)
{
    *(rgb_ptr+addr) = *rgb;
}

/***********************************************************
* name :ws_Clear
*
* describe :clear RGB map value
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static void ws_Clear(RGB_Controller* rgb_ptr)
{
    uint16_t i;
    RGB temp_rgb={0,0,0};
    for(i=0;i < rgb_ptr->rgb_led_num;i++)
    {
      ws_Set(rgb_ptr,i,&temp_rgb);
    }
		
}

static void ws_Clear2(RGB* rgb_ptr)
{
  uint8_t i=0;
    RGB temp_rgb={0,0,0};
    for(i=0;i < RGB_LED_NUM;i++)
    {
      ws_Set2(rgb_ptr,i,&temp_rgb);
    }
}
#if 0
/***********************************************************
* name :ws_Init
*
* describe :init RGB map buffer
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static void ws_Init(RGB_Controller* rgb_ptr) 
{	
    ws_Clear(rgb_ptr);//RGB值清零
    ws2812_Refresh(rgb_ptr);
}

/***********************************************************
* name :rgb_controller_init
*
* describe :
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
static void rgb_controller_init(RGB_Controller* rgb_ptr) 
{ 
    int i;
    rgb_ptr->last_stl_id = -1;
    rgb_ptr->rgb_time = 0;
    rgb_ptr->rgb_timestamp = rgb_getTime();
    
    for(i=0; i<rgb_ptr->rgb_led_num; i++)//清除缓存
    {
        rgb_ptr->rgbs[i].r = 0;
        rgb_ptr->rgbs[i].g = 0;
        rgb_ptr->rgbs[i].b = 0;
    }
    
    rgb_ptr->current_stl.id = 0;
    rgb_ptr->current_stl.element_num = 0;
    rgb_ptr->current_stl.brightness = 0;
    rgb_ptr->current_stl.total_time = 0; 
    ws_Init(rgb_ptr);	
}
#endif

/***********************************************************
* name :task_rgb_ch
*
* describe :
*
* input :none
*
* output :none
*
* notice :2024/03/13
***********************************************************/
void task_rgb_ch(RGB_Controller* rgb_ptr)
{
    int i;
    if((rgb_ptr->light_set_stl !=rgb_ptr->last_stl_id) ||(rgb_ptr->update_flag))
    {
        //检测到灯条样式改变
        //更新样式文件信息
        rgb_update_stl_info(rgb_ptr,rgb_ptr->light_set_stl);
        rgb_ptr->last_stl_id = rgb_ptr->light_set_stl;
        rgb_ptr->rgb_time = 0;
        rgb_ptr->rgb_timeold = rgb_ptr->rgb_timestamp = rgb_getTime();
        rgb_ptr->update_flag=0;        					
        ws_Clear(rgb_ptr);//清空显示	
    }
    if(rgb_ptr->current_stl.id == 0)// 初始=0
    {						
        ws_Clear(rgb_ptr);//清除显示
    }
    else
    {
      rgb_ptr->rgb_time += (rgb_getTime() - rgb_ptr->rgb_timeold);
      rgb_ptr->rgb_timeold = rgb_getTime();
     
      if(rgb_ptr->rgb_time > rgb_ptr->current_stl.total_time)//一个显示周期循环完毕
      {
          rgb_ptr->rgb_time = 0;
          rgb_ptr->rgb_timeold = rgb_ptr->rgb_timestamp = rgb_getTime();
      }         
      for(i = 0; i < rgb_ptr->current_stl.element_num; i++)//动画元素数量
      {
          if((rgb_ptr->rgb_time >= rgb_ptr->elements[i].start_time) && (rgb_ptr->rgb_time <= rgb_ptr->elements[i].end_time))
          {
              //基本动画元素
              //如果当前元素有效
              int8_t position; 
              uint16_t brightness;
              //计算位置
              position = (int8_t)(rgb_interpolation(rgb_ptr->elements[i].start_time,rgb_ptr->elements[i].end_time,rgb_ptr->elements[i].start_position,\
                                      rgb_ptr->elements[i].end_position,rgb_ptr->rgb_time,rgb_ptr->elements[i].position_pattern) );//
              position =position%10;
              //计算亮度
              brightness = (uint8_t)(rgb_interpolation(rgb_ptr->elements[i].start_time,rgb_ptr->elements[i].end_time,rgb_ptr->elements[i].start_brightness,\
                                      rgb_ptr->elements[i].end_brightness,rgb_ptr->rgb_time,rgb_ptr->elements[i].brightness_pattern));//
              
              //计算颜色
              rgb_ptr->rgbs[position].r = (uint8_t)(((float)(brightness) / 256) * rgb_interpolation(
                                                          rgb_ptr->elements[i].start_time,
                                                          rgb_ptr->elements[i].end_time,
                                                          rgb_ptr->elements[i].start_color.r,
                                                          rgb_ptr->elements[i].end_color.r,
                                                          rgb_ptr->rgb_time,
                                                          rgb_ptr->elements[i].color_pattern) );//+ 0.5
              rgb_ptr->rgbs[position].g = (uint8_t)(((float)(brightness) / 256) * rgb_interpolation(
                                                          rgb_ptr->elements[i].start_time,
                                                          rgb_ptr->elements[i].end_time,
                                                          rgb_ptr->elements[i].start_color.g,
                                                          rgb_ptr->elements[i].end_color.g,
                                                          rgb_ptr->rgb_time,
                                                          rgb_ptr->elements[i].color_pattern) );//+ 0.5
              rgb_ptr->rgbs[position].b = (uint8_t)(((float)(brightness) / 256) * rgb_interpolation(
                                                          rgb_ptr->elements[i].start_time,
                                                          rgb_ptr->elements[i].end_time,
                                                          rgb_ptr->elements[i].start_color.b,
                                                          rgb_ptr->elements[i].end_color.b,
                                                          rgb_ptr->rgb_time,
                                                          rgb_ptr->elements[i].color_pattern) );// + 0.5            
          }
      }
  
      for(i=0; i<rgb_ptr->rgb_led_num; i++)
      {
          ws_Set(rgb_ptr,i, &rgb_ptr->rgbs[i]);
      }  
    }
    //TIM_SetCounter(TIM1,0);
    ws2812_Refresh(rgb_ptr); 
}
#if 0
/***********************************************************
* name :rgb_ctr_init
*
* describe : paramater Front_RGB init
*
* input :none
*
* output :none
*
* notice :2024/03/04
***********************************************************/
static void rgb_ctr_init(void)
{
    rgb_controller_init(&Front_RGB);
}
#endif

/***********************************************************
* name :ws_Refresh
*
* describe : refresh timer compare value via DMA
*
* input :none
*
* output :none
*
* notice :2024/03/04
***********************************************************/
static void ws_Refresh(void)
{
  uint16_t i;
  uint8_t b;
  uint16_t n;
  n = 0;
  for(i=0;i<SizeOf(Disp_RGB_Tab1); i++)//循环更新每颗灯的状态数组，发送到DMA
  {
    for( b = 0x80; b; b >>= 1, n++)
    {
      if(Disp_RGB_Tab1[i].g & b)
      {
        TIM_CCR_OUT_BUF1[n] = WS2811_BIT1;
      }
      else
      {
        TIM_CCR_OUT_BUF1[n] = WS2811_BIT0;
      }
    }
    for( b = 0x80; b; b >>= 1, n++)
    {
      if(Disp_RGB_Tab1[i].r & b)
      {
        TIM_CCR_OUT_BUF1[n] = WS2811_BIT1;
      }
      else
      {
        TIM_CCR_OUT_BUF1[n] = WS2811_BIT0;
      }
    }
    for( b = 0x80; b; b >>= 1, n++)
    {
      if(Disp_RGB_Tab1[i].b & b)
      {
        TIM_CCR_OUT_BUF1[n] = WS2811_BIT1;
      }
      else
      {
        TIM_CCR_OUT_BUF1[n] = WS2811_BIT0;
      }
    }
  }
  TIM_CCR_OUT_BUF1[n] = 0;
  WS2812_DMAInit(TIM_CCR_OUT_BUF1,sizeof(TIM_CCR_OUT_BUF1));
}


void Task_RGBTest(void)
{
    uint8_t i=0;
    for (i = 0;i < Logo_Param.rgb_num;i++)
    {
        Disp_RGB_Tab1[i].r = Logo_Param.r_val;
        Disp_RGB_Tab1[i].g = Logo_Param.g_val;
        Disp_RGB_Tab1[i].b = Logo_Param.b_val;
    }
    ws_Refresh();
}

void bsp_ws2812_init(void)
{
    WS2812_HardwareInit();
    ws_Clear2(Disp_RGB_Tab1);
    ws_Refresh();
}

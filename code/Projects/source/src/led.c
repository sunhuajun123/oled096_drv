#include "led.h"

/****************************************************************************** 
* Function Name    	: lEDGpio_Init
* Modify/Creat time	: 2024/08/05
* Modified/Creat By	: shj
* Description    	: LED gpio init 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/
static void lEDGpio_Init(void)
{
    GPIO_InitTypeDef Struct_GPIOInit;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
    
//    GPIO_DeInit(LED_PORT);
    Struct_GPIOInit.GPIO_Pin = LED_GPIO_PIN;
    Struct_GPIOInit.GPIO_Mode = GPIO_Mode_OUT;
    Struct_GPIOInit.GPIO_OType = GPIO_OType_PP;
    Struct_GPIOInit.GPIO_PuPd = GPIO_PuPd_DOWN;
    Struct_GPIOInit.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_Init(LED_PORT,&Struct_GPIOInit);
}

/****************************************************************************** 
* Function Name    	: led_ctr
* Modify/Creat time	: 2024/08/05
* Modified/Creat By	: shj
* Description    	: led on or off control 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/
static void led_ctr(uint8_t sta)
{
    if(sta)
    {
        LED_ON();
    }
    else
    {
        LED_OFF();
    }
}

/****************************************************************************** 
* Function Name    	: HardwareInit_Led
* Modify/Creat time	: 2024/08/05
* Modified/Creat By	: shj
* Description    	: task LED hardware init 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void HardwareInit_Led(void)
{
    lEDGpio_Init();
}
/****************************************************************************** 
* Function Name    	: Task_Led
* Modify/Creat time	: 2024/08/05
* Modified/Creat By	: shj
* Description    	: led flash cycle 1s 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void Task_Led(void)
{
  static uint8_t led_sta=0;
  led_sta ^= 1;
  led_ctr(led_sta);
}
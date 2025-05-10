#ifndef __WS2812_DRV_H__
#define __WS2812_DRV_H__
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"

#define WS2812_PORT GPIOA
#define WS2812_GPIO_PIN GPIO_Pin_10


#define RGB_TIM TIM1

#define MAIN_CLK        48000000
#define SizeOf(x)       (sizeof(x) / sizeof(x[0]))

#define WS2811_BIT0_NS  350//350ns
#define WS2811_BIT1_NS  700//700ns
#define WS2811_FULL_NS  1250//1250ns
#define	WS2811_BIT0     (WS2811_BIT0_NS*(MAIN_CLK/1000000)/1000)
#define	WS2811_BIT1	(WS2811_BIT1_NS*(MAIN_CLK/1000000)/1000)
#define	WS2811_FULL	(WS2811_FULL_NS*(MAIN_CLK/1000000)/1000)
#define	WS2811_TIM_DIV	(1)

extern void WS2812_HardwareInit(void);
extern void WS2812_DMAInit(uint8_t* pData, uint16_t DataLen);
#endif 

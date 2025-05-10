#ifndef __LED_H__
#define __LED_H__
#include "stm32f0xx.h"
#define LED_PORT GPIOA
#define LED_GPIO_PIN GPIO_Pin_4

#define LED_ON() GPIO_SetBits(LED_PORT,LED_GPIO_PIN);
#define LED_OFF() GPIO_ResetBits(LED_PORT,LED_GPIO_PIN);

extern void HardwareInit_Led(void);
extern void Task_Led(void);
#endif

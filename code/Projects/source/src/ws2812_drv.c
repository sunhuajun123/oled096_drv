#include "ws2812_drv.h"


static void WS2812_GpioInit(void)
{
    GPIO_InitTypeDef Struct_GPIOInit;
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);        
//    GPIO_DeInit(WS2812_PORT);
    Struct_GPIOInit.GPIO_Pin = WS2812_GPIO_PIN;
    Struct_GPIOInit.GPIO_Mode = GPIO_Mode_AF;
    Struct_GPIOInit.GPIO_Speed = GPIO_Speed_Level_3;
    Struct_GPIOInit.GPIO_OType = GPIO_OType_PP;
    Struct_GPIOInit.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(WS2812_PORT,&Struct_GPIOInit);
    GPIO_PinAFConfig(WS2812_PORT,GPIO_PinSource10,GPIO_AF_2);//gpio端口备用功能配置需要在GPIO功能配置后面
    
}

static void WS2812_TimeInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
    
    TIM_DeInit(RGB_TIM);

    TIM_TimeBaseStructure.TIM_Period = WS2811_FULL-1;//90
    TIM_TimeBaseStructure.TIM_Prescaler = WS2811_TIM_DIV-1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(RGB_TIM, &TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = 0;

    TIM_OC3Init(RGB_TIM, &TIM_OCInitStructure);      
    TIM_OC3PreloadConfig(RGB_TIM, TIM_OCPreload_Enable);
//    TIM_SetCompare3(RGB_TIM,WS2811_FULL/2);
    TIM_ARRPreloadConfig(RGB_TIM, ENABLE);  
    TIM_Cmd(RGB_TIM, ENABLE);
    TIM_CtrlPWMOutputs(RGB_TIM, ENABLE);
}

void WS2812_DMAInit(uint8_t* pData, uint16_t DataLen)
{
    DMA_InitTypeDef DMA_InitStructure;
    TIM_DMACmd(RGB_TIM, TIM_DMA_CC3, DISABLE);        
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode =  DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_BufferSize = DataLen;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&RGB_TIM->CCR3;
    
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    TIM_DMACmd(RGB_TIM, TIM_DMA_CC3, ENABLE); 
    DMA_Cmd(DMA1_Channel5, ENABLE);

     
}

void WS2812_HardwareInit(void)
{
    WS2812_GpioInit();
    WS2812_TimeInit();
}


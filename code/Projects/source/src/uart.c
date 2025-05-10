#include "uart.h"
#include "stm32f0xx_usart.h"
#include "string.h"

uint8_t Logo2IOT_Buff[9]={0xa5,0x71,0x01,0x03,0x41,0x43,0x4b,0xe9,0x5a};
union IOT2Logo_Protocol_ IOT2Logo_Protocol;
Logo_Param_Set_ Logo_Param = 
{
    115200,1,8,0,1,
    10,0,255,0
};

/****************************************************************************** 
* Function Name    	: Usart1_Init
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: USART1 init 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void Usart1_Init(void)
{
    GPIO_InitTypeDef Struct_GPIOInit;
    USART_InitTypeDef Struct_USARTInit;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_DMA1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    
    GPIO_PinAFConfig(USART1_GPIO_PORT,GPIO_PinSource2,GPIO_AF_1);
    GPIO_PinAFConfig(USART1_GPIO_PORT,GPIO_PinSource3,GPIO_AF_1);
    
    Struct_GPIOInit.GPIO_Pin = USART1_TX_PIN | USART1_RX_PIN;
    Struct_GPIOInit.GPIO_Mode = GPIO_Mode_AF;
    Struct_GPIOInit.GPIO_OType = GPIO_OType_PP;
    Struct_GPIOInit.GPIO_PuPd = GPIO_PuPd_UP;
    Struct_GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(USART1_GPIO_PORT,&Struct_GPIOInit);
    
    Struct_USARTInit.USART_BaudRate = 115200;
    Struct_USARTInit.USART_WordLength = USART_WordLength_8b;
    Struct_USARTInit.USART_StopBits = USART_StopBits_1;
    Struct_USARTInit.USART_Parity = USART_Parity_No;
    Struct_USARTInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    Struct_USARTInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&Struct_USARTInit);
    
    USART_ITConfig(USART1,USART_IT_TC,DISABLE);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
    USART1->ICR |= (1<<4);    
    USART_ClearITPendingBit(USART1,USART_IT_TXE);
    USART_ClearITPendingBit(USART1,USART_IT_TC);
    USART_ClearITPendingBit(USART1,USART_IT_ORE);
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(USART1,ENABLE);
}

/****************************************************************************** 
* Function Name    	: Usart_NVICInit
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: USART1 interrupt enable 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void Usart_NVICInit(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/****************************************************************************** 
* Function Name    	: USART1_DMATransInit
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: USART1 DMA send config 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void USART1_DMATransInit(uint8_t* pData,uint16_t DataLen)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->TDR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = DataLen;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel2,&DMA_InitStructure);
    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
    DMA_Cmd(DMA1_Channel2, DISABLE);
}

/****************************************************************************** 
* Function Name    	: USART1_DMAReceiveInit
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: USART1 DMA receive config 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
static void USART1_DMAReceiveInit(uint8_t* pData,uint16_t DataLen)
{
    DMA_InitTypeDef DMA_InitStructure;
    
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->RDR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = DataLen;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel3,&DMA_InitStructure);
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
}

/****************************************************************************** 
* Function Name    	: DMAChannel_Update
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: DMA channel update
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void DMAChannel_Update(DMA_Channel_TypeDef *DMAy_Channelx,uint32_t addr,uint16_t len)
{
    DMA_Cmd(DMAy_Channelx, DISABLE);
    DMAy_Channelx->CMAR = addr;
    DMAy_Channelx->CNDTR = len;
    DMA_Cmd(DMAy_Channelx, ENABLE);
}

/****************************************************************************** 
* Function Name    	: HardwareInit_Usart
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: usart1 init ,public interface
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void HardwareInit_Usart(void)
{
    Usart1_Init();
    Usart_NVICInit();    
    USART1_DMATransInit(Logo2IOT_Buff,9);
    USART1_DMAReceiveInit(IOT2Logo_Protocol.data_buffer,sizeof(IOT2Logo_Protocol));
}

/****************************************************************************** 
* Function Name    	: Frame_Check
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: protocol frame check 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
uint8_t Frame_Check(union IOT2Logo_Protocol_* frame_handle)
{
    uint8_t check_err=0;
    if (frame_handle->data_buffer[0]==0xA5 && frame_handle ->data_buffer[1] == 0x38)
    {
        check_err = 0;
    }
    else
    {
        check_err = 1;
    }
    return check_err;
}

/****************************************************************************** 
* Function Name    	: Logo_Response2IOT
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: protocol frame response(logo to IOT)
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void Logo_Response2IOT(void)
{
    DMAChannel_Update(DMA1_Channel2,(uint32_t)Logo2IOT_Buff,sizeof(Logo2IOT_Buff));
}

/****************************************************************************** 
* Function Name    	: ProtocolFrame_Process
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: protocol frame process
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/
void ProtocolFrame_Process(union IOT2Logo_Protocol_* frame_handle)
{
    uint8_t frame_check_err=0,cmd;
    frame_check_err = Frame_Check(frame_handle);
    if (!frame_check_err)
    {
        cmd = frame_handle->data_buffer[2];
        switch(cmd)
        {
        case 0:
          Logo_Param.baudrate_val = frame_handle->protocol_cmd0_iot2logo_.cmd0_iot2logo.baudrate;
          Logo_Param.start_bit = frame_handle->protocol_cmd0_iot2logo_.cmd0_iot2logo.start_bit;
          Logo_Param.data_bit = frame_handle->protocol_cmd0_iot2logo_.cmd0_iot2logo.data_bit;
          Logo_Param.check_bit = frame_handle->protocol_cmd0_iot2logo_.cmd0_iot2logo.check_bit;
          Logo_Param.stop_bit = frame_handle->protocol_cmd0_iot2logo_.cmd0_iot2logo.stop_bit;
          break;
        case 1:
          Logo_Param.rgb_num = frame_handle->protocol_cmd1_iot2logo_.cmd1_iot2logo.rgb_num;
          Logo_Param.r_val = frame_handle->protocol_cmd1_iot2logo_.cmd1_iot2logo.RGB_R;
          Logo_Param.g_val = frame_handle->protocol_cmd1_iot2logo_.cmd1_iot2logo.RGB_G;
          Logo_Param.b_val = frame_handle->protocol_cmd1_iot2logo_.cmd1_iot2logo.RGB_B;
          break;
        default:break;
        }
//        USART1->CR1 &= ~(1<<2); //关闭接收功能
//        Logo_Response2IOT();
    }
}

/****************************************************************************** 
* Function Name    	: USART1_IRQHandler
* Modify/Creat time	: 2024/11/27
* Modified/Creat By	: shj
* Description    	: USART1 interrupt process function 
* Input Param    	: void   
* Output Param    	: void
*******************************************************************************/ 
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1,USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1,USART_IT_TC);
//        USART1->CR1 |= (1<<2);
//        DMAChannel_Update(DMA1_Channel3,(uint32_t)IOT2Logo_Protocol.data_buffer,sizeof(IOT2Logo_Protocol.data_buffer));
    }
    if (USART_GetITStatus(USART1,USART_IT_IDLE) != RESET)
    {
        USART1->ICR |= (1<<4);
        USART_ClearITPendingBit(USART1,USART_IT_IDLE);
        ProtocolFrame_Process(&IOT2Logo_Protocol);   
        memset(IOT2Logo_Protocol.data_buffer,0,sizeof(IOT2Logo_Protocol.data_buffer));
        DMAChannel_Update(DMA1_Channel3,(uint32_t)IOT2Logo_Protocol.data_buffer,sizeof(IOT2Logo_Protocol.data_buffer));
    }
    USART_ClearITPendingBit(USART1,USART_IT_TXE);
    USART_ClearITPendingBit(USART1,USART_IT_TC);
    USART_ClearITPendingBit(USART1,USART_IT_ORE);
}



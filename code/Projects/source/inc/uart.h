#ifndef __UART_H__
#define __UART_H__
#include "stm32f0xx.h"

#define USART1_GPIO_PORT GPIOA
#define USART1_TX_PIN    GPIO_Pin_2
#define USART1_RX_PIN    GPIO_Pin_3

typedef struct 
{
  uint8_t destination_addr:4;
  uint8_t source_addr:4;
}struct_addr_;

typedef struct
{
    uint32_t baudrate;
    uint8_t start_bit;
    uint8_t data_bit;
    uint8_t check_bit;
    uint8_t stop_bit;
}cmd0_iot2logo_struct_;

typedef struct
{
    uint8_t rgb_num;
    uint8_t RGB_R;
    uint8_t RGB_G;
    uint8_t RGB_B;
}cmd1_iot2logo_struct_;

union IOT2Logo_Protocol_
{
    uint8_t data_buffer[20];
    struct 
    {
        uint8_t frame_head;
        struct_addr_ addr;
        uint8_t cmd;
        uint8_t data_length;
        cmd0_iot2logo_struct_ cmd0_iot2logo;
        uint8_t check_sum;
        uint8_t frame_end;
    }protocol_cmd0_iot2logo_;
    struct
    {
        uint8_t frame_head;
        struct_addr_ addr;
        uint8_t cmd;
        cmd1_iot2logo_struct_ cmd1_iot2logo;
        uint8_t check_sum;
    }protocol_cmd1_iot2logo_;
};

typedef struct
{
    uint32_t baudrate_val;
    uint8_t start_bit;
    uint8_t data_bit;
    uint8_t check_bit;
    uint8_t stop_bit;
    uint8_t rgb_num;
    uint8_t r_val;
    uint8_t g_val;
    uint8_t b_val;
}Logo_Param_Set_;

extern Logo_Param_Set_ Logo_Param;

extern void HardwareInit_Usart(void);
#endif

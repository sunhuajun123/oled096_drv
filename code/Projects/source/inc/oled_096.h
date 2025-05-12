#ifndef __OLED_096_H__
#define __OLED_096_H__
#include "iic_drv.h"

#define OLED_CMD  0
#define OLED_DATA 1

#define PAGE_MODE 0
#define HORIZONTAL_MODE 1
#define VERTICAL_MODE 2

typedef enum
{
    SHOW_GYRO=1,
    SHOW_ACCEL,
    SHOW_PITCH,
}OLED_MENU_;

extern OLED_MENU_ menu;

void OLED_Init(void);
void Test_OLEDShow(void);
void OLED_Clear(void);
#endif

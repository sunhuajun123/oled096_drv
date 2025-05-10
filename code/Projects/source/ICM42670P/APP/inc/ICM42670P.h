#ifndef __ICM42670_H__
#define __ICM42670_H__
#include "stm32f0xx.h"

#define PI 3.1415926535898
#define SoftDelay(x)	{for(uint16_t i=x;i>0;i--);}

#define s16 int16_t 

typedef struct
{
  float gyro_x; 
  float gyro_y; 
  float gyro_z;
  float acc_x; 
  float acc_y; 
  float acc_z;
  int angle_pitch;	
  int angle_roll;	
}scooter_gyro_;
extern scooter_gyro_ scooter_gyro;

enum sensor_type
{
	ENICM20600 = 0x11,
	ENICM42670P = 0x67,
};

extern enum sensor_type SENSORTPYE;

uint8_t Init_Icm20602(void);
uint8_t Read_Icm20602_Data(void);
extern uint64_t inv_imu_get_time_us(void);
extern void AccelGyroInfo_Process(void);
extern void ICM42670P_AccelAngleCount(scooter_gyro_* accel);
extern uint16_t ICM42670P_TCSProcess(scooter_gyro_* accel);
#endif

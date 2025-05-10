#include "ICM42670P.h"
#include "iic_drv.h"
#include "inv_imu_defs.h"
#include "inv_imu_regmap.h"
#include "math.h"
#include "main.h"


scooter_gyro_ scooter_gyro = {0};
enum sensor_type SENSORTPYE;

#define communication_err 0x00
#define communication_success 0x01

#define     ICM20602_DEVICE_ADDRESS                                 0xD0 //

#define     ICM20602_ID_REG_ADDRESS                                 (0x75)  //who am i

#define     ICM20602_CFG_REG_ADDR                                   (0x1A)
#define     ICM20602_GYRO_CFG_REG_ADDR                              (0x1B)
#define     ICM20602_ACC_CFG_REG_ADDR                               (0x1C)
#define     ICM20602_ACC_CFG2_REG_ADDR                              (0x1D)
#define     ICM20602_GYRO_LPM_CFG_REG_ADDR                          (0x1E)

#define     ICM20602_ACC_XOUT_H_REG_ADDR                            (0x3B)
#define     ICM20602_ACC_XOUT_L_REG_ADDR                            (0x3C)
#define     ICM20602_ACC_YOUT_H_REG_ADDR                            (0x3D)
#define     ICM20602_ACC_YOUT_L_REG_ADDR                            (0x3E)
#define     ICM20602_ACC_ZOUT_H_REG_ADDR                            (0x3F)
#define     ICM20602_ACC_ZOUT_L_REG_ADDR                            (0x40)

#define     ICM20602_TMEP_OUT_H_REG_ADDR                            (0x41)
#define     ICM20602_TMEP_OUT_L_REG_ADDR                            (0x42)

#define     ICM20602_GYRO_XOUT_H_REG_ADDR                           (0x43)
#define     ICM20602_GYRO_XOUT_L_REG_ADDR                           (0x44)
#define     ICM20602_GYRO_YOUT_H_REG_ADDR                           (0x45)
#define     ICM20602_GYRO_YOUT_L_REG_ADDR                           (0x46)
#define     ICM20602_GYRO_ZOUT_H_REG_ADDR                           (0x47)
#define     ICM20602_GYRO_ZOUT_L_REG_ADDR                           (0x48)


#define     ICM20602_USER_CTRL_REG_ADDR                             (0x6A)
#define     ICM20602_PWR_MGMT_1_REG_ADDR                            (0x6B)
#define     ICM20602_PWR_MGMT_2_REG_ADDR                            (0x6C)



void inv_imu_sleep_us(uint32_t us)
{
	SoftDelay(us);
}

uint64_t inv_imu_get_time_us(void)
{
   return sys_TimeGet()/1000;
}


//static volatile u32 sensor_temp = 0;
unsigned char GetData(unsigned char REG_Address,int16_t* data_buff)
{
  uint8_t Data_H,Data_L;

  if(I2C_ByteRead(ICM20602_DEVICE_ADDRESS,REG_Address,&Data_H))
  {
    if(I2C_ByteRead(ICM20602_DEVICE_ADDRESS,REG_Address+1,&Data_L))
    {
      *data_buff = ((char)Data_H<<8)+(char)Data_L;
      return communication_success;
    }
  }
  return communication_err;
}


static int inv_imu_device_reset(void)
{
	uint8_t data;
	uint8_t device_config_backup;
	uint8_t intf_config1_backup;
	
	/* Backup registers to configure serial interface */
	
	I2C_ByteRead(ICM20602_DEVICE_ADDRESS, DEVICE_CONFIG&0xff,&device_config_backup);
	I2C_ByteRead(ICM20602_DEVICE_ADDRESS, INTF_CONFIG1&0xff,&intf_config1_backup);

	/* Trigger soft reset (bit is automatically cleared once the reset is completed) */
	data = (uint8_t)SIGNAL_PATH_RESET_SOFT_RESET_DEVICE_CONFIG_EN;
	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,SIGNAL_PATH_RESET&0xff,data);


	/* Wait 1ms for soft reset to be effective */
	SoftDelay(1000);

	/* Reload OTP procedure (See AN-000273) */
	// reload_otp(s);
	
	/* Restore registers to configure serial interface */
	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,DEVICE_CONFIG&0xff, device_config_backup);
	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,INTF_CONFIG1&0xff, intf_config1_backup);

	return 1;
}

static uint32_t inv_imu_convert_odr_bitfield_to_us(uint32_t odr_bitfield)
{
	/*
 odr bitfield - frequency : odr ms
			0 - N/A
			1 - N/A
			2 - N/A
			3 - N/A
			4 - N/A
			5 - 1.6k      : 0.625ms
  (default) 6 - 800       : 1.25ms
			7 - 400       : 2.5 ms
			8 - 200       : 5 ms
			9 - 100       : 10 ms
			10 - 50       : 20 ms
			11 - 25       : 40 ms
			12 - 12.5     : 80 ms
			13 - 6.25     : 160 ms
			14 - 3.125    : 320 ms
			15 - 1.5625   : 640 ms
		*/
	
	switch(odr_bitfield ) {
	case ACCEL_CONFIG0_ODR_1600_HZ:    return 625;
	case ACCEL_CONFIG0_ODR_800_HZ:     return 1250;
	case ACCEL_CONFIG0_ODR_400_HZ:     return 2500;
	case ACCEL_CONFIG0_ODR_200_HZ:     return 5000;
	case ACCEL_CONFIG0_ODR_100_HZ:     return 10000;
	case ACCEL_CONFIG0_ODR_50_HZ:      return 20000;
	case ACCEL_CONFIG0_ODR_25_HZ:      return 40000;
	case ACCEL_CONFIG0_ODR_12_5_HZ:    return 80000;
	case ACCEL_CONFIG0_ODR_6_25_HZ:    return 160000;
	case ACCEL_CONFIG0_ODR_3_125_HZ:   return 320000;
	case ACCEL_CONFIG0_ODR_1_5625_HZ:
	default:                           return 640000;
	}
}
static int select_rcosc(void)
{
	int status = 0;
	uint8_t data;
	I2C_ByteRead(ICM20602_DEVICE_ADDRESS, PWR_MGMT0&0xff,&data);
	data &= ~PWR_MGMT0_ACCEL_LP_CLK_SEL_MASK;
	data |= PWR_MGMT0_ACCEL_LP_CLK_RCOSC;
	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,PWR_MGMT0&0xff,data);
	return status;
}

static int inv_imu_enable_accel_low_noise_mode(void)
{
	int status = 0;
	PWR_MGMT0_ACCEL_MODE_t accel_mode;
	PWR_MGMT0_GYRO_MODE_t  gyro_mode;
	ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint32_t accel_odr_us;
	uint8_t pwr_mgmt0_reg;
	uint8_t accel_config0_reg;

	I2C_ByteRead(ICM20602_DEVICE_ADDRESS, PWR_MGMT0&0xff,&pwr_mgmt0_reg);
	accel_mode = (PWR_MGMT0_ACCEL_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_ACCEL_MODE_MASK);
	gyro_mode = (PWR_MGMT0_GYRO_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_GYRO_MODE_MASK);
	/* Check if the accelerometer is the only one enabled */
	if (   (accel_mode == PWR_MGMT0_ACCEL_MODE_LP) 
		&& (   (gyro_mode == PWR_MGMT0_GYRO_MODE_OFF) 
			|| (gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY))) {
		/* Get accelerometer's ODR for next required wait */
		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, ACCEL_CONFIG0&0xff,&accel_config0_reg);
		acc_odr_bitfield = (ACCEL_CONFIG0_ODR_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_ODR_MASK);
		
		accel_odr_us = inv_imu_convert_odr_bitfield_to_us(acc_odr_bitfield);
		/* Select the RC OSC as clock source for the accelerometer */
		status |= select_rcosc();
		/* Wait one accel ODR before switching to low noise mode */
		SoftDelay(accel_odr_us);
	}

	/* Enable/Switch the accelerometer in/to low noise mode */
	/* Read a new time because select_rcosc() modified it */
	I2C_ByteRead(ICM20602_DEVICE_ADDRESS, PWR_MGMT0&0xff,&pwr_mgmt0_reg);		
	pwr_mgmt0_reg &= ~PWR_MGMT0_ACCEL_MODE_MASK;
	pwr_mgmt0_reg |= PWR_MGMT0_ACCEL_MODE_LN;
	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,PWR_MGMT0&0xff,pwr_mgmt0_reg);
	SoftDelay(200);

	SoftDelay(1000);
	return status;
}

static int inv_imu_enable_gyro_low_noise_mode(void)
{
	int status = 0;
	PWR_MGMT0_ACCEL_MODE_t accel_mode;
	PWR_MGMT0_GYRO_MODE_t gyro_mode;
	ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint32_t accel_odr_us;
	uint8_t pwr_mgmt0_reg;
	uint8_t accel_config0_reg;

	I2C_ByteRead(ICM20602_DEVICE_ADDRESS, PWR_MGMT0&0xff,&pwr_mgmt0_reg);
	accel_mode = (PWR_MGMT0_ACCEL_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_ACCEL_MODE_MASK);
	gyro_mode = (PWR_MGMT0_GYRO_MODE_t)(pwr_mgmt0_reg & PWR_MGMT0_GYRO_MODE_MASK);
	/* Check if the accelerometer is the only one enabled */
	if (   (accel_mode == PWR_MGMT0_ACCEL_MODE_LP) 
		&& (   (gyro_mode == PWR_MGMT0_GYRO_MODE_OFF) 
			|| (gyro_mode == PWR_MGMT0_GYRO_MODE_STANDBY))) {
		/* Get accelerometer's ODR for next required wait */
		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, ACCEL_CONFIG0&0xff,&accel_config0_reg);
		acc_odr_bitfield = (ACCEL_CONFIG0_ODR_t)(accel_config0_reg & ACCEL_CONFIG0_ACCEL_ODR_MASK);
		accel_odr_us = inv_imu_convert_odr_bitfield_to_us(acc_odr_bitfield);
		/* Select the RC OSC as clock source for the accelerometer */
		status |= select_rcosc();
		/* Wait one accel ODR before enabling the gyroscope */
		SoftDelay(accel_odr_us);
	}

	/* Enable/Switch the gyroscope in/to low noise mode */
	/* Read a new time because select_rcosc() modified it */
	I2C_ByteRead(ICM20602_DEVICE_ADDRESS, PWR_MGMT0&0xff,&pwr_mgmt0_reg);
	pwr_mgmt0_reg &= ~PWR_MGMT0_GYRO_MODE_MASK;
	pwr_mgmt0_reg |= (uint8_t)PWR_MGMT0_GYRO_MODE_LN;
	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,PWR_MGMT0&0xff,pwr_mgmt0_reg);
	SoftDelay(200);

	

	return status;
}



uint8_t Init_Icm20602(void)
{
  uint8_t uc_icm20602_reg_config_value_p = 0;
  uint8_t uc_icm20602_reg_value_fdb_p = 0;
  uint8_t value;
  /**
  * ICM20602_PWR_MGMT_1_REG_ADDR
  * bit7-RST          1-trig device reset     0.-no action
  * bit6-SLEEP        1-SLEEP                             0-NORMAL
  * bit5-CYCLE        1-ENABLE                            0-DISABLE ????ACC
  * bit4-GYRO_STANDBY     1-GYRO_SLEEP    0-GYRO_NROMAL
  * bit3-TMEP_DIS 1-DISBALE TEMP              0-ENABLE_TEMP
  * bit2:0-CLKSEL[2:0]    0or6-internal 20M  1~5-auto clk 7-no clk
  */
  //who am i, if ICM20600 return (0xD1)
  //uc_icm20602_reg_value_fdb_p = I2C_ByteRead(ICM20602_ID_REG_ADDRESS);
	I2C_ByteRead(ICM20602_DEVICE_ADDRESS,ICM20602_ID_REG_ADDRESS,&uc_icm20602_reg_value_fdb_p);
	if(uc_icm20602_reg_value_fdb_p!=0x11 && uc_icm20602_reg_value_fdb_p !=0x67)
	{
		return 0;
	} 
	else if (uc_icm20602_reg_value_fdb_p == ENICM42670P ) 
	{
		SENSORTPYE = ENICM42670P;
		// ÅäÖÃÎªI2CÄ£Ê½ 
		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, INTF_CONFIG1&0xff,&value);
		value &= ~(INTF_CONFIG1_I3C_SDR_EN_MASK | INTF_CONFIG1_I3C_DDR_EN_MASK);
		I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,INTF_CONFIG1&0xff,value);
		SoftDelay(3000);
		inv_imu_device_reset();
		SoftDelay(3000);
		// Disable Fsync
		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, FSYNC_CONFIG_MREG1,&value);
		value &= ~FSYNC_CONFIG_FSYNC_UI_SEL_MASK;
		value |= (uint8_t)FSYNC_CONFIG_UI_SEL_NO;
		I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,FSYNC_CONFIG_MREG1,value);
		SoftDelay(3000);
		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, TMST_CONFIG1_MREG1,&value);
		value &= ~TMST_CONFIG1_TMST_FSYNC_EN_MASK;
		value |= TMST_CONFIG1_TMST_FSYNC_DIS;
		I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,TMST_CONFIG1_MREG1,value);
		SoftDelay(3000);
		/* make sure FIFO is disabled */	
		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, FIFO_CONFIG1&0xff,&value);
		value &= ~FIFO_CONFIG1_FIFO_BYPASS_MASK;
		value |= (uint8_t)FIFO_CONFIG1_FIFO_BYPASS_ON;
		I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,FIFO_CONFIG1&0xff,value);
		SoftDelay(3000);

		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, ACCEL_CONFIG0&0xff,&value);
	    value &= ~ACCEL_CONFIG0_ACCEL_ODR_MASK;
	    value |= ACCEL_CONFIG0_ODR_800_HZ;
        value |= ACCEL_CONFIG0_FS_SEL_2g;
		I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,ACCEL_CONFIG0&0xff,value);
		SoftDelay(3000);

		I2C_ByteRead(ICM20602_DEVICE_ADDRESS, GYRO_CONFIG0&0xff,&value);
	    value &= ~GYRO_CONFIG0_GYRO_ODR_MASK;
	    value |= ACCEL_CONFIG0_ODR_800_HZ;
        value |= GYRO_CONFIG0_FS_SEL_2000dps;
		I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,GYRO_CONFIG0&0xff,value);
		SoftDelay(3000);
		inv_imu_enable_accel_low_noise_mode();
		inv_imu_enable_gyro_low_noise_mode();
		SoftDelay(3000);
		
		
  }else if (uc_icm20602_reg_value_fdb_p == ENICM20600  ) {
		SENSORTPYE = ENICM20600;
  
 	 /*auto clk to achieve full gyroscope performance*/
	  uc_icm20602_reg_config_value_p = 0x01;   
  	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,ICM20602_PWR_MGMT_1_REG_ADDR,uc_icm20602_reg_config_value_p);
  	/**
	  * ICM20602_CFG_REG_ADDR
 	 * bit6-FIFO MODE                    1-FIFO??????        0-FIFO????
 	 * bit5:3-EXT_SYNC_SET           1~7-SYNC SOURCE         0-DISABLE 
  	* bit2:0-DLPF_CFG[2:0]      DLPF??????
  	*/
 	 uc_icm20602_reg_config_value_p = 0x00;   
  	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,ICM20602_CFG_REG_ADDR,uc_icm20602_reg_config_value_p);
  
  	/**
  	* ICM20602_GYRO_CFG_REG_ADDR
  	* bit7-XG_ST                            X_GYRO self-test
 	 * bit6-YG_ST                            Y_GYRO self-test
  	* bit5-ZG_ST                            Z_GYRO self-test
  	* bit4:3-FS_SEL                     00-?50dps  01-?00dps  10-?000dps  11-?000dps
  	* bit1:0-FCHOICE_B              1~3-DLPF DISBALE        0-DLPF ENABLE
  	*/
  	uc_icm20602_reg_config_value_p = 0x18;   
  	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,ICM20602_GYRO_CFG_REG_ADDR,uc_icm20602_reg_config_value_p);
  
  	//f_icm20602_gyro_scale = (1/131.f)*(0x01<<( (uc_icm20602_reg_config_value_p&0x18)>>3 ));
  
  	/**
  	* ICM20602_ACC_CFG_REG_ADDR
  	* bit7-XA_ST                            X_ACC self-test
 	 * bit6-YA_ST                            Y_ACC self-test
  	* bit5-ZA_ST                            Z_ACC self-test
  	* bit4:3-FS_SEL                    
 	 */
  	uc_icm20602_reg_config_value_p = 0x00;   
 	I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,ICM20602_ACC_CFG_REG_ADDR,uc_icm20602_reg_config_value_p);
 }
  return 1;// f_icm20602_acc_scale = (1/16384.f)*(0x01<<( (uc_icm20602_reg_config_value_p&0x18)>>3 ));
}



uint8_t Read_Icm20602_Data(void)
{    
  uint8_t iic_accbuffer[6];
  uint8_t iic_gyrobuffer[6];
  s16 acc_x = 0;
  s16 acc_y = 0;
  s16 acc_z = 0;
  s16 gyro_x = 0;
  s16 gyro_y = 0;
  s16 gyro_z = 0;
  uint8_t acc_flag,gyro_flag;

  if (SENSORTPYE == ENICM42670P) {
	acc_flag = I2C_BytesRead(ICM20602_DEVICE_ADDRESS,ACCEL_DATA_X1&0xff,iic_accbuffer,6);
    gyro_flag = I2C_BytesRead(ICM20602_DEVICE_ADDRESS,GYRO_DATA_X1&0xff,iic_gyrobuffer,6);
  } else if (SENSORTPYE == ENICM20600) {
	acc_flag = I2C_BytesRead(ICM20602_DEVICE_ADDRESS,ICM20602_ACC_XOUT_H_REG_ADDR,iic_accbuffer,6);
    gyro_flag = I2C_BytesRead(ICM20602_DEVICE_ADDRESS,ICM20602_GYRO_XOUT_H_REG_ADDR,iic_gyrobuffer,6);
  }

  if(acc_flag)
  {
    acc_x = ((char)iic_accbuffer[0]<<8) + (char)iic_accbuffer[1];
    acc_y = ((char)iic_accbuffer[2]<<8) + (char)iic_accbuffer[3];
    acc_z = ((char)iic_accbuffer[4]<<8) + (char)iic_accbuffer[5];
    scooter_gyro.acc_x = (float)acc_x/16384;
    scooter_gyro.acc_y = (float)acc_y/16384;
    scooter_gyro.acc_z = (float)acc_z/16384;
    
  }
  if(gyro_flag)
  {
    gyro_x = ((char)iic_gyrobuffer[0]<<8) + (char)iic_gyrobuffer[1];
    gyro_y = ((char)iic_gyrobuffer[2]<<8) + (char)iic_gyrobuffer[3];
    gyro_z = ((char)iic_gyrobuffer[4]<<8) + (char)iic_gyrobuffer[5];
    
    scooter_gyro.gyro_x = (float)gyro_x/16.4;
    scooter_gyro.gyro_y = (float)gyro_y/16.4;
    scooter_gyro.gyro_z = (float)gyro_z/16.4;
  }

  
  return (acc_flag&gyro_flag);
}

/******************************************************************************
* Function Name    :icm20600_enter_sleep_mode
* Created          :
* Description      : none
* Input            : void
* return           : void
*******************************************************************************/
void icm20600_enter_sleep_mode(void)
{
  uint8_t uc_icm20602_reg_config_value_p  = 0x41;   
  I2C_ByteWrite(ICM20602_DEVICE_ADDRESS,ICM20602_PWR_MGMT_1_REG_ADDR,uc_icm20602_reg_config_value_p);
}

void AccelGyroInfo_Process(void)
{
	uint8_t reg=0;
	static uint8_t sSensorReadFail = 0;
	
	reg = Read_Icm20602_Data();
	if (!reg)
	{
		sSensorReadFail++;
	}
	if (sSensorReadFail > 50)
	{
		sSensorReadFail = 0;
		I2C_HardwareInit();
		Init_Icm20602();
	}
}


void ICM42670P_AccelAngleCount(scooter_gyro_* accel)
{
	double A;
	A = accel->acc_x * accel->acc_x + accel->acc_y * accel->acc_y;
	A = sqrt(A);
	A = (double)A / accel->acc_z;
	A = atan(A);
	accel->angle_pitch = (int)(A * 180 / PI); 
}

uint16_t ICM42670P_TCSProcess(scooter_gyro_* accel)
{
	uint16_t speed_return = 0;

//	lowPass_filter(&rc_gyro_x, accel->gyro_x);
//	lowPass_filter(&rc_gyro_y, accel->gyro_y);
//	lowPass_filter(&rc_gyro_z, accel->gyro_z);
	
//	if (accel->gyro_x < accel->acc_y)
//	{

//		speed_return = (uint16_t)accel->acc_y;
//	}
//	else
//	{
//		speed_return = (uint16_t)accel->gyro_x;
//	}
	speed_return = (uint16_t)(accel->gyro_z);
	return (speed_return);
}

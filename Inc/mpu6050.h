#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f4xx_hal.h"

//Mpu6050 init
//****************************************
#define	SMPLRT_DIV		0x19	

#define	CONFIG				0x1A	
#define	GYRO_CONFIG		0x1B	
#define	ACCEL_CONFIG	0x1C	
#define	ACCEL_XOUT_H	0x3B  
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E 
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41  
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43  
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B	
#define	WHO_AM_I			0x75	


//****************************

#define	MPU6050_Addr   0xD0

#define I2C_Timeout 0x1000

extern UART_HandleTypeDef huart3;
/*Include function*/

uint8_t  Mpu6050_Init(void);
void Read_Mpu6050_Data(void);


#endif

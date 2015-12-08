#ifndef __BMI160_H
#define __BMI160_H

#include "stm32f4xx_hal.h"

#define BMI160_CHIP_ID__REG		0x00
#define BMI160_PMU_STATUS			0x03
#define BMI160_ACCEL_CONF			0x40
#define BMI160_ACCEL_RANGE		0x41
#define BMI160_GYRO_CONF			0x42
#define BMI160_GYRO_RANGE			0x43
#define BMI160_PMU_TRIGGER_ADDR	0X6C
#define BMI160_ADRR						0xD0

#define BMI160_ACCEL_X_OUT_L			0x12
#define BMI160_ACCEL_X_OUT_H			0x13
#define BMI160_ACCEL_Y_OUT_L			0x14
#define BMI160_ACCEL_Y_OUT_H			0x15
#define BMI160_ACCEL_Z_OUT_L			0x16
#define BMI160_ACCEL_Z_OUT_H			0x17

#define BMI160_GYRO_X_OUT_L				0x0C
#define BMI160_GYRO_X_OUT_H				0x0D
#define BMI160_GYRO_Y_OUT_L				0x0E
#define BMI160_GYRO_Y_OUT_H				0x0F
#define BMI160_GYRO_Z_OUT_L				0x10
#define BMI160_GYRO_Z_OUT_H				0x11

#define BMI160_ERR_REG		0x02

extern UART_HandleTypeDef huart3;

uint8_t  BMI160_Init(void);
void Read_Bmi160_Data(void);

#endif

#ifndef __MMA8451Q_H
#define __MMA8451Q_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
	 
#define	MMA8451Q_ADDR		0x38
#define	ID_MMA8451Q	0x1A	//The ID value of MMA8451Q
#define	REG8451Q_WHO_AM_I	0x0D

#define REG8451Q_STATUS			0x00
#define	REG8451Q_OUT_X_MSB	0x01
#define	REG8451Q_OUT_X_LSB	0x02
#define	REG8451Q_OUT_Y_MSB	0x03
#define	REG8451Q_OUT_Y_LSB	0x04
#define	REG8451Q_OUT_Z_MSB	0x05
#define	REG8451Q_OUT_Z_LSB	0x06

#define	REG8451Q_CTRL_REG1	0x2A
#define	REG8451Q_CTRL_REG2	0x2B
#define	REG8451Q_CTRL_REG3	0x2C
#define	REG8451Q_CTRL_REG4	0x2D
#define	REG8451Q_CTRL_REG5	0x2E

#define REG8451Q_XYZ_DATA_CFG	0x0E


/* Function ----------------------------------------------------------------------*/
uint8_t  Mma8451q_Init(void);
void Read_Mma8451q_Data(void);

extern UART_HandleTypeDef huart3;

#endif


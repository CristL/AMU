#ifndef __TMP101_H
#define __TMP101_H

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define	TMP101_ADDR	0x90		//ADD0值为0

#define OUTPUT_CFG	0x00

#define TMP101_CFG_REG	0x01

#define	CONFIG_VALUE	0x6E	//配置为12位模式，连续转换

/* include ********************************************/

uint8_t  Tmp101_Init(void);
void Read_Tmp101_Data(void);

extern UART_HandleTypeDef huart3;

#endif


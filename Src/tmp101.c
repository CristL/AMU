#include "tmp101.h"
#include "i2c.h"
#include "string.h"
#include "stm32f4xx_hal.h"

//#define TMP101_CFG_REG	0x01	<--0xFE	/* 配置为12位 且连续输转换输出 */

uint8_t  Tmp101_Init(void)
{    
	Sensor_Single_WriteI2C(TMP101_CFG_REG, CONFIG_VALUE);/* 配置为12位 且连续输转换输出 */
							
  return WRITE_OK;
}


void Read_Tmp101_Data(void)
{
//	static uint8_t uint8_buffer[2];
	static uint16_t int16_buffer;
	static char dataOut[256];
		
	int16_buffer = Sensor_Single_ReadI2C(OUTPUT_CFG);

	sprintf(dataOut, "TMP:%d\r\n", (int)int16_buffer);
    HAL_UART_Transmit(&huart3, (uint8_t*)dataOut, strlen(dataOut), 5000);
}


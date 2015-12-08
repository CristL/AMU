#include "mma8451q.h"
#include "i2c.h"
#include "string.h"


//#define REG8451Q_CTRL_REG3	0x2C	<--0x01	/* Setup interrupt pin as open drain */
//#define REG8451Q_CTRL_REG2	0x2B	<--0x40	/* Reset accelerometer chipset */
//#define REG8451Q_CTRL_REG1	0x2A	<--0xD8	/* Put device in Standby / Sleep Mode Poll Rate of 1.56Hz / Output 100Hz*/
//#define REG8451Q_XYZ_DATA_CFG	0x0E	<--0x02	/* Set the range to +-8g */
uint8_t  Mma8451q_Init(void)
{    
	Sensor_Single_WriteI2C(REG8451Q_CTRL_REG3, 0x01);/* Setup interrupt pin as open drain */
	Sensor_Single_WriteI2C(REG8451Q_CTRL_REG2, 0x40);/* Reset accelerometer chipset */
	Sensor_Single_WriteI2C(REG8451Q_CTRL_REG1, 0xD8);/* Put device in Standby / Sleep Mode Poll Rate of 1.56Hz / Output 100Hz*/
	Sensor_Single_WriteI2C(REG8451Q_XYZ_DATA_CFG, 0x02);/* Set the range to +-8g */
							
  return WRITE_OK;
}


void Read_Mma8451q_Data(void)
{
	//uint8_t* pBuffer;
	static uint8_t mma_id;
	static uint8_t reg_status;
	static int8_t int8_buffer[6];
	static int16_t int16_buffer[3];
	static char dataOut[256];

		mma_id = Sensor_Single_ReadI2C(REG8451Q_WHO_AM_I);
	
		reg_status = Sensor_Single_ReadI2C(REG8451Q_STATUS);
		
		int8_buffer[0] = Sensor_Single_ReadI2C(REG8451Q_OUT_X_LSB);
		int8_buffer[1] = Sensor_Single_ReadI2C(REG8451Q_OUT_X_MSB);
		int16_buffer[0] = (int8_buffer[1])<<8 | int8_buffer[0];
		
		int8_buffer[2] = Sensor_Single_ReadI2C(REG8451Q_OUT_Y_LSB);
		int8_buffer[3] = Sensor_Single_ReadI2C(REG8451Q_OUT_Y_MSB);
		int16_buffer[1] = (int8_buffer[3])<<8 | int8_buffer[2];
		
		int8_buffer[4] = Sensor_Single_ReadI2C(REG8451Q_OUT_Z_LSB);
		int8_buffer[5] = Sensor_Single_ReadI2C(REG8451Q_OUT_Z_MSB);
		int16_buffer[2] = (int8_buffer[5])<<8 | int8_buffer[4];
		
		sprintf(dataOut, "MMA_ID:%d %d %d %d %d\r\n", (int)mma_id,(int)reg_status,(int)int16_buffer[0],(int)int16_buffer[1],(int)int16_buffer[2]);
    HAL_UART_Transmit(&huart3, (uint8_t*)dataOut, strlen(dataOut), 5000);
}


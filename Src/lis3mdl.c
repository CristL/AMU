#include "lis3mdl.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "string.h"
/*set the register 1-4 init_config*/
/*
**LIS3MDL_M_CTRL_REG3_M		0x00	Sets continuous-measurement mode;
**LIS3MDL_M_CTRL_REG1_M		0xFC	Sets UHP mode on the X/Y axes,ODR at 80 Hz and activates temerature sensor;
**LIS3MDL_M_CTRL_REG2_M		0x40	Sets full scale +-12;
**LIS3MDL_M_CTRL_REG4_M		0x0C	Sets UHP mode on the Z-axis;
*/

uint8_t  LIS3MDL_Init(void)
{    
  /* Conversion mode selection */
  
  Sensor_Single_WriteI2C(LIS3MDL_M_CTRL_REG3_M, 0x00);
  
    
  /* X and Y axes Operative mode selection */
   
  Sensor_Single_WriteI2C(LIS3MDL_M_CTRL_REG1_M, 0xFC);
  
    
  /* Full scale selection */
  Sensor_Single_WriteI2C(LIS3MDL_M_CTRL_REG2_M, 0x40);
	
	
	/* Z axes Operative mode selection */
  Sensor_Single_WriteI2C(LIS3MDL_M_CTRL_REG4_M, 0x0C);
	
  
  return WRITE_OK;
  
  /******************************/
}


void Read_Lis3mdl_Data(void)
{
	static int8_t magn_id;
	static int8_t int8_buffer[6];
	static int16_t int16_buffer[3];
	static char dataOut[256];                       /*!< DataOut Frame */

	magn_id = Sensor_Single_ReadI2C(LIS3MDL_M_WHO_AM_I_ADDR);

	int8_buffer[0] = Sensor_Single_ReadI2C(LIS3MDL_M_OUT_X_L_M);
	int8_buffer[1] = Sensor_Single_ReadI2C(LIS3MDL_M_OUT_X_H_M);
	int16_buffer[0] = (int8_buffer[1])<<8 | int8_buffer[0];

	int8_buffer[2] = Sensor_Single_ReadI2C(LIS3MDL_M_OUT_Y_L_M);
	int8_buffer[3] = Sensor_Single_ReadI2C(LIS3MDL_M_OUT_Y_H_M);
	int16_buffer[1] = (int8_buffer[3])<<8 | int8_buffer[2];

	int8_buffer[4] = Sensor_Single_ReadI2C(LIS3MDL_M_OUT_Z_L_M);
	int8_buffer[5] = Sensor_Single_ReadI2C(LIS3MDL_M_OUT_Z_H_M);
	int16_buffer[2] = (int8_buffer[5])<<8 | int8_buffer[4];
	
	sprintf(dataOut, "MAGN_ID: %d %d %d %d\r\n", (int)magn_id,(int)int16_buffer[0],(int)int16_buffer[1],(int)int16_buffer[2]);
  HAL_UART_Transmit(&huart3, (uint8_t*)dataOut, strlen(dataOut), 5000);

}

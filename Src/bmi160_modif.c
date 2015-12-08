#include "bmi160_modif.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"
#include "string.h"
/*set the register 1-4 init_config*/
/*
**BMI160_USER_ACCEL_CONF		0x28	100Hz	nomal_mode;
**BMI160_USER_ACCEL_RANGE		0x08	+-8g;
**BMI160_USER_GYRO_CONF			0x28	100Hz	normal_mode;
**BMI160_USER_GYRO_RANGE		0x01	+-1000dps;
*/

uint8_t  BMI160_Init(void)
{    
	/* 启动正常模式切换 */
	Sensor_Single_WriteI2C(BMI160_PMU_TRIGGER_ADDR, 0x00);
	HAL_Delay(1);
	/* 选择工作模式为normal mode */
	Sensor_Single_WriteI2C(BMI160_PMU_STATUS, 0x14);
	
	HAL_Delay(1);
	
	/* 100Hz	nomal_mode */

	Sensor_Single_WriteI2C(BMI160_ACCEL_CONF, 0x28);

	HAL_Delay(1);
	/* +-8g */

	Sensor_Single_WriteI2C(BMI160_ACCEL_RANGE, 0x08);

	HAL_Delay(1);
	/* 100Hz	normal_mode */
	Sensor_Single_WriteI2C(BMI160_GYRO_CONF, 0x28);

	HAL_Delay(1);
	/* +-1000dps */
	Sensor_Single_WriteI2C(BMI160_GYRO_RANGE, 0x01);


	return WRITE_OK;
  
  /******************************/
}


void Read_Bmi160_Data(void)
{
	static uint8_t bmi160_id;
	static uint8_t err_reg;
	static uint8_t pmu_status;
	static int8_t int8_buffer[12];
	static int16_t int16_buffer[6];
	static char dataOut[256];                       /*!< DataOut Frame */

	bmi160_id = Sensor_Single_ReadI2C(BMI160_CHIP_ID__REG);
	
	err_reg = Sensor_Single_ReadI2C(BMI160_ERR_REG);
	
	pmu_status = Sensor_Single_ReadI2C(BMI160_PMU_STATUS);

	int8_buffer[0] = Sensor_Single_ReadI2C(BMI160_ACCEL_X_OUT_L);
	int8_buffer[1] = Sensor_Single_ReadI2C(BMI160_ACCEL_X_OUT_H);
	int16_buffer[0] = (int8_buffer[1])<<8 | int8_buffer[0];

	int8_buffer[2] = Sensor_Single_ReadI2C(BMI160_ACCEL_Y_OUT_L);
	int8_buffer[3] = Sensor_Single_ReadI2C(BMI160_ACCEL_Y_OUT_H);
	int16_buffer[1] = (int8_buffer[3])<<8 | int8_buffer[2];

	int8_buffer[4] = Sensor_Single_ReadI2C(BMI160_ACCEL_Z_OUT_L);
	int8_buffer[5] = Sensor_Single_ReadI2C(BMI160_ACCEL_Z_OUT_H);
	int16_buffer[2] = (int8_buffer[5])<<8 | int8_buffer[4];
		
	int8_buffer[6] = Sensor_Single_ReadI2C(BMI160_GYRO_X_OUT_L);
	int8_buffer[7] = Sensor_Single_ReadI2C(BMI160_GYRO_X_OUT_H);
	int16_buffer[3] = (int8_buffer[7])<<8 | int8_buffer[6];
	
	int8_buffer[8] = Sensor_Single_ReadI2C(BMI160_GYRO_Y_OUT_L);
	int8_buffer[9] = Sensor_Single_ReadI2C(BMI160_GYRO_Y_OUT_H);
	int16_buffer[4] = (int8_buffer[9])<<8 | int8_buffer[8];
	
	int8_buffer[10] = Sensor_Single_ReadI2C(BMI160_GYRO_Z_OUT_L);
	int8_buffer[11] = Sensor_Single_ReadI2C(BMI160_GYRO_Z_OUT_H);
	int16_buffer[5] = (int8_buffer[11])<<8 | int8_buffer[10];
	
	sprintf(dataOut, "BMI_ID: %d;ERR_REG:%d;PMU_STATUS:%d;data:%d %d %d %d %d %d\r\n", (int)bmi160_id,(int)err_reg,(int)pmu_status,(int)int16_buffer[0],\
		(int)int16_buffer[1],(int)int16_buffer[2],(int)int16_buffer[3],(int)int16_buffer[4],(int)int16_buffer[5]);
    HAL_UART_Transmit(&huart3, (uint8_t*)dataOut, strlen(dataOut), 5000);

}

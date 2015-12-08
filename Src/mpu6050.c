#include "mpu6050.h"
#include "i2c.h"
#include "string.h"

/*
uint8_t pBuffer_SMPLRT_DIV[] = "0x02";
uint8_t pBuffer_PWR_MGMT_1[] = "0x00";
uint8_t pBuffer_CONFIG[] = "0x02";
uint8_t pBuffer_GYRO_CONFIG[] = "0x18";
uint8_t pBuffer_ACCEL_CONFIG[] = "0x11";
*/
static uint8_t pBuffer_SMPLRT_DIV = 0x02;
static uint8_t pBuffer_PWR_MGMT_1 = 0x00;
static uint8_t pBuffer_CONFIG = 0x02;
static uint8_t pBuffer_GYRO_CONFIG = 0x18;
static uint8_t pBuffer_ACCEL_CONFIG = 0x11;

uint8_t  Mpu6050_Init(void)
{    
	/*
	HAL_I2C_Mem_Write(&hi2c3, SENSOR_Addr, (uint16_t)SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, pBuffer_SMPLRT_DIV, 1, I2C_Timeout);
	HAL_I2C_Mem_Write(&hi2c3, SENSOR_Addr, (uint16_t)PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, pBuffer_PWR_MGMT_1, 1, I2C_Timeout);
	HAL_I2C_Mem_Write(&hi2c3, SENSOR_Addr, (uint16_t)CONFIG, I2C_MEMADD_SIZE_8BIT, pBuffer_CONFIG, 1, I2C_Timeout);
	HAL_I2C_Mem_Write(&hi2c3, SENSOR_Addr, (uint16_t)GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, pBuffer_GYRO_CONFIG, 1, I2C_Timeout);
	HAL_I2C_Mem_Write(&hi2c3, SENSOR_Addr, (uint16_t)ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, pBuffer_ACCEL_CONFIG, 1, I2C_Timeout);
	*/
	Sensor_Single_WriteI2C(SMPLRT_DIV, pBuffer_SMPLRT_DIV);//取样率2分频，1k/2 = 500Hz
	Sensor_Single_WriteI2C(PWR_MGMT_1, pBuffer_PWR_MGMT_1);//接触休眠状态
	Sensor_Single_WriteI2C(CONFIG, pBuffer_CONFIG);//低通滤波94Hz
	Sensor_Single_WriteI2C(GYRO_CONFIG, pBuffer_GYRO_CONFIG);//2000dps
	Sensor_Single_WriteI2C(ACCEL_CONFIG, pBuffer_ACCEL_CONFIG);//+-8g,5Hz高通滤波
								
  return WRITE_OK;
}


void Read_Mpu6050_Data(void)
{
	//uint8_t* pBuffer;
	static uint8_t mpu_id;
	static int8_t int8_buffer[12];
	static int16_t int16_buffer[6];
	static char dataOut[256];
	/*
		HAL_I2C_Mem_Read(&hi2c3, SENSOR_Addr, (uint16_t)WHO_AM_I, I2C_MEMADD_SIZE_8BIT, pBuffer, 1,I2C_Timeout);
		sprintf(dataOut, "ID: %d\n\r", (int)pBuffer[0]);
    HAL_UART_Transmit(&huart3, (uint8_t*)dataOut, strlen(dataOut), 5000);
		
		HAL_I2C_Mem_Read(&hi2c3, SENSOR_Addr, (uint16_t)ACCEL_XOUT_L, I2C_MEMADD_SIZE_8BIT, pBuffer, 1,I2C_Timeout);
		sprintf(dataOut, "Ax: %d\n\r", (int)pBuffer[0]);
    HAL_UART_Transmit(&huart3, (uint8_t*)dataOut, strlen(dataOut), 5000);
		*/
		mpu_id = Sensor_Single_ReadI2C(WHO_AM_I);
		
		int8_buffer[0] = Sensor_Single_ReadI2C(ACCEL_XOUT_L);
		int8_buffer[1] = Sensor_Single_ReadI2C(ACCEL_XOUT_H);
		int16_buffer[0] = (int8_buffer[1])<<8 | int8_buffer[0];
		
		int8_buffer[2] = Sensor_Single_ReadI2C(ACCEL_YOUT_L);
		int8_buffer[3] = Sensor_Single_ReadI2C(ACCEL_YOUT_H);
		int16_buffer[1] = (int8_buffer[3])<<8 | int8_buffer[2];
		
		int8_buffer[4] = Sensor_Single_ReadI2C(ACCEL_ZOUT_L);
		int8_buffer[5] = Sensor_Single_ReadI2C(ACCEL_ZOUT_H);
		int16_buffer[2] = (int8_buffer[5])<<8 | int8_buffer[4];
		
		int8_buffer[6] = Sensor_Single_ReadI2C(GYRO_XOUT_L);
		int8_buffer[7] = Sensor_Single_ReadI2C(GYRO_XOUT_H);
		int16_buffer[3] = (int8_buffer[7])<<8 | int8_buffer[6];
		
		int8_buffer[8] = Sensor_Single_ReadI2C(GYRO_YOUT_L);
		int8_buffer[9] = Sensor_Single_ReadI2C(GYRO_YOUT_H);
		int16_buffer[4] = (int8_buffer[9])<<8 | int8_buffer[8];
		
		int8_buffer[10] = Sensor_Single_ReadI2C(GYRO_ZOUT_L);
		int8_buffer[11] = Sensor_Single_ReadI2C(GYRO_ZOUT_H);
		int16_buffer[5] = (int8_buffer[11])<<8 | int8_buffer[10];
		
		sprintf(dataOut, "MPU_ID: %d %d %d %d %d %d %d\r\n", (int)mpu_id,(int)int16_buffer[0],\
			(int)int16_buffer[1],(int)int16_buffer[2],(int)int16_buffer[3],(int)int16_buffer[4],(int)int16_buffer[5]);
    HAL_UART_Transmit(&huart3, (uint8_t*)dataOut, strlen(dataOut), 5000);
}


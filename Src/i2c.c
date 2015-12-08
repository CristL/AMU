/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "mpu6050.h"
#include "lis3mdl.h"
#include "bmi160_modif.h"
#include "mma8451q.h"

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 208;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{

  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  }
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
} 

/* USER CODE BEGIN 1 */
void Sensor_Single_WriteI2C(uint8_t REG_address,uint8_t REG_data)
{
	uint8_t txData[2] = {REG_address,REG_data};
	while(HAL_I2C_Master_Transmit(&hi2c1,SENSOR_Addr,txData,2,0xffff) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{}
	}
}



#if defined (TMP101)
uint16_t Sensor_Single_ReadI2C(uint8_t REG_address)
{
	uint8_t REG_8data[2];
	uint16_t REG_16data;
	while(HAL_I2C_Master_Transmit(&hi2c1,SENSOR_Addr,&REG_address,1,0xffff) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{}
	}
	
	if(HAL_I2C_Master_Receive(&hi2c1,SENSOR_Addr+1,REG_8data,2,0xffff) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{}
	}
	/*
	hi2c1.Instance -> CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);//I2C_ACKnowledgeConfig(I2C1,DISABLE);���һλ��Ҫ�ر�Ӧ���źţ�
	hi2c1.Instance -> CR1 |= I2C_CR1_STOP;//I2C_GenerateSTOP(I2C1,ENABLE);����ֹͣλ
	if(HAL_I2C_Master_Receive(&hi2c1,SENSOR_Addr+1,&REG_8data[1],1,0xffff) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{}
	}
	hi2c1.Instance -> CR1 |= I2C_CR1_ACK;//��Ӧ���ź�
	*/
	REG_16data = ((REG_8data[0] << 8)|(REG_8data[1])) >> 4;
	
	return REG_16data;
}

#else

uint8_t Sensor_Single_ReadI2C(uint8_t REG_address)
{
	uint8_t REG_data;
	while(HAL_I2C_Master_Transmit(&hi2c1,SENSOR_Addr,&REG_address,1,0xffff) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{}
	}
	
	if(HAL_I2C_Master_Receive(&hi2c1,SENSOR_Addr+1,&REG_data,1,0xffff) != HAL_OK)
	{
		if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{}
	}
	return REG_data;
}

#endif
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

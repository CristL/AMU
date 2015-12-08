/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"


typedef struct
{
	uint8_t RTC_Hours;    /*!< Specifies the RTC Time Hour.
						This parameter must be set to a value in the 0-12 range
						if the RTC_HourFormat_12 is selected or 0-23 range if
						the RTC_HourFormat_24 is selected. */

	uint8_t RTC_Minutes;  /*!< Specifies the RTC Time Minutes.
						This parameter must be set to a value in the 0-59 range. */

	uint8_t RTC_Seconds;  /*!< Specifies the RTC Time Seconds.
						This parameter must be set to a value in the 0-59 range. */
	uint16_t RTC_McrosSeconds;

	uint8_t RTC_H12;      /*!< Specifies the RTC AM/PM Time.
						This parameter can be a value of @ref RTC_AM_PM_Definitions */
}RTC_Ms_TimeTypeDef; 

RTC_Ms_TimeTypeDef TickstoStandardFormat(uint32_t Ticks);

#endif


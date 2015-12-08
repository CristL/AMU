/*************************************************************
*by CL at 12.3
************************************************************/

#include "timer.h"

RTC_Ms_TimeTypeDef TickstoStandardFormat(uint32_t Ticks){
	RTC_Ms_TimeTypeDef RTC_Now_TimeStructure;

	RTC_Now_TimeStructure.RTC_McrosSeconds = Ticks % 1000;/* get the time (ms) */
	Ticks /= 1000;
	RTC_Now_TimeStructure.RTC_Seconds = Ticks % 60;/* get the time (s) */
	Ticks /= 60;
	RTC_Now_TimeStructure.RTC_Minutes = Ticks % 60;/* get the time (mins) */
	Ticks /= 60;
	RTC_Now_TimeStructure.RTC_Hours = Ticks % 24;/* get the time (hours)) */
	
	return RTC_Now_TimeStructure;
}


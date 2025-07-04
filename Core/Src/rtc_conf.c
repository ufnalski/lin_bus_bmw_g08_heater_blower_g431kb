/*
 * rtc_conf.c
 *
 *  Created on: Dec 15, 2023
 *      Author: user
 */


#include "rtc_conf.h"
#include <stdlib.h>
#include <string.h>
//#include "rtc.h"

RTC_TimeTypeDef timeNow;
RTC_DateTypeDef dateNow;

void SetRtcCompilationTimeAndDate(void)
{
	GetCompilationDate(&dateNow);
	CalculateDayOfWeek(&dateNow);
	HAL_RTC_SetDate(&hrtc, &dateNow, RTC_FORMAT_BIN);
	GetCompilationTime(&timeNow);
	HAL_RTC_SetTime(&hrtc, &timeNow, RTC_FORMAT_BIN);
}

void GetCompilationTime(RTC_TimeTypeDef *timePtr)
{
	char temp[] = __TIME__;

	timePtr->Seconds = atoi(temp + 6);
	*(temp + 5) = 0;
	timePtr->Minutes = atoi(temp + 3);
	*(temp + 2) = 0;
	timePtr->Hours = atoi(temp);
}

void GetCompilationDate(RTC_DateTypeDef *datePtr)
{
	const char *months[] =
	{ "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct",
			"Nov", "Dec" };
	char temp[] = __DATE__;
	uint8_t i;

	datePtr->Year = atoi(temp + 9);
	*(temp + 6) = 0;
	datePtr->Date = atoi(temp + 4);
	*(temp + 3) = 0;
	for (i = 0; i < 12; i++)
	{
		if (!strcmp(temp, months[i]))
		{
			datePtr->Month = i + 1;
			return;
		}
	}
}

// https://www.geeksforgeeks.org/find-day-of-the-week-for-a-given-date/
void CalculateDayOfWeek(RTC_DateTypeDef *datePtr)
{
	const int16_t t[] =
	{ 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
	int16_t y, m, d;
	m = datePtr->Month;
	y = (datePtr->Year) + 2000 - (m < 3);
	d = datePtr->Date;
	datePtr->WeekDay = (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
	if (datePtr->WeekDay == 0)
	{
		datePtr->WeekDay = 7;
	}
}

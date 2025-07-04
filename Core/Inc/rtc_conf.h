/*
 * rtc_conf.h
 *
 *  Created on: Dec 15, 2023
 *      Author: user
 */

#ifndef INC_RTC_CONF_H_
#define INC_RTC_CONF_H_

#include "rtc.h"

void SetRtcCompilationTimeAndDate(void);
void GetCompilationTime(RTC_TimeTypeDef *timePtr);
void GetCompilationDate(RTC_DateTypeDef *datePtr);
void CalculateDayOfWeek(RTC_DateTypeDef *datePtr);

#endif /* INC_RTC_CONF_H_ */

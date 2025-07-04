/*
 * df2301q_voice_recognition.c
 *
 *  Created on: Jul 3, 2025
 *      Author: user
 */

#include "df2301q_voice_recognition.h"

uint8_t DF2301G_GetCommandID(void)
{
	HAL_StatusTypeDef hal_status;
	uint8_t cmd_id;

	hal_status = HAL_I2C_Mem_Read(DF2301Q_I2C_HANDLE_PTR, DF2301Q_I2C_ADDR,
			DF2301Q_I2C_REG_CMDID, 1, &cmd_id, 1, 100);

	if (hal_status != HAL_OK)
	{
		return 0xFF;
	}
	else
	{
		return cmd_id;
	}
}


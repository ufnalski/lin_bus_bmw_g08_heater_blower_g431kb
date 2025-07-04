/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ssd1306.h"
#include "df2301q_voice_recognition.h"
#include "rtc_conf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEATER_BLOWER_DIAGNOSTICS_LIN_MSG_ID 0x1C
#define HEATER_BLOWER_REF_SPEED_LIN_MSG_ID 0x1D

#define LIN_UART_HANDLE_PTR &huart1
#define LIN_UART_INSTANCE USART1
#define LIN_SEND_PERIOD 250
#define TX_BUFFER_SIZE 17 //11
#define RX_BUFFER_SIZE 17 //12

#define ENC_KNOB_CNT_MIN 8
#define ENC_KNOB_CNT_MAX 64

#define CURRENT_SCALING 5.657f
#define VOLTAGE_SCALING 10.0f
#define SPEED_SCALING 26.2f // rpm

#define BATTERY_VOLTAGE_PROTECTION 11.1f   // V

#define VOICE_CHECK_PERIOD 500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
uint8_t txData[TX_BUFFER_SIZE];
uint8_t rxData[RX_BUFFER_SIZE];
uint8_t rxDataShadow[RX_BUFFER_SIZE];
uint32_t LinSendSoftTimer;
uint32_t VoiceRecognitionSoftTimer;

volatile uint8_t lin_data_received_flag = 0;

uint8_t uart_line[128];
uint8_t lcd_line[32];

float drive_voltage;
float drive_current;

uint8_t low_voltage_flag = 0;
uint8_t ref_speed = 0;

uint8_t voice_cmd_id;

// Timestamp
extern RTC_TimeTypeDef timeNow;
extern RTC_DateTypeDef dateNow;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// https://controllerstech.com/stm32-uart-8-lin-protocol-part-1/
// https://controllerstech.com/stm32-uart-9-lin-protocol-part-2/
// https://controllerstech.com/stm32-uart-10-lin-protocol-part-3/
uint8_t Pid_Calc(uint8_t ID);
uint8_t Checksum_Calc(uint8_t PID, uint8_t *data, uint8_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_I2C3_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 0);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(28, 12);
	ssd1306_WriteString("LIN bus demo", Font_6x8, White);
	ssd1306_SetCursor(2, 22);
	ssd1306_WriteString("BMW G08 heater blower", Font_6x8, White);
	ssd1306_SetCursor(8, 32);
	ssd1306_WriteString("with voice commands", Font_6x8, White);
	ssd1306_UpdateScreen();

	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim2, (ENC_KNOB_CNT_MIN + ENC_KNOB_CNT_MAX) / 2);

	SetRtcCompilationTimeAndDate();

	/* USER CODE END 2 */

	/* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
	BspCOMInit.BaudRate = 115200;
	BspCOMInit.WordLength = COM_WORDLENGTH_8B;
	BspCOMInit.StopBits = COM_STOPBITS_1;
	BspCOMInit.Parity = COM_PARITY_NONE;
	BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
	if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN BSP */

	/* -- Sample board code to send message over COM1 port ---- */
	printf("Welcome to LIN bus world!\r\n");

	/* USER CODE END BSP */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	VoiceRecognitionSoftTimer = HAL_GetTick();
	LinSendSoftTimer = HAL_GetTick();

	while (1)
	{

		// Reference speed sending
		if (HAL_GetTick() - LinSendSoftTimer > LIN_SEND_PERIOD)
		{
			LinSendSoftTimer = HAL_GetTick();
			HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

			if (low_voltage_flag == 0)
			{
				ref_speed = __HAL_TIM_GET_COUNTER(&htim2);
			}
			else
			{
				ref_speed = 0;
			}

			txData[0] = 0x55;
			txData[1] = Pid_Calc(HEATER_BLOWER_REF_SPEED_LIN_MSG_ID);
			txData[2] = 0x00;
			txData[3] = 0x00;
			txData[4] = 0x08; // activates blower?
			txData[5] = 0x00;
			txData[6] = 0x00;
			txData[7] = ref_speed; // sets speed
			txData[8] = 0x00;
			txData[9] = 0x00;
			txData[10] = Checksum_Calc(txData[1], txData + 2, 8);

			HAL_UARTEx_ReceiveToIdle_IT(LIN_UART_HANDLE_PTR, rxData,
			RX_BUFFER_SIZE);
			HAL_LIN_SendBreak(LIN_UART_HANDLE_PTR);
			HAL_UART_Transmit(LIN_UART_HANDLE_PTR, txData, 11, LIN_SEND_PERIOD);

			HAL_Delay(LIN_SEND_PERIOD / 2);
			// ask for diagnostics
			txData[0] = 0x55;
			txData[1] = Pid_Calc(HEATER_BLOWER_DIAGNOSTICS_LIN_MSG_ID);

			memset(rxData, 0x00, RX_BUFFER_SIZE);
			HAL_UARTEx_ReceiveToIdle_IT(LIN_UART_HANDLE_PTR, rxData,
			RX_BUFFER_SIZE);
			HAL_LIN_SendBreak(LIN_UART_HANDLE_PTR);
			HAL_UART_Transmit(LIN_UART_HANDLE_PTR, txData, 2, LIN_SEND_PERIOD);
		}

		// Diagnostic data reading
		if (lin_data_received_flag == 1)
		{
			lin_data_received_flag = 0;

			drive_voltage = ((float) (rxDataShadow[8])) / VOLTAGE_SCALING;

			if (rxDataShadow[5] != 0xFE)
			{
				drive_current = ((float) (rxDataShadow[5])) / CURRENT_SCALING;
			}
			else
			{
				drive_current = 0;
			}

			HAL_RTC_GetTime(&hrtc, &timeNow, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &dateNow, RTC_FORMAT_BIN);
			// https://msalamon.pl/a-jak-to-jest-z-tym-rtc-na-stm32f4/
			float timeNow_Milliseconds = (timeNow.SecondFraction
					- timeNow.SubSeconds) / ((float) timeNow.SecondFraction + 1)
					* 1000;
			sprintf((char*) uart_line,
					"[%02d:%02d:%02d:%03.0f] I = %2.1f A,  U = %3.1f V, w = %4.0f rpm (w_ref = %4.0f rpm)\r\n",
					timeNow.Hours, timeNow.Minutes, timeNow.Seconds,
					timeNow_Milliseconds, drive_current, drive_voltage,
					((float) (rxDataShadow[6])) * SPEED_SCALING,
					((float) ref_speed) * SPEED_SCALING);
			printf((char*) uart_line);

			ssd1306_SetCursor(2, 46);
			if (low_voltage_flag == 0)
			{
				sprintf((char*) lcd_line, "Speed: %.0f/%.0f rpm   ",
						((float) (rxDataShadow[6])) * SPEED_SCALING,
						((float) ref_speed) * SPEED_SCALING);
			}
			else
			{
				sprintf((char*) lcd_line, "Ref. speed: %.0f (UVP)  ",
						((float) ref_speed) * SPEED_SCALING);
			}
			ssd1306_WriteString((char*) lcd_line, Font_6x8, White);

			ssd1306_SetCursor(2, 56);
			sprintf((char*) lcd_line, "I: %.1f A,  U: %.1f V   ", drive_current,
					drive_voltage);
			ssd1306_WriteString((char*) lcd_line, Font_6x8, White);
			ssd1306_UpdateScreen();

			if (drive_voltage < BATTERY_VOLTAGE_PROTECTION)
			{
				low_voltage_flag = 1;
			}
		}

		// Voice recognition module
		if (HAL_GetTick() - VoiceRecognitionSoftTimer > VOICE_CHECK_PERIOD)
		{
			VoiceRecognitionSoftTimer = HAL_GetTick();
			voice_cmd_id = DF2301G_GetCommandID();
			switch (voice_cmd_id)
			{
			case VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_ONE:
				__HAL_TIM_SET_COUNTER(&htim2, ENC_KNOB_CNT_MAX / 3);
				printf("Voice command: %s\r\n",
				VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_ONE_STRING);
				break;
			case VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_TWO:
				__HAL_TIM_SET_COUNTER(&htim2, ENC_KNOB_CNT_MAX / 3 * 2);
				printf("Voice command: %s\r\n",
				VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_TWO_STRING);
				break;
			case VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_THREE:
				__HAL_TIM_SET_COUNTER(&htim2, ENC_KNOB_CNT_MAX);
				printf("Voice command: %s\r\n",
				VOICE_CMD_TURN_FAN_SPEED_TO_GEAR_THREE_STRING);
				break;
			default:
				__NOP();
			}
		}
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static uint8_t event_counter = 0;
	if (huart->Instance == LIN_UART_INSTANCE)
	{
		if (rxData[2] == Pid_Calc(HEATER_BLOWER_DIAGNOSTICS_LIN_MSG_ID))
		{
			memcpy(rxDataShadow, rxData, sizeof(rxData) * sizeof(rxData[0]));
			memset(rxData, 0x00, RX_BUFFER_SIZE);
			lin_data_received_flag = 1;
			event_counter++;
			event_counter %= 8;
			if (event_counter == 0)
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			}
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		if (__HAL_TIM_GET_COUNTER(&htim2) < ENC_KNOB_CNT_MIN)
		{
			__HAL_TIM_SET_COUNTER(&htim2, ENC_KNOB_CNT_MIN);
		}
		else if (__HAL_TIM_GET_COUNTER(&htim2) > ENC_KNOB_CNT_MAX)
		{
			__HAL_TIM_SET_COUNTER(&htim2, ENC_KNOB_CNT_MAX);
		}
	}
}

uint8_t Pid_Calc(uint8_t ID)
{
	if (ID > 0x3F)
		Error_Handler();
	uint8_t IDBuf[6];
	for (int i = 0; i < 6; i++)
	{
		IDBuf[i] = (ID >> i) & 0x01;
	}

	uint8_t P0 = (IDBuf[0] ^ IDBuf[1] ^ IDBuf[2] ^ IDBuf[4]) & 0x01;
	uint8_t P1 = ~((IDBuf[1] ^ IDBuf[3] ^ IDBuf[4] ^ IDBuf[5]) & 0x01);

	ID = ID | (P0 << 6) | (P1 << 7);
	return ID;
}

uint8_t Checksum_Calc(uint8_t PID, uint8_t *data, uint8_t size)
{
	uint8_t buffer[size + 2];
	uint16_t sum = 0;
	buffer[0] = PID;
	for (int i = 0; i < size; i++)
	{
		buffer[i + 1] = data[i];
	}

	for (int i = 0; i < size + 1; i++)
	{
		sum = sum + buffer[i];
		if (sum > 0xff)
			sum = sum - 0xff;
	}

	sum = 0xff - sum;
	return sum;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

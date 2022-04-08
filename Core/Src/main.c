/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "shell.h"
#include "VL53L1X_api.h"
#include "VL53l1X_calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct VL53L1X_xshut{
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} VL53L1X_xshut;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NB_SENSORS 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

VL53L1X_xshut xshut[2] = {
		{GPIOA, GPIO_PIN_0},
		{GPIOC, GPIO_PIN_0},
};

int status = 0;
uint16_t dev[2];			// device i2c address
uint16_t Distance[2];		// distance in millimeters
uint16_t SignalRate[2];
uint16_t AmbientRate[2];
uint16_t SpadNum[2];
uint8_t RangeStatus[2];
uint8_t dataReady[2];
int16_t offset[2];
uint16_t xtalk[2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	shell_init(&huart2);
	printf("lib_vl53l1x_uld\r\n");

	for(int i=0; i<NB_SENSORS; i++){
		dev[i] = 0x29 << 1;
		HAL_GPIO_WritePin(xshut[i].GPIOx, xshut[i].GPIO_Pin, GPIO_PIN_RESET);
	}


	HAL_Delay(500);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	/* Sensors initialization -----------------------------------------------*/

	for(int i=0; i<NB_SENSORS; i++)
	{
		uint8_t sensorState = 0;
		uint8_t byteData;
		uint16_t wordData;

		HAL_GPIO_WritePin(xshut[i].GPIOx, xshut[i].GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);

		status = VL53L1_RdByte(dev[i], 0x010F, &byteData);
		printf("VL53L1X Model_ID[%d]: %X\r\n", i, byteData);
		status = VL53L1_RdByte(dev[i], 0x0110, &byteData);
		printf("VL53L1X Module_Type[%d]: %X\r\n", i, byteData);
		status = VL53L1_RdWord(dev[i], 0x010F, &wordData);
		printf("VL53L1X[%d]: %X\r\n", i, wordData);

		while(sensorState == i){
			status = VL53L1X_BootState(dev[i], &sensorState);
			printf("sensorState: %d\r\n", sensorState);
			HAL_Delay(500);
		}
		printf("Chip booted\r\n");

		// This function must to be called to initialize the sensor with the default setting
		status = VL53L1X_SensorInit(dev[i]);
		printf("SensorInit() : %d\r\n", status);

		if(i == 0){
			if((status = VL53L1X_SetI2CAddress(dev[i], 0x56 + i * 3)) == 0){
				dev[i] = 0x56 + i * 3;
				printf("new addr = 0x%2X\r\n", dev[i]);
			}
		}


		// Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances
		status = VL53L1X_SetDistanceMode(dev[i], 1); // 1=short, 2=long
		status = VL53L1X_SetInterMeasurementInMs(dev[i], 200); // in ms, IM must be > = TB
		status = VL53L1X_SetTimingBudgetInMs(dev[i], 200); // in ms possible values [20, 50, 100, 200, 500]
		//status = VL53L1X_SetDistanceThreshold(dev, 500, 10, 1, 0); // config sympa
		status = VL53L1X_SetROI(dev[i], 16, 16); // minimum ROI 4,4

		status = VL53L1X_SetOffset(dev[i], 20); // offset compensation in mm
		//status = VL53L1X_CalibrateOffset(dev, 140, &offset); // may take few second to perform the offset cal
		//status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); // may take few second to perform the xtalk cal

	}

	/*

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	// Sensor 1

	status = VL53L1_RdByte(dev[0], 0x010F, &byteData);
	printf("VL53L1X Model_ID[%d]: %X\r\n", 0, byteData);
	status = VL53L1_RdByte(dev[0], 0x0110, &byteData);
	printf("VL53L1X Module_Type[%d]: %X\r\n", 0, byteData);
	status = VL53L1_RdWord(dev[0], 0x010F, &wordData);
	printf("VL53L1X[%d]: %X\r\n", 0, wordData);

	sensorState = 0;
	while(sensorState == 0){
		status = VL53L1X_BootState(dev[0], &sensorState);
		printf("sensorState: %d\r\n", sensorState);
		HAL_Delay(500);
	}
	printf("Chip booted\r\n");

	// This function must to be called to initialize the sensor with the default setting
	status = VL53L1X_SensorInit(dev[0]);
	printf("SensorInit() : %d\r\n", status);

	HAL_Delay(500);

	if((status = VL53L1X_SetI2CAddress(dev[0], 0x56)) == 0){
		dev[0] = 0x56;
		printf("new addr = 0x%2X\r\n", dev[0]);
	}


	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

	// Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances
	status = VL53L1X_SetDistanceMode(dev[0], 1); // 1=short, 2=long
	status = VL53L1X_SetInterMeasurementInMs(dev[0], 200); // in ms, IM must be > = TB
	status = VL53L1X_SetTimingBudgetInMs(dev[0], 200); // in ms possible values [20, 50, 100, 200, 500]
	//status = VL53L1X_SetDistanceThreshold(dev, 500, 10, 1, 0); // config sympa
	status = VL53L1X_SetROI(dev[0], 16, 16); // minimum ROI 4,4

	status = VL53L1X_SetOffset(dev[0], 20); // offset compensation in mm
	//status = VL53L1X_CalibrateOffset(dev, 140, &offset); // may take few second to perform the offset cal
	//status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); // may take few second to perform the xtalk cal

	//printf("VL53L1X Ultra Lite Driver Example running ...\r\n");

	HAL_Delay(1000);

	// Sensor 2

	status = VL53L1_RdByte(dev[1], 0x010F, &byteData);
	printf("VL53L1X Model_ID[%d]: %X\r\n", 1, byteData);
	status = VL53L1_RdByte(dev[1], 0x0110, &byteData);
	printf("VL53L1X Module_Type[%d]: %X\r\n", 1, byteData);
	status = VL53L1_RdWord(dev[1], 0x010F, &wordData);
	printf("VL53L1X[%d]: %X\r\n", 1, wordData);

	sensorState = 0;
	while(sensorState == 0){
		status = VL53L1X_BootState(dev[1], &sensorState);
		printf("sensorState: %d\r\n", sensorState);
		HAL_Delay(500);
	}
	printf("Chip booted\r\n");

	// This function must to be called to initialize the sensor with the default setting
	status = VL53L1X_SensorInit(dev[1]);
	printf("SensorInit() : %d\r\n", status);

	HAL_Delay(500);

	if((status = VL53L1X_SetI2CAddress(dev[1], dev[1] + 1 + 1)) == 0){
		dev[1] = dev[1] + 1 + 1;
		printf("new addr = 0x%2X\r\n", dev[1]);
	}

	HAL_Delay(500);

	// Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances
	status = VL53L1X_SetDistanceMode(dev[1], 1); // 1=short, 2=long
	status = VL53L1X_SetInterMeasurementInMs(dev[1], 200); // in ms, IM must be > = TB
	status = VL53L1X_SetTimingBudgetInMs(dev[1], 200); // in ms possible values [20, 50, 100, 200, 500]
	//status = VL53L1X_SetDistanceThreshold(dev, 500, 10, 1, 0); // config sympa
	status = VL53L1X_SetROI(dev[1], 16, 16); // minimum ROI 4,4

	status = VL53L1X_SetOffset(dev[1], 20); // offset compensation in mm
	//status = VL53L1X_CalibrateOffset(dev, 140, &offset); // may take few second to perform the offset cal
	//status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); // may take few second to perform the xtalk cal

	//printf("VL53L1X Ultra Lite Driver Example running ...\r\n");

	 */

	HAL_Delay(1000);

	status = VL53L1X_StartRanging(dev[0]); // This function has to be called to enable the ranging
	status = VL53L1X_StartRanging(dev[1]); // This function has to be called to enable the ranging

	while (1)
	{
		for(int j=0; j<NB_SENSORS; j++)
		{
			printf("Dist[%d]: %-10d ", j, Distance[j]);
		}
		printf("\r\n");
		HAL_Delay(200);

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
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == GPIO_PIN_4){

#if (NB_SENSORS >= 1)
		status = VL53L1X_GetRangeStatus(dev[0], &RangeStatus[0]);
		status = VL53L1X_GetDistance(dev[0], &Distance[0]);
		status = VL53L1X_GetSignalRate(dev[0], &SignalRate[0]);
		status = VL53L1X_GetAmbientRate(dev[0], &AmbientRate[0]);
		status = VL53L1X_GetSpadNb(dev[0], &SpadNum[0]);
		//printf("Dist[0]=%u, %u, %u, %u, %u\r\n", Distance[0], RangeStatus[0], SignalRate[0], AmbientRate[0],SpadNum[0]);
		status = VL53L1X_ClearInterrupt(dev[0]); // clear interrupt has to be called to enable next interrupt
#endif
	}

	if(GPIO_Pin == GPIO_PIN_7){

#if (NB_SENSORS >= 2)
		status = VL53L1X_GetRangeStatus(dev[1], &RangeStatus[1]);
		status = VL53L1X_GetDistance(dev[1], &Distance[1]);
		status = VL53L1X_GetSignalRate(dev[1], &SignalRate[1]);
		status = VL53L1X_GetAmbientRate(dev[1], &AmbientRate[1]);
		status = VL53L1X_GetSpadNb(dev[1], &SpadNum[1]);
		//printf("Dist[1]=%u, %u, %u, %u, %u\r\n", Distance[1], RangeStatus[1], SignalRate[1], AmbientRate[1],SpadNum[1]);
		status = VL53L1X_ClearInterrupt(dev[1]); // clear interrupt has to be called to enable next interrupt
#endif
	}

	if(GPIO_Pin == GPIO_PIN_10){
#if (NB_SENSORS >= 3)
		printf("EXTI10-15\r\n");
#endif
	}
}

/* USER CODE END 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

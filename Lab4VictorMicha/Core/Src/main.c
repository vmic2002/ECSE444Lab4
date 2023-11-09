/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */



#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_accelero.h"

//#include "stm32l475e_iot01.h"
//#include "stm32l4xx_hal_conf.h"
//#include "stm32l4xx_it.h"

#include "hsensor.h"
#include "psensor.h"
#include "tsensor.h"

#include "hts221.h"
#include "lps22hb.h"
#include "lis3mdl.h"
#include "lsm6dsl.h"

#include "stm32l475e_iot01_qspi.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart1;

osThreadId ReadSensorTaskHandle;
osThreadId UARTTransmitterHandle;
osThreadId Button_SensorHandle;
osThreadId SensorDataFlashHandle;
/* USER CODE BEGIN PV */

float humidity, pressure;
int16_t magneto[3];//magneto is Pointer on 3 magnetometer values table with magneto[0] = X axis, magneto[1] = Y axis, magneto[2] = Z axis

int16_t accelero[3];//accelero Pointer on 3 angular accelerations table with accelero[0] = X axis, accelero[1] = Y axis, accelero[2] = Z axis


int pressed = 0;//0 -> not pressed, 1 -> pressed
int sensorOutput = 0;//to decide which sensor to display

char buffer[40];
int transmitted = 1; //1 if data already transmitted using UART


uint8_t* hum;
uint8_t* press;

uint8_t* magn;
uint8_t* acc;

const int sampleRateSensorHz = 10;//in Hz -> osDelay(100);//100ms to sample at rate of 10Hz
const int sampleRateSensorMs = 1000/sampleRateSensorHz;


//FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 1024K
const int FLASH_BASE_MEMORY = 134217728; //0x8000000;
const int FLASH_MEMORY_SIZE = 1024000;
uint32_t flashNextAvailableByte = FLASH_BASE_MEMORY;//pointer that keeps track of where to add to flash memory (like the stack pointer) (doesnt take into account that flash mem needs to be erased before written to)

int sensorValsRead = 0;//==1 if and only if sensorVals have been read and all sensor vars updated


int numBlocks = 2;//<=16 -> 16 total blocks in FLASH

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_QUADSPI_Init(void);
void ReadSensorVals(void const * argument);
void StartUARTTransmitter(void const * argument);
void StartButton_Sensor(void const * argument);
void SaveSensorDataToFlash(void const * argument);

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
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_QUADSPI_Init();
	/* USER CODE BEGIN 2 */
	BSP_QSPI_Init();
	BSP_HSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_MAGNETO_Init();
	BSP_ACCELERO_Init();





	//FROM DOC: ->>>Again, flash must be erased before it can be written.


	//block == 64KB

	for (int i=0; i<numBlocks; i++) {
		if (BSP_QSPI_Erase_Block((uint32_t) i*64000)!= QSPI_OK)
			Error_Handler();
		//erases numBlocks blocks from flash
	}





	//------------TESTTTING-----------------------//



	/*float humidity1 = BSP_HSENSOR_ReadHumidity();
	float pressure1 = BSP_PSENSOR_ReadPressure();


	int* hum2;
	 *hum2 = (int) humidity1;
	char* temp = hum2;

	uint8_t *hum1 = malloc(4*sizeof(uint8_t));

	hum1[0] = temp[0];
	hum1[1] = temp[1];
	hum1[2] = temp[2];
	hum1[3] = temp[3];
	//uint8_t *press1 = malloc(4*sizeof(uint8_t));
	//hum1 = (uint8_t*)(&humidity1);
	//press1 = (uint8_t*)(&pressure1);


	sprintf(buffer, "!!!!!!!!!: %d %d %d %d", hum1[0], hum1[1], hum1[2], hum1[3]);

	HAL_UART_Transmit(&huart1,(uint8_t*) buffer, strlen(buffer), 1000);
	 */

	//-------------TESTTTING-----------//

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of ReadSensorTask */
	osThreadDef(ReadSensorTask, ReadSensorVals, osPriorityNormal, 0, 128);
	ReadSensorTaskHandle = osThreadCreate(osThread(ReadSensorTask), NULL);

	/* definition and creation of UARTTransmitter */
	osThreadDef(UARTTransmitter, StartUARTTransmitter, osPriorityIdle, 0, 128);
	UARTTransmitterHandle = osThreadCreate(osThread(UARTTransmitter), NULL);

	/* definition and creation of Button_Sensor */
	osThreadDef(Button_Sensor, StartButton_Sensor, osPriorityIdle, 0, 128);
	Button_SensorHandle = osThreadCreate(osThread(Button_Sensor), NULL);

	/* definition and creation of SensorDataFlash */
	osThreadDef(SensorDataFlash, SaveSensorDataToFlash, osPriorityIdle, 0, 128);
	SensorDataFlashHandle = osThreadCreate(osThread(SensorDataFlash), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */


		humidity = BSP_HSENSOR_ReadHumidity();
		pressure = BSP_PSENSOR_ReadPressure();
		BSP_MAGNETO_GetXYZ(magneto);
		BSP_ACCELERO_AccGetXYZ(accelero);
		HAL_Delay(100);//100ms to sample at rate of 10Hz





		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET) {
			//if button released
			if (pressed) pressed = 0;

		}
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) != GPIO_PIN_SET) {
			//if button pressed

			if (!pressed) {
				pressed = 1;

				char buffer[40];

				if (sensorOutput%4==0){
					sprintf(buffer, "Humidity: %.2f ", humidity);
				} else if (sensorOutput%4==1){
					sprintf(buffer, "Pressure: %.2f ", pressure);
				} else if (sensorOutput%4==2) {
					//	sprintf(buffer, "Magneto: %.2f\n", magneto);
					sprintf(buffer,"Magnetometer Values: %d, %d, %d ", magneto[0], magneto[1], magneto[2]);
				} else {//if (sensorOutput%4==3)
					//	sprintf(buffer, "accelero: %.2f\n", accelero);
					sprintf(buffer,"Accelerometer Values: %d, %d, %d ", accelero[0],accelero[1],accelero[2]);
				}
				sensorOutput++;
				HAL_UART_Transmit(&huart1,(uint8_t*) buffer, strlen(buffer), 1000);
			}
		}


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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 40;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10909CEC;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief QUADSPI Initialization Function
 * @param None
 * @retval None
 */
static void MX_QUADSPI_Init(void)
{

	/* USER CODE BEGIN QUADSPI_Init 0 */

	/* USER CODE END QUADSPI_Init 0 */

	/* USER CODE BEGIN QUADSPI_Init 1 */

	/* USER CODE END QUADSPI_Init 1 */
	/* QUADSPI parameter configuration*/
	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 255;
	hqspi.Init.FifoThreshold = 1;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 1;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN QUADSPI_Init 2 */

	/* USER CODE END QUADSPI_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(REDLED_GPIO_Port, REDLED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GREENLED_GPIO_Port, GREENLED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : REDLED_Pin */
	GPIO_InitStruct.Pin = REDLED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(REDLED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GREENLED_Pin */
	GPIO_InitStruct.Pin = GREENLED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GREENLED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	HAL_GPIO_WritePin(REDLED_GPIO_Port, REDLED_Pin, GPIO_PIN_SET);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ReadSensorVals */
/**
 * @brief  Function implementing the ReadSensorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_ReadSensorVals */
void ReadSensorVals(void const * argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(sampleRateSensorMs);

		if (!sensorValsRead) {

			humidity = BSP_HSENSOR_ReadHumidity();
			pressure = BSP_PSENSOR_ReadPressure();
			BSP_MAGNETO_GetXYZ(magneto);
			BSP_ACCELERO_AccGetXYZ(accelero);



			hum = &humidity;
			press = &pressure;

			magn = magneto;

			acc = accelero;

			/*
			 *hum = (uint8_t) humidity;
			 *press = (uint8_t) pressure;

			magn[0] = (uint8_t) magneto[0];
			magn[1] = (uint8_t) magneto[1];
			magn[2] = (uint8_t) magneto[2];

			acc[0] = (uint8_t) accelero[0];
			acc[1] = (uint8_t) accelero[1];
			acc[2] = (uint8_t) accelero[2];
			 */
			sensorValsRead = 1;
		}
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUARTTransmitter */
/**
 * @brief Function implementing the UARTTransmitter thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUARTTransmitter */
void StartUARTTransmitter(void const * argument)
{
	/* USER CODE BEGIN StartUARTTransmitter */
	/* Infinite loop */
	for(;;)
	{
		osDelay(50);
		if (!transmitted) {
			//print to terminal
			HAL_UART_Transmit(&huart1,(uint8_t*) buffer, strlen(buffer), 1000);
			transmitted = 1;
		}
	}
	/* USER CODE END StartUARTTransmitter */
}

/* USER CODE BEGIN Header_StartButton_Sensor */
/**
 * @brief Function implementing the Button_Sensor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartButton_Sensor */
void StartButton_Sensor(void const * argument)
{
	/* USER CODE BEGIN StartButton_Sensor */
	/* Infinite loop */
	for(;;)
	{
		osDelay(25);//check button pressed every 1/40 second
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_SET) {
			//if button released
			if (pressed) pressed = 0;
		}
		if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) != GPIO_PIN_SET) {
			//if button pressed
			if (!pressed) {
				pressed = 1;
				if (sensorOutput%4==0){
					sprintf(buffer, "Humidity: %d ", (int) humidity);
				} else if (sensorOutput%4==1){
					sprintf(buffer, "Pressure: %d ", (int) pressure);
				} else if (sensorOutput%4==2) {
					//	sprintf(buffer, "Magneto: %.2f\n", magneto);
					sprintf(buffer,"Magnetometer Values: %d, %d, %d ", magneto[0], magneto[1], magneto[2]);
				} else {//if (sensorOutput%4==3)
					//	sprintf(buffer, "accelero: %.2f\n", accelero);
					sprintf(buffer,"Accelerometer Values: %d, %d, %d ", accelero[0],accelero[1],accelero[2]);
				}
				sensorOutput++;
				transmitted = 0;
			}
		}
	}
	/* USER CODE END StartButton_Sensor */
}

/* USER CODE BEGIN Header_SaveSensorDataToFlash */
/**
 * @brief Function implementing the SensorDataFlash thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SaveSensorDataToFlash */
void SaveSensorDataToFlash(void const * argument)
{
	/* USER CODE BEGIN SaveSensorDataToFlash */
	/* Infinite loop */
	for(;;)
	{
		osDelay(sampleRateSensorMs);

		if (sensorValsRead) {
			uint32_t sizeUint8_t = (uint32_t) sizeof(uint8_t);

			//adding total of 20*sizeUint8_t to flash per iteration



			//if (flashNextAvailableByte+20*sizeUint8_t>=FLASH_BASE_MEMORY+FLASH_MEMORY_SIZE || flashNextAvailableByte+20*sizeUint8_t>=64000*numBlocks){break;}




			//if (BSP_QSPI_Write(hum, flashNextAvailableByte, 4*sizeUint8_t) != QSPI_OK)
		//		Error_Handler();

/*

			if (BSP_QSPI_Write(press, (uint32_t) (flashNextAvailableByte+4*sizeUint8_t), 4*sizeUint8_t) != QSPI_OK)
				Error_Handler();

			if (BSP_QSPI_Write(magn, (uint32_t) (flashNextAvailableByte+8*sizeUint8_t), (uint32_t) 6*sizeUint8_t) != QSPI_OK)
				Error_Handler();

			if (BSP_QSPI_Write(acc, (uint32_t) (flashNextAvailableByte+14*sizeUint8_t), (uint32_t) 6*sizeUint8_t) != QSPI_OK)
				Error_Handler();
*/
			//flashNextAvailableByte+= 20*sizeUint8_t;

			sensorValsRead = 0;

		}


	}
	/* USER CODE END SaveSensorDataToFlash */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(REDLED_GPIO_Port, REDLED_Pin, GPIO_PIN_RESET);
	__BKPT();
	//turns on a red LED, and then halts the debugger with a breakpoint instruction


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

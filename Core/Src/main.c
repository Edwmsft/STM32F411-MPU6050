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
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "mpu6050reg.h"

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

/* USER CODE BEGIN PV */

uint8_t MPU6050_Check;

/* Сканер шины I2C */
uint8_t buf[8] = {0};
uint8_t separator[] = " . ";
uint8_t new_line[] = "\r\n";
uint8_t start_text[] = "Scan I2C: \r\n";
uint8_t end_text[] = "\r\nStop scanning";

/* MPU6050 переменные для работы с режимами гироскопа и акселерометра */
uint8_t MPU6050_Aconfig;
uint8_t MPU6050_Gconfig;

/* MPU6050 переменные для работы с данными акселерометра */
uint8_t ARAWtoG;
uint8_t AData[6];
uint8_t Accel_X, Accel_Y, Accel_Z, AX, AY, AZ;

/* MPU6050 переменные для работы с данными гироскопа */
uint8_t GRAWtoDS;
uint8_t GData[6];
uint8_t Gyro_X, Gyro_Y, Gyro_Z, GX, GY, GZ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int _write(int file, char *ptr, int len)
{
	// Для вывода в отладочную консоль.
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar(*ptr++);
	return len;
}

void I2C_Scan(void)
{
	uint8_t row = 0, state;

	HAL_Delay(1000);

	printf(start_text);
	for( uint8_t i=1; i<128; i++ ){
	    state = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);

	    if ( state != HAL_OK ){
	        printf(separator);
	    }

	    else if(state == HAL_OK){
	        printf(buf, "0x%X", i);
	    }

	    if( row == 15 ){
	    	row = 0;
	        printf(new_line);
	    } else
	    	row ++;
	}
	printf(end_text);

	HAL_Delay(1000);
}

void MPU6050_Init(void)
{
	HAL_Delay(500);

	HAL_I2C_Mem_Read(&hi2c1, MPU6050_MAIN_ADDRESS_REG, MPU6050_WHO_AM_I_REG, 1, &MPU6050_Check, 1, 1000);

	if (MPU6050_Check == 0x68)
	{
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_MAIN_ADDRESS_REG, MPU6050_PWR_MGMT_1_REG, 1, 0, 1, 1000);

		HAL_I2C_Mem_Write(&hi2c1, MPU6050_MAIN_ADDRESS_REG, MPU6050_SMPLRT_DIV_REG, 1, 0x07, 1, 1000);

		HAL_I2C_Mem_Write(&hi2c1, MPU6050_MAIN_ADDRESS_REG, MPU6050_ACCEL_CONFIG_REG, 1, &MPU6050_Aconfig, 1, 1000);

		HAL_I2C_Mem_Write(&hi2c1, MPU6050_MAIN_ADDRESS_REG, MPU6050_GYRO_CONFIG_REG, 1, &MPU6050_Aconfig, 1, 1000);

		HAL_Delay(500);

		return 0;
	}

	HAL_Delay(500);

	return 1;
}

void MPU6050_Read_Accel(void)
{
	ARAWtoG = (MPU6050_Aconfig == 0x00) ? 2 * 1024: (MPU6050_Aconfig == 0x08) ? 4 * 1024: (MPU6050_Aconfig == 0x10) ? 8 * 1024: (MPU6050_Aconfig == 0x18) ? 16 * 1024: 0;

	HAL_Delay(500);

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_MAIN_ADDRESS_REG, MPU6050_ACCEL_XOUT_H_REG, 1, AData, 6, 1000);

	Accel_X = (int16_t)(AData[0] << 8 | AData [1]);
	Accel_Y = (int16_t)(AData[2] << 8 | AData [3]);
	Accel_Z = (int16_t)(AData[4] << 8 | AData [5]);

	// Необработанные данные преобразуются в g.

	AX = Accel_X/ARAWtoG;
	AY = Accel_Y/ARAWtoG;
	AZ = Accel_Z/ARAWtoG;
}

void MPU6050_Read_Gyro(void)
{
	GRAWtoDS = (MPU6050_Gconfig == 0x00) ? 131.0: (MPU6050_Gconfig == 0x08) ? 65.5: (MPU6050_Gconfig == 0x10) ? 32.8: (MPU6050_Gconfig == 0x18) ? 16.4: 0;

	HAL_Delay(500);

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_MAIN_ADDRESS_REG, MPU6050_GYRO_XOUT_H_REG, 1, AData, 6, 1000);

	Gyro_X = (int16_t)(GData[0] << 8 | GData [1]);
	Gyro_Y = (int16_t)(GData[2] << 8 | GData [3]);
	Gyro_Z = (int16_t)(GData[4] << 8 | GData [5]);

	// Необработанные данные преобразуются в градусы в секунду.

	GX = Gyro_X/GRAWtoDS;
	GY = Gyro_Y/GRAWtoDS;
	GZ = Gyro_Z/GRAWtoDS;

	printf("AX = %f, AY = %f, AZ = %f");
	printf("GX = %f, GY = %f, GZ = %f"); // Вывод в отладочную консоль по Serial Wire Output.
}

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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  I2C_Scan();

	  MPU6050_Aconfig = 0x00; // 0x00 - 2g; 0x08 - 4g; 0x10 - 8g; 0x18 - 16g.
	  MPU6050_Gconfig = 0x00; // 0x00 - 250 deg/s; 0x08 - 500 deg/s; 0x10 - 1000 deg/s; 0x18 - 2000 deg/s. (Register Map)

	  MPU6050_Init();

	  MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();
	  printf("MPU6050 check: %d \r\n", MPU6050_Check);
	  HAL_Delay(50);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

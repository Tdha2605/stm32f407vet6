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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "stdio.h"
#include "ha_hal.h"

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
SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxBuffer[250];
int g_sys_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void SDIO_SDCard_Test(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  /* USER CODE BEGIN _write */
  for (int i = 0; i < len; i++)
  {
    SEGGER_RTT_Write(0, &ptr[i], 1);
  }
  return len;
  /* USER CODE END _write */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  printf("===== STM32F407 GPS Tracker =====\n");

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
  MX_USART2_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  ha_hal_hardware_delay(5000);
  SDIO_SDCard_Test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ha_hal_gpio_set_level(GPIOA, LED_PIN_1_Pin, GPIO_PIN_SET);
    ha_hal_hardware_delay(1000);
    ha_hal_gpio_set_level(GPIOA, LED_PIN_1_Pin, GPIO_PIN_RESET);
    ha_hal_gpio_set_level(GPIOA, LED_PIN_2_Pin, GPIO_PIN_SET);
    ha_hal_hardware_delay(1000);
    ha_hal_gpio_set_level(GPIOA, LED_PIN_2_Pin, GPIO_PIN_RESET);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 24999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_PIN_1_Pin|LED_PIN_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_PIN_1_Pin LED_PIN_2_Pin */
  GPIO_InitStruct.Pin = LED_PIN_1_Pin|LED_PIN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void SDIO_SDCard_Test(void)
{
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];

  // The entire test is wrapped in a do-while(0) loop to allow for easy breaking on error.
  do
  {
    //------------------[ 1. Mount The SD Card ]--------------------
    // FIX 1: Use the correct mount option '1' for immediate mounting.
    FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status != FR_OK)
    {
      // FIX 2: Use printf directly for efficiency.
      printf("Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
      break;
    }
    printf("SD Card Mounted Successfully!\r\n\n");

    //------------------[ 2. Get & Print The SD Card Size & Free Space ]--------------------
    // FIX 3: Add error checking for f_getfree.
    FR_Status = f_getfree("", &FreeClusters, &FS_Ptr);
    if (FR_Status != FR_OK)
    {
      printf("Error! While getting free space, Error Code: (%i)\r\n", FR_Status);
      break;
    }

    // FIX 4: Correct size calculation for Bytes. Assumes sector size is 512 bytes (FF_MAX_SS).
    TotalSize = (FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 512;
    FreeSpace = FreeClusters * FS_Ptr->csize * 512;

    printf("Total SD Card Size: %lu Bytes (%.2f MB)\r\n", TotalSize, TotalSize / (1024.0 * 1024.0));
    printf("Free SD Card Space: %lu Bytes (%.2f MB)\r\n\n", FreeSpace, FreeSpace / (1024.0 * 1024.0));

    //------------------[ 3. Open A Text File For Write & Write Data ]--------------------
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if (FR_Status != FR_OK)
    {
      printf("Error! While Creating/Opening a new file, Error Code: (%i)\r\n", FR_Status);
      break;
    }
    printf("File 'MyTextFile.txt' created & opened. Writing data...\r\n");

    // Write data using f_puts() and f_write()
    f_puts("Hello from STM32 using f_puts()!\n", &Fil);
    strcpy(RW_Buffer, "Hello from STM32 using f_write()!\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    printf("Data written successfully.\r\n\n");

    // Close the file to save changes
    f_close(&Fil);

    //------------------[ 4. Open The File For Read & Read Its Data ]--------------------
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ);
    if (FR_Status != FR_OK)
    {
      printf("Error! While opening 'MyTextFile.txt' for reading, Error Code: (%i)\r\n", FR_Status);
      break;
    }
    printf("Reading data from 'MyTextFile.txt':\r\n");

    // Read the entire file's data into the buffer
    // Clear buffer before reading
    memset(RW_Buffer, 0, sizeof(RW_Buffer));
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);

    // Ensure the buffer is null-terminated before printing, even if f_read fills it
    if (RWC > 0 && RWC < sizeof(RW_Buffer))
    {
      RW_Buffer[RWC] = '\0';
    }

    printf("-------------------- FILE START --------------------\r\n");
    printf("%s", RW_Buffer);
    printf("--------------------- FILE END ---------------------\r\n\n");

    f_close(&Fil);

    //------------------[ 5. Update The Existing File ]--------------------
    printf("Updating 'MyTextFile.txt'...\r\n");

    // FIX 5: Check f_open result before proceeding.
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_OPEN_APPEND | FA_WRITE);
    if (FR_Status != FR_OK)
    {
      printf("Error! While opening file for update, Error Code: (%i)\r\n", FR_Status);
      break;
    }

    // Write a new line of text to the end of the file
    FR_Status = f_puts("This new line was appended to the file.\r\n", &Fil);
    if (FR_Status < 0) // f_puts returns a negative number on error
    {
      printf("Error! While writing appended data.\r\n");
    }

    f_close(&Fil);
    printf("File updated.\r\n\n");

    // (Read back the updated file)
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ);
    memset(RW_Buffer, 0, sizeof(RW_Buffer));
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);

    printf("Data read from 'MyTextFile.txt' after update:\r\n");
    printf("-------------------- FILE START --------------------\r\n");
    printf("%s", RW_Buffer);
    printf("--------------------- FILE END ---------------------\r\n\n");

    f_close(&Fil);

    //------------------[ 6. Delete The Text File ]--------------------
    // FIX 6: Uncomment and fix the unlink section.
    printf("Deleting 'MyTextFile.txt'...\r\n");
    FR_Status = f_unlink("MyTextFile.txt");
    if (FR_Status != FR_OK)
    {
      printf("Error! While deleting the file, Error Code: (%i)\r\n", FR_Status);
    }
    else
    {
      printf("File deleted successfully.\r\n");
    }

  } while (0);

  //------------------[ 7. Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, SDPath, 0);
  if (FR_Status != FR_OK)
  {
    printf("\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
  }
  else
  {
    printf("\r\nTest complete. SD Card Un-mounted Successfully!\r\n");
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    g_sys_count++;
    printf("System counter: %d\n", g_sys_count);
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
#ifdef USE_FULL_ASSERT
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

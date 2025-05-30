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
#include "ili9341.h"
#include "bitmaps.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "fatfs_sd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define deviceAdress 0x55
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define ROW_BYTES  (320*2)   // 640
#define SECTOR_SZ  512

#define GRID_ROWS 240
#define GRID_COLS 320

#define MAX_WIDTH 320
static uint8_t line_buf[MAX_WIDTH * 2];
#define DATA_PA  (LCD_D0_Pin | LCD_D2_Pin | LCD_D7_Pin)
#define DATA_PB  (LCD_D3_Pin | LCD_D4_Pin | LCD_D5_Pin | LCD_D6_Pin)
#define DATA_PC  (LCD_D1_Pin)

FATFS fs;
FIL   f;
FRESULT fr;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];

uint8_t rx_byte;

#define TOTAL_PARQUEOS 8

typedef struct {
	bool DISPONIBLE;
	bool ActualizarEstado;
} parqueos;

parqueos parqueo[TOTAL_PARQUEOS];

int ParqueosDisponibles = 8;



uint8_t bufferI2C;
uint8_t TX_bufferI2C[] = "s";
uint8_t RX_bufferI2C[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void drawImageFromSD(const char *fileName, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height);

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
	 for (int i = 0; i < TOTAL_PARQUEOS; i++) {
		  		  parqueo[i].ActualizarEstado = true;
		  		  parqueo[i].DISPONIBLE = true;
		  	  }
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
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  LCD_Init();
  LCD_Clear(0x0000);

  MX_USART2_UART_Init();
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

  HAL_Delay(1000);

  char *msg = "Hola PC desde Nucleo!\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  fr = f_mount(&fs, "/", 0);
  	    if (fr == FR_OK)
  	    {
  	    	char errMsg[32];
  	    	snprintf(errMsg, sizeof(errMsg), "MONTADA\n");
  	    	HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), HAL_MAX_DELAY);
  	    }
  	    else if (fr != FR_OK)
  	    {
  	    	char errMsg[32];
  	    	snprintf(errMsg, sizeof(errMsg), "FALLO EL INICIO\n");
  	    	HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), HAL_MAX_DELAY);
  	    }

  	    HAL_Delay(1500);


  	    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  	    HAL_SPI_Init(&hspi1);

  	    HAL_Delay(500);

  	    msg = "R";
  	    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  		drawImageFromSD("PARQUEO.raw", 0, 0, 320, 240);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for (int i = 0; i < TOTAL_PARQUEOS; i++) {
	  		  if(parqueo[i].DISPONIBLE != parqueo[i].ActualizarEstado)
	  		  {
	  			if(parqueo[i].DISPONIBLE)
	  			{
	  				if(i < 4){drawImageFromSD("FONDO 40x80.raw", 20 + 55*i, 20, 40, 80);}
	  				else{drawImageFromSD("FONDO 40x80.raw", 20 + 55*(i-4), 130, 40, 80);}
	  				ParqueosDisponibles++;

	  			}
	  			else
	  			{
	  				if(i < 4){drawImageFromSD("CARRO.raw", 20 + 55*i, 20, 40, 80);}
	  				else{drawImageFromSD("CARRO.raw", 20 + 55*(i-4), 130, 40, 80);}
	  				ParqueosDisponibles--;
	  			}
	  			parqueo[i].ActualizarEstado = parqueo[i].DISPONIBLE;


	  			if(ParqueosDisponibles == 0){drawImageFromSD("FONDO 80x80.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 1){drawImageFromSD("1.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 2){drawImageFromSD("2.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 3){drawImageFromSD("3.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 4){drawImageFromSD("4.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 5){drawImageFromSD("5.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 6){drawImageFromSD("6.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 7){drawImageFromSD("7.raw", 240, 90, 80, 80);}
	  			if(ParqueosDisponibles == 8){drawImageFromSD("8.raw", 240, 90, 80, 80);}


	  			 if ((HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) == GPIO_PIN_RESET) && (parqueo[0].DISPONIBLE == true))
	  			 {
	  				parqueo[0].DISPONIBLE = false;
	  			 }
	  			if ((HAL_GPIO_ReadPin(IR1_GPIO_Port, IR1_Pin) == GPIO_PIN_SET) && (parqueo[0].DISPONIBLE == false))
				 {
					parqueo[0].DISPONIBLE = true;
				 }

	  			if ((HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) == GPIO_PIN_RESET) && (parqueo[1].DISPONIBLE == true))
				 {
					parqueo[1].DISPONIBLE = false;
				 }
				if ((HAL_GPIO_ReadPin(IR2_GPIO_Port, IR2_Pin) == GPIO_PIN_SET) && (parqueo[1].DISPONIBLE == false))
				 {
					parqueo[1].DISPONIBLE = true;
				 }

				if ((HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) == GPIO_PIN_RESET) && (parqueo[2].DISPONIBLE == true))
				 {
					parqueo[2].DISPONIBLE = false;
				 }
				if ((HAL_GPIO_ReadPin(IR3_GPIO_Port, IR3_Pin) == GPIO_PIN_SET) && (parqueo[2].DISPONIBLE == false))
				 {
					parqueo[2].DISPONIBLE = true;
				 }

				if ((HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) == GPIO_PIN_RESET) && (parqueo[3].DISPONIBLE == true))
				 {
					parqueo[3].DISPONIBLE = false;
				 }
				if ((HAL_GPIO_ReadPin(IR4_GPIO_Port, IR4_Pin) == GPIO_PIN_SET) && (parqueo[3].DISPONIBLE == false))
				 {
					parqueo[3].DISPONIBLE = true;
				 }



	  			/*
	  			char filename[16];
	  			snprintf(filename, sizeof(filename), "%u.raw", ParqueosDisponibles);
	  			drawImageFromSD(filename, 240, 90, 80, 80);
				*/

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* SPI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, F_CS_Pin|LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : F_CS_Pin LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = F_CS_Pin|LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IR4_Pin IR3_Pin IR2_Pin IR1_Pin */
  GPIO_InitStruct.Pin = IR4_Pin|IR3_Pin|IR2_Pin|IR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2)
    {
    	if (rx_byte == '1') {parqueo[0].DISPONIBLE = true;}
    	if (rx_byte == '2') {parqueo[0].DISPONIBLE = false;}

    	if (rx_byte == '3') {parqueo[1].DISPONIBLE = true;}
    	if (rx_byte == '4') {parqueo[1].DISPONIBLE = false;}

    	if (rx_byte == '5') {parqueo[2].DISPONIBLE = true;}
    	if (rx_byte == '6') {parqueo[2].DISPONIBLE = false;}

    	if (rx_byte == '7') {parqueo[3].DISPONIBLE = true;}
    	if (rx_byte == '8') {parqueo[3].DISPONIBLE = false;}

    	if (rx_byte == 'a') {parqueo[4].DISPONIBLE = true;}
		if (rx_byte == 's') {parqueo[4].DISPONIBLE = false;}

		if (rx_byte == 'd') {parqueo[5].DISPONIBLE = true;}
		if (rx_byte == 'f') {parqueo[5].DISPONIBLE = false;}

		if (rx_byte == 'g') {parqueo[6].DISPONIBLE = true;}
		if (rx_byte == 'h') {parqueo[6].DISPONIBLE = false;}

		if (rx_byte == 'j') {parqueo[7].DISPONIBLE = true;}
		if (rx_byte == 'k') {parqueo[7].DISPONIBLE = false;}

    	    }


        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

}

void drawImageFromSD(const char *fileName,
                     uint16_t x0, uint16_t y0,
                     uint16_t width, uint16_t height)
{
    FIL     file;
    FRESULT fr;
    UINT    br;

    fr = f_open(&file, fileName, FA_OPEN_EXISTING | FA_READ);
    if (fr != FR_OK) return;

    // bytes reales por fila según el ancho
    uint16_t row_bytes = width * 2;

    for (uint16_t row = 0; row < height; row++) {
        // si row_bytes cabe en un solo sector, léelo de una vez:
        if (row_bytes <= SECTOR_SZ) {
            fr = f_read(&file, line_buf, row_bytes, &br);
            if (fr != FR_OK || br != row_bytes) break;
        } else {
            // caso ancho grande (320×) en dos trozos
            fr = f_read(&file, line_buf, SECTOR_SZ, &br);
            if (fr != FR_OK || br != SECTOR_SZ) break;
            fr = f_read(&file,
                        line_buf + SECTOR_SZ,
                        row_bytes - SECTOR_SZ,
                        &br);
            if (fr != FR_OK || br != row_bytes - SECTOR_SZ) break;
        }

        // swap sólo sobre los row_bytes leídos
        for (uint32_t i = 0; i < row_bytes; i += 2) {
            uint8_t tmp   = line_buf[i];
            line_buf[i]   = line_buf[i+1];
            line_buf[i+1] = tmp;
        }

        // dibuja exactamente width×1 fila
        LCD_BitmapFast(x0, y0 + row,
                       width, 1,
                       line_buf);
    }

    f_close(&file);
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

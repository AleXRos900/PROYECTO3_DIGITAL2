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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t buffer_tx[1];
volatile uint8_t i2c_pendiente_envio = 0;


volatile uint8_t parqueo_ocupado = 0;
volatile uint8_t espacios_libres = 8;
const uint16_t pines_segmento[7] = {
    LED_A_Pin, LED_B_Pin, LED_C_Pin, LED_D_Pin,
    LED_E_Pin, LED_F_Pin, LED_G_Pin
};
const uint8_t segmentos[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void mostrar_en_display(uint8_t numero)
{
    if (numero > 9) return; // límite de visualización

    uint8_t valor = segmentos[numero];

    for (int i = 0; i < 7; i++) {
        HAL_GPIO_WritePin(LED_A_GPIO_Port, pines_segmento[i], (valor >> i) & 0x01);
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    buffer_tx[0] = espacios_libres;
}
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
  /* USER CODE BEGIN 2 */
  MX_I2C1_Init();
  if (HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK) {
      Error_Handler();
  }

  //HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_LISTEN_COMPLETE_CB_ID, HAL_I2C_ListenCpltCallback);

  //mostrar_en_display(8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	      HAL_Delay(200);
	  uint8_t ocupados = 0;

	  if (HAL_GPIO_ReadPin(PARQUEO1_GPIO_Port, PARQUEO1_Pin) == GPIO_PIN_RESET)
	      ocupados++;

	  if (HAL_GPIO_ReadPin(PARQUEO2_GPIO_Port, PARQUEO2_Pin) == GPIO_PIN_RESET)
	      ocupados++;

	  if (HAL_GPIO_ReadPin(PARQUEO3_GPIO_Port, PARQUEO3_Pin) == GPIO_PIN_RESET)
	 	      ocupados++;

	 	  if (HAL_GPIO_ReadPin(PARQUEO4_GPIO_Port, PARQUEO4_Pin) == GPIO_PIN_RESET)
	 	      ocupados++;
///////leds
	 	 if (HAL_GPIO_ReadPin(PARQUEO1_GPIO_Port, PARQUEO1_Pin) == GPIO_PIN_RESET)
	 	     {
	 	         HAL_GPIO_WritePin(GPIOA, R1_Pin, GPIO_PIN_SET);   // Rojo encendido
	 	         HAL_GPIO_WritePin(GPIOA, G1_Pin, GPIO_PIN_RESET); // Verde apagado
	 	     }
	 	     else
	 	     {
	 	         HAL_GPIO_WritePin(GPIOA, R1_Pin, GPIO_PIN_RESET); // Rojo apagado
	 	         HAL_GPIO_WritePin(GPIOA, G1_Pin, GPIO_PIN_SET);   // Verde encendido
	 	     }

	 	 //LED DEL PARQUEO 2//
	 	if (HAL_GPIO_ReadPin(PARQUEO2_GPIO_Port, PARQUEO2_Pin) == GPIO_PIN_RESET)
	 		 	     {
	 		 	         HAL_GPIO_WritePin(GPIOC, R2_Pin, GPIO_PIN_SET);   // Rojo encendido
	 		 	         HAL_GPIO_WritePin(GPIOC, G2_Pin, GPIO_PIN_RESET); // Verde apagado
	 		 	     }
	 		 	     else
	 		 	     {
	 		 	         HAL_GPIO_WritePin(GPIOC, R2_Pin, GPIO_PIN_RESET); // Rojo apagado
	 		 	         HAL_GPIO_WritePin(GPIOC, G2_Pin, GPIO_PIN_SET);   // Verde encendido
	 		 	     }
	 	 //LED DEL PARQUEO 2//
	 		 	if (HAL_GPIO_ReadPin(PARQUEO3_GPIO_Port, PARQUEO3_Pin) == GPIO_PIN_RESET)
	 		 		 	     {
	 		 		 	         HAL_GPIO_WritePin(GPIOC, R3_Pin, GPIO_PIN_SET);   // Rojo encendido
	 		 		 	         HAL_GPIO_WritePin(GPIOC, G3_Pin, GPIO_PIN_RESET); // Verde apagado
	 		 		 	     }
	 		 		 	     else
	 		 		 	     {
	 		 		 	         HAL_GPIO_WritePin(GPIOC, R3_Pin, GPIO_PIN_RESET); // Rojo apagado
	 		 		 	         HAL_GPIO_WritePin(GPIOC, G3_Pin, GPIO_PIN_SET);   // Verde encendido
	 		 		 	     }
	 			 //LED DEL PARQUEO 2//
	 			 		 	if (HAL_GPIO_ReadPin(PARQUEO4_GPIO_Port, PARQUEO4_Pin) == GPIO_PIN_RESET)
	 			 		 		 	     {
	 			 		 		 	         HAL_GPIO_WritePin(GPIOA, R4_Pin, GPIO_PIN_SET);   // Rojo encendido
	 			 		 		 	         HAL_GPIO_WritePin(GPIOA, G4_Pin, GPIO_PIN_RESET); // Verde apagado
	 			 		 		 	     }
	 			 		 		 	     else
	 			 		 		 	     {
	 			 		 		 	         HAL_GPIO_WritePin(GPIOA, R4_Pin, GPIO_PIN_RESET); // Rojo apagado
	 			 		 		 	         HAL_GPIO_WritePin(GPIOA, G4_Pin, GPIO_PIN_SET);   // Verde encendido
	 			 		 		 	 }



	  espacios_libres = 8 - ocupados;
	  mostrar_en_display(espacios_libres);


	      HAL_Delay(200); // para evitar parpadeo y lectura muy rápida
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 16;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_A_Pin|LED_B_Pin|LED_C_Pin|LED_D_Pin
                          |LED_E_Pin|LED_F_Pin|LED_G_Pin|G2_Pin
                          |R2_Pin|R3_Pin|G3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|G1_Pin|R1_Pin|G4_Pin
                          |R4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PARQUEO3_Pin */
  GPIO_InitStruct.Pin = PARQUEO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PARQUEO3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_A_Pin LED_B_Pin LED_C_Pin LED_D_Pin
                           LED_E_Pin LED_F_Pin LED_G_Pin G2_Pin
                           R2_Pin R3_Pin G3_Pin */
  GPIO_InitStruct.Pin = LED_A_Pin|LED_B_Pin|LED_C_Pin|LED_D_Pin
                          |LED_E_Pin|LED_F_Pin|LED_G_Pin|G2_Pin
                          |R2_Pin|R3_Pin|G3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PARQUEO1_Pin PARQUEO2_Pin */
  GPIO_InitStruct.Pin = PARQUEO1_Pin|PARQUEO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin G1_Pin R1_Pin G4_Pin
                           R4_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|G1_Pin|R1_Pin|G4_Pin
                          |R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PARQUEO4_Pin */
  GPIO_InitStruct.Pin = PARQUEO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PARQUEO4_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    static uint8_t last_state_entrada = 1;
    static uint8_t last_state_salida = 1;

    if (GPIO_Pin == PARQUEO1_Pin) // Entrada (PA0)
    {
        uint8_t current_state = HAL_GPIO_ReadPin(PARQUEO1_GPIO_Port, PARQUEO1_Pin);
        if (last_state_entrada == 1 && current_state == 0) // flanco de bajada
        {
            if (espacios_libres > 0)
                espacios_libres--;
        }
        last_state_entrada = current_state;
    }
    else if (GPIO_Pin == PARQUEO2_Pin) // Salida (PA1)
    {
        uint8_t current_state = HAL_GPIO_ReadPin(PARQUEO2_GPIO_Port, PARQUEO2_Pin);
        if (last_state_salida == 1 && current_state == 0) // flanco de bajada
        {
            if (espacios_libres < 8)
                espacios_libres++;
        }
        last_state_salida = current_state;
    }

    mostrar_en_display(espacios_libres);
}*/



void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c){
    HAL_I2C_EnableListen_IT(hi2c);  // Reinicia modo escucha
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        buffer_tx[0] = espacios_libres;
        HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, buffer_tx, 1, I2C_FIRST_AND_LAST_FRAME);
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
  while (1);
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

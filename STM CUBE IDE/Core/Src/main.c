/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t menu, start;
uint8_t inc, dec, enter, aux_init;
uint8_t aux_menu, aux_start, accel_time, set_ok;
uint8_t aux_accel, aux_running, aux_decel, config_finish;
uint8_t a_inc, r_inc, d_inc, inc_running, inc_decel;
uint8_t config_init, start_init, pwm;
uint8_t pwm_preset, aux_pwmPreset, motor;

char buffer_accel [15] = {0x00};
char buffer_running [15] = {0x00};
char buffer_decel [15] = {0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  motor = 0;
  void Buzzer_Teclas();
  void ENTER_Boucing();
  void DEC_Boucing();
  void INC_Boucing();
  void start_preset();
  void start_config();
  void sys_init();
  void set_converter();
  void converter_preset();

  sys_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if ((start == 1) && (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin)))
		{
			ENTER_Boucing();
			Buzzer_Teclas();
			start = 3;
			converter_preset();
		}

		if (HAL_GPIO_ReadPin(INC_GPIO_Port, INC_Pin) && (start == 1))
		{
			INC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			menu = 1;
			start = 0;
		}

		if ((menu == 1) && (HAL_GPIO_ReadPin(DEC_GPIO_Port, DEC_Pin)))
		{
			DEC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			menu = 0;
			start = 2;
		}

		if ((aux_start == 1) && (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin)))
		{
			ENTER_Boucing();
			Buzzer_Teclas();
			start = 3;
			aux_start = 0;
			converter_preset();
		}

		if ((aux_menu == 1) && (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin)))
		{
			ENTER_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			menu = 2;
			aux_menu = 0;
			a_inc = 5;
			sprintf(buffer_accel, "%d", a_inc);
		}

		if ((aux_accel == 1) && (HAL_GPIO_ReadPin(INC_GPIO_Port, INC_Pin) && (a_inc <= 80)))
		{
			aux_menu =0;
			INC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			a_inc += 5;
			sprintf(buffer_accel, "%d", a_inc);
			menu = 3;
		}
		else if (((aux_accel == 1) && (HAL_GPIO_ReadPin(DEC_GPIO_Port, DEC_Pin) && (a_inc <= 80) && (a_inc >5))))
		{
			aux_menu = 0;
			DEC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			a_inc -= 5;
			sprintf(buffer_accel, "%d", a_inc);
			menu = 3;
		}

		if ((aux_running == 1) && (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin)))
		{
			a_inc = a_inc;
			aux_accel = 0;
			ENTER_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			menu = 4;
			r_inc = 5;
			sprintf(buffer_running, "%d", r_inc);
		}

		if ((inc_running == 1) && (HAL_GPIO_ReadPin(INC_GPIO_Port, INC_Pin) && (r_inc <= 180)))
		{
			aux_running = 0;
			INC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			r_inc += 5;
			sprintf(buffer_running, "%d", r_inc);
			menu = 5;
		}
		else if ((inc_running == 1) && (HAL_GPIO_ReadPin(DEC_GPIO_Port, DEC_Pin) && (r_inc <= 180) && (r_inc > 5)))
		{
			aux_running = 0;
			DEC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			r_inc -= 5;
			sprintf(buffer_running, "%d", r_inc);
			menu = 5;
		}

		if ((aux_decel == 1) && (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin)))
		{
			r_inc = r_inc;
			inc_running = 0;
			ENTER_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			menu = 6;
			d_inc = 5;
			sprintf(buffer_decel, "%d", d_inc);
		}

		if ((inc_decel == 1) && (HAL_GPIO_ReadPin(INC_GPIO_Port, INC_Pin) && (d_inc <= 80)))
		{
			aux_decel = 0;
			INC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			d_inc += 5;
			sprintf(buffer_decel, "%d", d_inc);
			menu = 7;
		}
		else if ((inc_decel == 1) && (HAL_GPIO_ReadPin(DEC_GPIO_Port, DEC_Pin) && (d_inc <= 80) && (d_inc > 5)))
		{
			aux_decel = 0;
			DEC_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			d_inc -= 5;
			sprintf(buffer_decel, "%d", d_inc);
			menu = 7;
		}

		if ((config_finish == 1) && (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin)))
		{
			d_inc = d_inc;
			inc_decel = 0;
			ENTER_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			start = 4;
			menu = 0;
		}

		if ((HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin)) && (config_init = 1))
		{
			ENTER_Boucing();
			Buzzer_Teclas();
			LCD_Clear();
			set_converter();
			start = 5;
		}

		switch (menu)
		{
		case 1:
			LCD_Cursor(0, 2);
			LCD_String("DC MOTOR SYS");
			LCD_Cursor(1, 0);
			LCD_String("<     MENU    ");
			aux_menu = 1;
			break;

		case 2:
			LCD_Cursor(0, 0);
			LCD_String("   ACCEL TIME   ");
			LCD_Cursor(1, 7);
			LCD_String(buffer_accel);
			LCD_Cursor(1, 8);
			LCD_String("s");
			aux_accel = 1;
			break;

		case 3:
			LCD_Cursor(0, 0);
			LCD_String("   ACCEL TIME   ");
			LCD_Cursor(1, 6);
			LCD_String(buffer_accel);
			LCD_Cursor(1, 8);
			LCD_String("s");
			aux_running = 1;
			break;

		case 4:
			LCD_Cursor(0, 0);
			LCD_String("  RUNNING TIME  ");
			LCD_Cursor(1, 7);
			LCD_String(buffer_running);
			LCD_Cursor(1, 8);
			LCD_String("s");
			inc_running = 1;
			break;

		case 5:
			LCD_Cursor(0, 0);
			LCD_String("  RUNNING TIME  ");
			LCD_Cursor(1, 6);
			LCD_String(buffer_running);
			LCD_Cursor(1, 9);
			LCD_String("s");
			aux_decel = 1;
			break;

		case 6:
			LCD_Cursor(0, 0);
			LCD_String("   DECEL TIME   ");
			LCD_Cursor(1, 7);
			LCD_String(buffer_decel);
			LCD_Cursor(1, 8);
			LCD_String("s");
			inc_decel = 1;
			break;

		case 7:
			LCD_Cursor(0, 0);
			LCD_String("   DECEL TIME   ");
			LCD_Cursor(1, 6);
			LCD_String(buffer_decel);
			LCD_Cursor(1, 8);
			LCD_String("s");
			config_finish = 1;
			break;
		}

		switch (start)
		{

		case 1:
			LCD_Cursor(0, 2);
			LCD_String("DC MOTOR SYS");
			LCD_Cursor(1, 0);
			LCD_String("     START     >");
			break;

		case 2:
			LCD_Cursor(0, 2);
			LCD_String("DC MOTOR SYS");
			LCD_Cursor(1, 0);
			LCD_String("     START     >");
			aux_start = 1;
			break;

		case 3:
			aux_pwmPreset = 1;
			start_preset();
			break;

		case 4:
			LCD_Cursor(0, 2);
			LCD_String("DC MOTOR SYS");
			LCD_Cursor(1, 0);
			LCD_String("     START     >");
			config_init = 1;
			config_finish = 0;
			break;

		case 5:
			start_config();
			break;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
}
	void Buzzer_Teclas()
	{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		HAL_Delay(50);
	}

	void INC_Boucing()
	{
		HAL_Delay(10);
		while (HAL_GPIO_ReadPin(INC_GPIO_Port, INC_Pin));
	}
	void DEC_Boucing()
	{
		HAL_Delay(10);
		while (HAL_GPIO_ReadPin(DEC_GPIO_Port, DEC_Pin));
	}
	void ENTER_Boucing()
	{
		HAL_Delay(10);
		while (HAL_GPIO_ReadPin(ENTER_GPIO_Port, ENTER_Pin));
	}

	void start_preset()
	{
	LCD_Clear();
	LCD_Cursor(0, 0);
	LCD_String("STATUS:    ACCEL");
	LCD_Cursor(1, 0);
	LCD_String("RPM: 1000");
	LCD_Cursor(1, 11);
	LCD_String("t:");
	LCD_Cursor(1, 13);
	LCD_String(buffer_accel);
	LCD_Cursor(1, 15);
	LCD_String("s");
	HAL_Delay(a_inc * 1000);
	LCD_Clear();

	LCD_Cursor(0, 0);
	LCD_String("STATUS:  RUNNING");
	LCD_Cursor(1, 0);
	LCD_String("RPM: 1000");
	LCD_Cursor(1, 10);
	LCD_String("t:");
	LCD_Cursor(1, 12);
	LCD_String(buffer_running);
	LCD_Cursor(1, 15);
	LCD_String("s");
	HAL_Delay(r_inc * 1000);
	LCD_Clear();

	LCD_Cursor(0, 0);
	LCD_String("STATUS:    DECEL");
	LCD_Cursor(1, 0);
	LCD_String("RPM: 1000");
	LCD_Cursor(1, 11);
	LCD_String("t:");
	LCD_Cursor(1, 13);
	LCD_String(buffer_decel);
	LCD_Cursor(1, 15);
	LCD_String("s");
	HAL_Delay(d_inc * 1000);
	LCD_Clear();
	start = 1;
	}

	void start_config()
	{
	LCD_Cursor(0, 0);
	LCD_String("STATUS:    ACCEL");
	LCD_Cursor(1, 0);
	LCD_String("RPM: 1000");
	LCD_Cursor(1, 11);
	LCD_String("t:");
	LCD_Cursor(1, 13);
	LCD_String(buffer_accel);
	LCD_Cursor(1, 15);
	LCD_String("s");
	HAL_Delay(a_inc * 1000);
	LCD_Clear();

	LCD_Cursor(0, 0);
	LCD_String("STATUS:  RUNNING");
	LCD_Cursor(1, 0);
	LCD_String("RPM: 1000");
	LCD_Cursor(1, 10);
	LCD_String("t:");
	LCD_Cursor(1, 12);
	LCD_String(buffer_running);
	LCD_Cursor(1, 15);
	LCD_String("s");
	HAL_Delay(r_inc * 1000);
	LCD_Clear();

	LCD_Cursor(0, 0);
	LCD_String("STATUS:    DECEL");
	LCD_Cursor(1, 0);
	LCD_String("RPM: 1000");
	LCD_Cursor(1, 11);
	LCD_String("t:");
	LCD_Cursor(1, 13);
	LCD_String(buffer_decel);
	LCD_Cursor(1, 15);
	LCD_String("s");
	HAL_Delay(d_inc * 1000);
	LCD_Clear();
	start = 4;
	}

	void sys_init()
	{
	LCD_Init();
	LCD_Cursor(0, 2);
	LCD_String("DC MOTOR SYS");
	LCD_Cursor(1, 2);
	LCD_String("VERSION: 1.0");
	HAL_Delay(3000);
	LCD_Clear();
	start = 1;
	}

	void set_converter()
	{
	a_inc = a_inc;
	sprintf(buffer_accel, "%d", a_inc);
	r_inc = r_inc;
	sprintf(buffer_running, "%d", r_inc);
	d_inc = d_inc;
	sprintf(buffer_decel, "%d", d_inc);
	}

	void converter_preset()
	{
	a_inc = 10;
	sprintf(buffer_accel, "%d", a_inc);
	r_inc = 120;
	sprintf(buffer_running, "%d", r_inc);
	d_inc = 10;
	sprintf(buffer_decel, "%d", d_inc);
	}
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_Pin|BUZZER_Pin|RS_Pin|D4_Pin
                          |D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin BUZZER_Pin RS_Pin D4_Pin
                           D5_Pin */
  GPIO_InitStruct.Pin = EN_Pin|BUZZER_Pin|RS_Pin|D4_Pin
                          |D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D6_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TACHOMETER_Pin */
  GPIO_InitStruct.Pin = TACHOMETER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TACHOMETER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DEC_Pin ENTER_Pin INC_Pin */
  GPIO_InitStruct.Pin = DEC_Pin|ENTER_Pin|INC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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

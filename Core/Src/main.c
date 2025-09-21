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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Thermistor.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "PID.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GPIO_LED GPIOB
#define PIN_LED_R GPIOB, GPIO_PIN_9
#define PIN_LED_G GPIOB, GPIO_PIN_10
#define PIN_LED_B GPIOB, GPIO_PIN_11

#define GPIO_BUTTON GPIOB
#define PIN_BUTTON_SELECT GPIOB, GPIO_PIN_0
#define PIN_BUTTON_LEFT GPIOB, GPIO_PIN_1
#define PIN_BUTTON_RIGHT GPIOB, GPIO_PIN_2

#define GPIO_HEATER GPIOA
#define PIN_HEATER GPIOA, GPIO_PIN_5
#define PIN_FAN GPIO_PIN_3

#define MAX_TEMP 80

#define DISP_FREQ 50 // Frequency of the display refreshing in milliseconds
#define CALC_FREQ 10 // Frequency of the thermal and input procesing in milliseconds.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

typedef struct {
	uint8_t poll;
	uint32_t last_press;
} button;
#define INIT_BUTTON(X) button X = {.poll = 0, .last_press = 0}
INIT_BUTTON(button_left);
INIT_BUTTON(button_right);
INIT_BUTTON(button_select);

uint8_t PWM_Counter = 0;
uint8_t PWM_Value = 0;

float targetTemp = 0.0;

char *filament_names[] = {"PLA", "PETG/ABS", "Nylon"};
float filament_temps[] = {55.0f, 65.0f, 75.0f};
uint8_t menu_index = 0;
int menu_num = 0;
uint8_t menu_size = 0;

char text[10];

enum {
		MAIN_MENU,
		HEATING
} program_state;

uint32_t last_screen_update = 0, last_math_update = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static inline void HEATERPWM_Process();
static inline void HEATERPWM_SET(uint16_t duty_cycle);
static inline bool is_pressed(button button);
static inline void update_screen();
static inline void update_inputs();
static inline void Therm_Process();
static inline uint32_t since_last(uint32_t last_time);

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Initialize TIM3 for low frequency PWM heater
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(PIN_LED_R, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PIN_LED_G, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PIN_LED_B, GPIO_PIN_SET);


  HAL_Delay(1000); // Wait for power supply to stabilize
  HAL_ADCEx_Calibration_Start(&hadc1);


  Thermistor_Init();
  ssd1306_Init();
  
  // Menu Init.
  program_state = MAIN_MENU;
  menu_size = sizeof(filament_names)/sizeof(filament_names[0]);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(since_last(last_screen_update) >= DISP_FREQ) { // Update screen everyy 50ms
		  update_screen();
		  last_screen_update = HAL_GetTick();
	  }
	  if(since_last(last_math_update) >= CALC_FREQ ) { // Process inputs and set PWM output every 10ms
		  update_inputs();
		  Therm_Process();
		  last_math_update = HAL_GetTick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2823 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void update_screen() {
	  long uint32_t =  HAL_GetTick();

    // Heating Menu
	  if(program_state == HEATING) {
		  ssd1306_Fill(Black); 
		  float temp = Thermistor_get_temp();
		  snprintf(text, 10, "%.2f/%.0f", Thermistor_get_temp(), targetTemp);
		  ssd1306_SetCursor(1, 1);
		  ssd1306_WriteString("Temp: ",Font_6x8, White);
		  ssd1306_SetCursor(50, 1);
		  if(temp < -30 || temp > 100) {
			  ssd1306_WriteString("THERM_ERROR", Font_6x8, White);
		  } else {
			  ssd1306_WriteString(text, Font_6x8, White);
		  }

		  snprintf(text, 10, "%.2f%%", PWM_Value / 255.0);
		  ssd1306_SetCursor(1, 10);
		  ssd1306_WriteString("Heater: ",Font_6x8, White);
		  ssd1306_SetCursor(50, 10);
		  ssd1306_WriteString(text, Font_6x8, White);
    
    // Main Menu
	  } else if (program_state == MAIN_MENU) {

		  ssd1306_Fill(Black);
		  ssd1306_SetCursor(0,0);
		  ssd1306_WriteString("Material:", Font_6x8, White);

		  ssd1306_SetCursor((128/2) - (strlen(filament_names[menu_index]) * 11/2), ((32/2) - (18/2)) + 1);
		  ssd1306_WriteString(filament_names[menu_index], Font_11x18, White);

		  ssd1306_SetCursor((128/2) + (strlen(filament_names[menu_index]) * 11/2) + 2, ((32/2) - (18/2)) + 1);
		  char text[4];
		  snprintf(text, 4, "%.0f°", filament_temps[menu_index]);
		  ssd1306_WriteString(text, Font_7x10, White);
	  }

	  ssd1306_UpdateScreen();
}

void update_inputs() {

	button_left.poll <<= 1;
	button_select.poll  <<= 1;
	button_right.poll   <<= 1;

	button_left.poll  |= HAL_GPIO_ReadPin(PIN_BUTTON_LEFT); // wrong button soldered on LEFT
	button_select.poll  |=   !HAL_GPIO_ReadPin(PIN_BUTTON_SELECT);
	button_right.poll  |=  !HAL_GPIO_ReadPin(PIN_BUTTON_RIGHT);

	if(is_pressed(button_right)) {
		if(HAL_GetTick() - button_right.last_press > 60){
			menu_index++;
			if(menu_index >= menu_size) {
				menu_index = 0;
			}
		}

		button_right.last_press = HAL_GetTick();
	}

	if(is_pressed(button_left)) {
	if(HAL_GetTick() - button_left.last_press > 60){
		if(menu_index == 0) {
			menu_index = menu_size - 1;
		} else {
			menu_index--;
		}
	}
		button_left.last_press = HAL_GetTick();
	}
	if(is_pressed(button_select)) {
		if(HAL_GetTick() - button_select.last_press > 60){
			program_state = !program_state;
			targetTemp = filament_temps[menu_index];
		}
		button_select.last_press = HAL_GetTick();
	}
}

void Therm_Process() {
		HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
		  Thermistor_Process(HAL_ADC_GetValue(&hadc1));
		  float temp = Thermistor_get_temp();

		  if(temp < -30  ||  temp > MAX_TEMP || program_state == MAIN_MENU) {
			 PWM_Value = 0;
		  } else {
			  PWM_Value = 255.0 * PID_GetDutyCycle(temp, targetTemp);
		  }

}

bool is_pressed(button  button) {
	return (button.poll & 0xF) == 0xF;
}

uint32_t since_last(uint32_t last_time) {
	uint32_t now = HAL_GetTick();
	if (last_time > now) {
		return now + (UINT32_MAX  - last_time);
	} else {
		return now - last_time;
	}
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim == &htim3 )
  {
	  PWM_Counter++; // Considering that the counter is an 8-bit integer, it will overflow naturally.
//	  if(PWM_Counter > 255) {
//		  PWM_Counter = 0;
//	  }
	if(PWM_Counter <= PWM_Value) {
		HAL_GPIO_WritePin(PIN_HEATER, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIN_LED_B, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(PIN_HEATER, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PIN_LED_B, GPIO_PIN_SET);
	}
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define PWM_MAX_DUTY      255
#define PWM_MIN_DUTY      50
#define PWM_START_DUTY    100

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint16_t bldc_step=0;
uint16_t motor_speed;
uint8_t duty;
uint16_t bayrak = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void bldc_move(void){        // BLDC motor commutation function
  switch(bldc_step){
    case 0:    //AH_BL
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);							//  SD2=HIGH
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);						//  SD1=low
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);						//  SD3=low
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);						//	IN2,IN3 = LOW	
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);							//  IN1=HIGH  	
			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);												//  SD2 PWM OFF
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);												// 	SD3 PWM OFF
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);											//  SD1 PWM ON
			
			HAL_NVIC_EnableIRQ(EXTI0_IRQn );															// Bemf C yükselen kenar etkin.
			HAL_NVIC_DisableIRQ(EXTI1_IRQn );
			HAL_NVIC_DisableIRQ(EXTI2_IRQn );
			HAL_NVIC_DisableIRQ(EXTI3_IRQn );
			HAL_NVIC_DisableIRQ(EXTI4_IRQn );
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn );		
      break;
      
		
    case 1:    //AH_CL
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);							//  SD3=HIGH
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);						//  SD1=low
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);						//  SD2=low
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);						//	IN2,IN3 = LOW	
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);							//  IN1=HIGH  	
			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);												//  SD2 PWM OFF
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);												// 	SD3 PWM OFF
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);											//  SD1 PWM ON
			
			HAL_NVIC_EnableIRQ(EXTI1_IRQn );											// Bemf B Düsen kenar etkin.
			HAL_NVIC_DisableIRQ(EXTI0_IRQn );
			HAL_NVIC_DisableIRQ(EXTI2_IRQn );
			HAL_NVIC_DisableIRQ(EXTI3_IRQn );
			HAL_NVIC_DisableIRQ(EXTI4_IRQn );
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn );			
      break;
		
      
    case 2: //BH_CL
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);							//  SD3=HIGH
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);						//  SD1=low
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);						//  SD2=low
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);						//	IN1,IN3 = LOW
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);							//  IN2=HIGH  
			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);												//  SD1 PWM OFF
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);												// 	SD3 PWM OFF
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);											//  SD2 PWM ON   

			HAL_NVIC_EnableIRQ(EXTI2_IRQn );															// Bemf A yükselen kenar etkin.		
			HAL_NVIC_DisableIRQ(EXTI0_IRQn );
			HAL_NVIC_DisableIRQ(EXTI1_IRQn );
			HAL_NVIC_DisableIRQ(EXTI3_IRQn );
			HAL_NVIC_DisableIRQ(EXTI4_IRQn );
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn );
      break;
      
    case 3: //BH_AL   
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);							//  SD1 = HIGH
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);						//  SD2=low
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);						//  SD3=low		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);						//	IN1,IN3 = LOW
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);							//  IN2=HIGH  
			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);												//  SD1 PWM OFF
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);												// 	SD3 PWM OFF
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);											//  SD2 PWM ON   
			
			HAL_NVIC_EnableIRQ(EXTI3_IRQn );															// Bemf C Düsen kenar etkin.
			HAL_NVIC_DisableIRQ(EXTI0_IRQn );
			HAL_NVIC_DisableIRQ(EXTI1_IRQn );
			HAL_NVIC_DisableIRQ(EXTI2_IRQn );
			HAL_NVIC_DisableIRQ(EXTI4_IRQn );
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn );			
      break;
      
    case 4:    //CH_AL
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);							//  SD1 = HIGH
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);						//  SD2=low
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);						//  SD3=low
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);						//	IN1,IN2 = LOW
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);							//  IN3=HIGH  
				
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);												//  SD1 PWM OFF
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);												// 	SD2 PWM OFF
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);											//  SD3 PWM ON      
			
			HAL_NVIC_EnableIRQ(EXTI4_IRQn );															// Bemf B yükselen kenar etkin.
			HAL_NVIC_DisableIRQ(EXTI0_IRQn );
			HAL_NVIC_DisableIRQ(EXTI1_IRQn );
			HAL_NVIC_DisableIRQ(EXTI2_IRQn );
			HAL_NVIC_DisableIRQ(EXTI3_IRQn );
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn );
			
      break;
      
    case 5:    //CH_BL
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);							//  SD2 = HIGH
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);						//  SD1=low
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);						//  SD3=low
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);						//	IN1,IN2 = LOW
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);							//  IN3=HIGH  
				
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);												//  SD1 PWM OFF
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);												// 	SD2 PWM OFF
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);											//  SD3 PWM ON
			
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn );														// Bemf A Düsen kenar etkin.
			HAL_NVIC_DisableIRQ(EXTI0_IRQn );
			HAL_NVIC_DisableIRQ(EXTI1_IRQn );
			HAL_NVIC_DisableIRQ(EXTI2_IRQn );
			HAL_NVIC_DisableIRQ(EXTI3_IRQn );
			HAL_NVIC_DisableIRQ(EXTI4_IRQn );
			    
      break;
  }
}

void SET_PWM_DUTY(uint8_t duty)
{
  if(duty < PWM_MIN_DUTY)
    duty  = PWM_MIN_DUTY;
  if(duty > PWM_MAX_DUTY)
    duty  = PWM_MAX_DUTY;
	
  TIM3 ->CCR1  = duty;                   // PA6     SD1 
  TIM3 ->CCR2  = duty;                   // PA7     SD2
  TIM3 ->CCR3  = duty;                   // PB0   	SD3
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
	
  while (1)
  {
		
		SET_PWM_DUTY(PWM_START_DUTY);    // Setup starting PWM with duty cycle = PWM_START_DUTY
  unsigned int i = 5000;
  // Motor start
					while(i > 100) {
   
						HAL_Delay(i/1000); 
							bldc_move();  
							bldc_step++;
							bldc_step %= 6;
							i = i - 20;
						}
  motor_speed = PWM_START_DUTY;
				while(1) {
							while(!(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)) && motor_speed < PWM_MAX_DUTY){
									motor_speed++;
									SET_PWM_DUTY(motor_speed);
									HAL_Delay(100);
								}
							while(!(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)) && motor_speed > PWM_MIN_DUTY){
									motor_speed--;
									SET_PWM_DUTY(motor_speed);
									HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA2 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

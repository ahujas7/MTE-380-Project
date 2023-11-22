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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"

#include "TCS34725.h"
#include "L298N.h"
#include "SG90.h"
#include "HCSR04.h"
// Define P controller constant
#define KP 3.5
#define KD 15
#define KI 0.02
// Define target values

#define TARGET_SPEED_LEFT 40 // Default: 45   RATIO is 12
#define TARGET_SPEED_RIGHT 48 // Default: 53


int target_speed_left = 36;
int target_speed_right = 42;

int control_signal = 0;
int count = 0;
float error = 0;
int left_speed = 0, right_speed = 0;
uint32_t diff = 0;
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
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

PID_Controller_HandleTypeDef pid;
PID_Controller_HandleTypeDef pid2;



TCS34725_HandleTypeDef rgb_sensor_left;
TCS34725_HandleTypeDef rgb_sensor_right;

L298N_HandleTypeDef motor_driver;

SG90_HandleTypeDef gripper_servo;

HCSR04_HandleTypeDef ultrasonic_sensor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float getError() {

	HAL_StatusTypeDef status1 = tcs34725_get_data(&rgb_sensor_left, &hi2c1);
    HAL_StatusTypeDef status2 = tcs34725_get_data(&rgb_sensor_right, &hi2c3);

	if (status1 != HAL_OK || status2 != HAL_OK) {
		HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
	}

    // Calculate the error based on the difference between sensor readings
    error = (rgb_sensor_left.r_ratio - rgb_sensor_right.r_ratio) * 100;

    return error;
}

void PID_Controller(PID_Controller_HandleTypeDef *pid) {
    // Get the proportional error
    float error = getError();

    // Calculate the control signal
    float proportional = KP * error;
    float integral = KI * (error + pid->integral);
    float derivative = KD * (error - pid->previous_error);

    // Calculate control signal
    control_signal = (int)(proportional + derivative + integral);

    // Update PID values for next iteration
    pid->previous_error = error;
    pid->integral += error;

    count++;

    if(count > 10) {
    	pid->integral = 0;
    	count = 0;
    }

    // Adjust motor speeds based on control signal

    // Line is to the right, turn left
    left_speed = target_speed_left - control_signal;
    right_speed = target_speed_right + control_signal;

    if (left_speed > 100) {
    	left_speed = 100;
    }

    if (left_speed < 0) {
    	left_speed = 0;
    }

    if (right_speed > 100) {
    	right_speed = 100;
    }

    if (right_speed < 0) {
    	right_speed = 0;
    }

    // Apply control to motors
    l298n_drive_forward(&motor_driver, &htim2, left_speed, right_speed);

}

// what if we add another function same as PID so when that starts that runs and that has a timer which will open the gripper when haltick matches our req.

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  tcs34725_get_device_id(&rgb_sensor_left, &hi2c1);
  tcs34725_get_device_id(&rgb_sensor_right, &hi2c3);

  tcs34725_set_enable_reg(&rgb_sensor_left, &hi2c1);
  tcs34725_set_enable_reg(&rgb_sensor_right, &hi2c3);

  tcs34725_set_timing_reg(&rgb_sensor_left, &hi2c1);
  tcs34725_set_timing_reg(&rgb_sensor_right, &hi2c3);

  l298n_init(&htim2);

  sg90_init(&htim3);

  hcsr04_init(&htim4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(HAL_GPIO_ReadPin(GPIOC, B1_Pin));

  sg90_open(&gripper_servo, &htim3);

  HAL_Delay(1000);

  uint32_t start_time = HAL_GetTick();

  l298n_drive_forward(&motor_driver, &htim2, 100, 100);

  while (1)
  {

	  PID_Controller(&pid);

	  if (HAL_GetTick() - start_time > 9000) {

		  target_speed_left = 33;
		  target_speed_right = 40;
	  }

	  if (rgb_sensor_left.b_ratio > rgb_sensor_left.g_ratio || rgb_sensor_right.b_ratio > rgb_sensor_right.g_ratio) {

		  l298n_brake(&motor_driver);
		  HAL_Delay(200);


		  //First reverse after detecting blue
		  l298n_drive_reverse(&motor_driver, &htim2, 100, 100);
		  HAL_Delay(30);

		  l298n_drive_reverse(&motor_driver, &htim2, TARGET_SPEED_LEFT, TARGET_SPEED_RIGHT);
		  HAL_Delay(550);

		  l298n_brake(&motor_driver);
		  HAL_Delay(150);

		  hcsr04_get_distance(&ultrasonic_sensor, &htim3);

		  int counter = 0;

		  while(!(ultrasonic_sensor.distance >= WALL_TURN_DISTANCE_MIN &&
				  ultrasonic_sensor.distance <= WALL_TURN_DISTANCE_MAX && counter >= 17)){

			  l298n_rotate_counter(&motor_driver, &htim2, 100, 100);
		      HAL_Delay(30);

		      l298n_rotate_counter(&motor_driver, &htim2, 40, 48);
		      HAL_Delay(50);

		      l298n_brake(&motor_driver);
		      HAL_Delay(250);

		      counter += 1;

			  hcsr04_get_distance(&ultrasonic_sensor, &htim3);

		  }

		  l298n_brake(&motor_driver);

		  HAL_Delay(500);

		  while (ultrasonic_sensor.distance > 21) {

			  l298n_drive_reverse(&motor_driver, &htim2, 100, 100);
			  HAL_Delay(30);

			  l298n_drive_reverse(&motor_driver, &htim2, 40, 48);
			  HAL_Delay(50);

			  l298n_brake(&motor_driver);
			  HAL_Delay(250);

			  hcsr04_get_distance(&ultrasonic_sensor, &htim3);
		  }

		  l298n_brake(&motor_driver);

		  HAL_Delay(500);

		  sg90_close(&gripper_servo, &htim3);

		  HAL_Delay(500);

		  l298n_drive_forward(&motor_driver, &htim2, 100, 100);
		  HAL_Delay(30);

		  l298n_drive_forward(&motor_driver, &htim2, 40, 48);
		  HAL_Delay(430);

		  l298n_brake(&motor_driver);

		  for (int i = 0; i < 10; i++) {

			  l298n_rotate_counter(&motor_driver, &htim2, 100, 100);
			  HAL_Delay(30);

			  l298n_rotate_counter(&motor_driver, &htim2, 40, 48);
			  HAL_Delay(50);

			  l298n_brake(&motor_driver);
			  HAL_Delay(250);
		  }


		  l298n_drive_reverse(&motor_driver, &htim2, 100, 100);
		  HAL_Delay(30);

		  l298n_drive_reverse(&motor_driver, &htim2, 40, 48);
		  HAL_Delay(800);

		  l298n_brake(&motor_driver);

		  HAL_Delay(500);

		  sg90_open(&gripper_servo, &htim3);

		  HAL_Delay(500);

		  l298n_drive_forward(&motor_driver, &htim2, 100, 100);
		  HAL_Delay(30);

		  l298n_drive_forward(&motor_driver, &htim2, 40, 48);
		  HAL_Delay(750);

		  l298n_brake(&motor_driver);

		  HAL_Delay(500);

		  target_speed_left = 60;
		  target_speed_right = 72;

		  continue;

//		  l298n_brake(&motor_driver);
//
//		  HAL_Delay(300);
//
//		  //first reverse after detecting blue
//
//		  l298n_drive_reverse(&motor_driver, &htim2, 100, 100);
//
//		  HAL_Delay(30);
//
//		  l298n_drive_reverse(&motor_driver, &htim2, 33, 40);
//
//		  HAL_Delay(1500);
//
//		  l298n_brake(&motor_driver);
//
//		  HAL_Delay(150);
//
//		  //180 degree turn
//
//		  l298n_rotate_counter(&motor_driver, &htim2, 100, 100);
//
//		  HAL_Delay(30);
//
//		  l298n_rotate_counter(&motor_driver, &htim2, 33, 40);
//
//		  HAL_Delay(2050);
//
//		  l298n_brake(&motor_driver);
//
//		  l298n_drive_reverse(&motor_driver, &htim2, 33, 40);
//
//		  HAL_Delay(800);
//
//		  l298n_brake(&motor_driver);
//
//		  HAL_Delay(100);
//
//		  //Pickup
//
//		  sg90_close(&gripper_servo, &htim3);
//
//		  HAL_Delay(150);
//
//		  return 0;

		  //Rotate towards Green Box
//
//		  l298n_rotate_counter(&motor_driver, &htim2, 100, 100);
//
//		  HAL_Delay(30);
//
//		  l298n_rotate_counter(&motor_driver, &htim2, 33, 40);
//
//		  HAL_Delay(1800);
//
//		  l298n_brake(&motor_driver);
//
//		  HAL_Delay(100);
//
//		  //Reverse to Green box
//
//		  l298n_drive_reverse(&motor_driver, &htim2, 100, 100);
//
//		  HAL_Delay(30);
//
//		  l298n_drive_reverse(&motor_driver, &htim2, 33, 40);
//
//		  HAL_Delay(2000);
//
//		  l298n_brake(&motor_driver);
//
//		  HAL_Delay(100);
//
//		  //drop off
//
//		  sg90_open(&gripper_servo, &htim3);
//
//		  HAL_Delay(1000);
//
//		  //turn back towards red line
//
//		  l298n_rotate_clockwise(&motor_driver, &htim2, 100, 100);
//
//		  HAL_Delay(30);
//
//		  l298n_rotate_clockwise(&motor_driver, &htim2, 33, 40);
//
//		  HAL_Delay(1200);
//
//		  l298n_brake(&motor_driver);
//
//		  HAL_Delay(50);
//
//		  //Drive to the red line
//
//		  l298n_drive_forward(&motor_driver, &htim2, 100, 100);
//
//		  HAL_Delay(30);
//
//		  l298n_drive_forward(&motor_driver, &htim2, 33, 40);
//
//		  HAL_Delay(2000);
//
//		  l298n_brake(&motor_driver);
//
//		  HAL_Delay(1000);
//
//		  continue;

//		  return 0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 127;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 625;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|S0_Pin|S1_Pin
                          |S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 S0_Pin S1_Pin
                           S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|S0_Pin|S1_Pin
                          |S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
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

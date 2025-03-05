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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

/* Definitions for HeartBeat */
osThreadId_t HeartBeatHandle;
const osThreadAttr_t HeartBeat_attributes = {
  .name = "HeartBeat",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UpdateThrottle */
osThreadId_t UpdateThrottleHandle;
const osThreadAttr_t UpdateThrottle_attributes = {
  .name = "UpdateThrottle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LightsControl */
osThreadId_t LightsControlHandle;
const osThreadAttr_t LightsControl_attributes = {
  .name = "LightsControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReadSensors */
osThreadId_t ReadSensorsHandle;
const osThreadAttr_t ReadSensors_attributes = {
  .name = "ReadSensors",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C2_Init(void);
void Heart_Beat(void *argument);
void Update_Throttle(void *argument);
void Lights_Control(void *argument);
void Read_Sensors(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//DAC outputs
uint16_t throttle;
uint16_t regen;
//MC GPIO outputs
GPIO_PinState mc_main_ctrl;
GPIO_PinState mc_pwreco_ctrl;
GPIO_PinState mc_fwdrev_ctrl;
GPIO_PinState rc_light_en;
GPIO_PinState rr_light_en;
GPIO_PinState rl_light_en;
GPIO_PinState fan_en;
GPIO_PinState horn_en;
GPIO_PinState mppt_contactor_en;
GPIO_PinState mppt_pre_contactor_en;
GPIO_PinState imu_int;

//turn signal and strobe
uint8_t rl_turn;
uint8_t rr_turn;
uint8_t strb_light_en;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];  // Array to store the received data

//CAN transmission

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	if (GPIO_PIN == GPIO_PIN_13) {

	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    // Retrieve the received message from FIFO 0
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
        // Error handling
        Error_Handler();
    }

    // Process the received data
    // Example: Check the received ID
    if (RxData[0] == 0) {
        // Process the received data in RxData
    	uint16_t cc = RxData[6]<<8 | RxData[5];
    	if (cc != 0)
    	{
    		throttle = cc;
    	}
    	else
    	{
    		throttle = (uint16_t)RxData[2]<<8 | RxData[1];
    	}
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8];
	uint32_t TxMailbox;

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x446;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;



  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  hdac1.State = HAL_DAC_STATE_RESET;


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1); //Start DAC 1 and 2
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);

  HAL_CAN_Start(&hcan1);

	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		  Error_Handler();
	}

	CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
//  canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
//  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  canfilterconfig.FilterIdHigh = 0x446<<5;
//  canfilterconfig.FilterIdLow = 0;
//  canfilterconfig.FilterMaskIdHigh = 0x446<<5;
//  canfilterconfig.FilterMaskIdLow = 0x0000;
//  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

	  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of HeartBeat */
  HeartBeatHandle = osThreadNew(Heart_Beat, NULL, &HeartBeat_attributes);

  /* creation of UpdateThrottle */
  UpdateThrottleHandle = osThreadNew(Update_Throttle, NULL, &UpdateThrottle_attributes);

  /* creation of LightsControl */
  LightsControlHandle = osThreadNew(Lights_Control, NULL, &LightsControl_attributes);

  /* creation of ReadSensors */
  ReadSensorsHandle = osThreadNew(Read_Sensors, NULL, &ReadSensors_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 2;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c2.Init.Timing = 0x00100D14;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MC_Main_Pin|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_Main_Pin PA1 PA2 */
  GPIO_InitStruct.Pin = MC_Main_Pin|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Heart_Beat */
/**
  * @brief  Function implementing the HeartBeat thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Heart_Beat */
void Heart_Beat(void *argument)
{
  /* USER CODE BEGIN 5 */


  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    osDelay(500);

  }


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Update_Throttle */
/**
* @brief Function implementing the UpdateThrottle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Update_Throttle */
void Update_Throttle(void *argument)
{
  /* USER CODE BEGIN Update_Throttle */
  /* Infinite loop */
  for(;;)
  {
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, throttle);
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, regen);

	  //updates gpio pins with states from global variables
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, mc_main_ctrl);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, mc_pwreco_ctrl);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, mc_fwdrev_ctrl);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, fan_en);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, horn_en);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, mppt_pre_contactor_en);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, mppt_contactor_en);

	  osDelay(20);
  }
  /* USER CODE END Update_Throttle */
}

/* USER CODE BEGIN Header_Lights_Control */
/**
* @brief Function implementing the LightsControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Lights_Control */
void Lights_Control(void *argument)
{
  /* USER CODE BEGIN Lights_Control */
  int counter = 0;

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, rc_light_en); //sets center rear light (brake light)

	  if (counter == 0) {
	     if (rl_turn == 1) {
	      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
	     }
	     if (rr_turn == 1) {
	      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_1);
	     }
	     if (strb_light_en == 1) {
	      	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	     }
	  }

	  if (rl_turn == 0) {
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, rl_light_en);
	  }

	  if (rr_turn == 0) {
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, rr_light_en);
	  }


	  counter += 100;
	  counter = counter%500;
	  osDelay(100);

  }
  /* USER CODE END Lights_Control */
}

/* USER CODE BEGIN Header_Read_Sensors */
/**
* @brief Function implementing the ReadSensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Read_Sensors */
void Read_Sensors(void *argument)
{
  /* USER CODE BEGIN Read_Sensors */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Read_Sensors */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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

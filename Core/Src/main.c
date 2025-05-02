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
#include "stdbool.h"
#include "INA226.h"
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
bool brakes_active;
bool blinkers_active;
bool left_turn_active;
bool right_turn_active;


bool dirrection;
bool mc_main_ctrl;
bool array;
bool array_precharge;
bool mc_pwreco_ctrl;

int signal_counter;



//GPIO_PinState mc_main_ctrl;
//GPIO_PinState mc_pwreco_ctrl;
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

//current sensor
uint8_t currentSenseAddress = 0x44;
uint8_t imuAddress = 0x68;
float power;
uint16_t powerInteger;
uint8_t powerLSB;
uint8_t powerMSB;

//IMU
//uint8_t imuAddress = 0x68;
uint16_t accelInteger;
uint8_t acceLSB;
uint8_t accelMSB;
uint16_t velocityInteger;
uint8_t velocityLSB;
uint8_t velocityMSB;




CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

CAN_RxHeaderTypeDef RxHeader;

uint8_t datacheck = 0;
uint8_t RxData[8];  // Array to store the received data

//CAN transmission

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	if (GPIO_PIN == GPIO_PIN_13) {
//		kill_sw = 0;
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, kill_sw);
		//
	}
}

// Can reception
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
  if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }
  if (RxHeader.StdId == 0x000)
  {
	  if (RxData[0] == 0) {
		throttle = (uint16_t)RxData[2]<<8 | RxData[1];
	  }
  }
  if (RxHeader.StdId == 0x7FF){
	  if(RxData[0] == 1){
		  //byte 1
		  //ignition switch
		  if((RxData[1] & 0x80) != 0x00){
			  mppt_pre_contactor_en = false;
			  mppt_contactor_en = false;
			  //preform shut down sequence
		  }

		  if((RxData[1] & 0x40) != 0x00){
			  brakes_active = true; // turn brakes on
		  }else{
			  brakes_active = false; // turn breaks off
		  }

		  if((RxData[1] & 0x20) != 0x00){
			  dirrection = true; //Forward
		  }else{
			  dirrection = false;
		  }

		  if((RxData[1] & 0x10) != 0x00){
			  mc_main_ctrl = true;
		  }else{
			  mc_main_ctrl = false;
		  }

		  if((RxData[1] & 0x08) != 0x00){
			  array = true;
		  }else{
			  array = false;
		  }

		  if((RxData[1] & 0x04) != 0x00){
			  array_precharge = true;
		  }else{
			  array_precharge = false;
		  }


		  //byte #2
		  if((RxData[2] & 0x01) != 0x00){
			  if (blinkers_active != true) {
				  blinkers_active = true;
				  signal_counter = 0;
			  }
			  blinkers_active = true; // turn brakes on

		  }else{
			  blinkers_active = false;
		  }

		  if((RxData[2] & 0x02) != 0x00){
			  if (left_turn_active != true) {
				  left_turn_active = true;
				  signal_counter = 0;
			  }
			  left_turn_active = true; // turn brakes on


		  }else{
			  left_turn_active = false; // turn brakes off
		  }

		  if((RxData[2] & 0x04) != 0x00){
			  if(right_turn_active != true){
				  right_turn_active = true; // Turn on right
				  left_turn_active = false; //Turn off left
				  signal_counter = 0;
			  }
			  right_turn_active = true;
		  }else{
			  right_turn_active = false;
		  }


	  }
  }
}

INA226_t INA226_IVP;

union FloatBytes {
    float value;
    uint8_t bytes[4];
};
union FloatBytes fb;



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  brakes_active = false;
  blinkers_active = false;
  left_turn_active = false;
  right_turn_active = true;


  dirrection = false;
  mc_pwreco_ctrl = false;
  mc_main_ctrl = false;
  array = false;
  array_precharge = false;


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
  MX_DAC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);

  //intalize can RX interupt
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }



  //hdac1.State = HAL_DAC_STATE_RESET;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x446;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 2;

  if(INA226_Initialize(&INA226_IVP, &hi2c2, 10, 20) != HAL_OK){ Error_Handler();}
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
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
  CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MC_Main_Pin|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 PC0 PC1
                           PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_3;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// GPIO Expander Interrupt Handler
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13){
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin, GPIO_PIN_RESET);
	}
}
*/
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
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);



  HAL_DAC_Start(&hdac1,DAC_CHANNEL_1); //Start DAC 1 and 2
  HAL_DAC_Start(&hdac1,DAC_CHANNEL_2);
  array = 1;
  array_precharge = 1;


  /* Infinite loop */
  for(;;)
  {
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, throttle);
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, regen);

	  //updates gpio pins with states from global variables

	  //change for bistable relay
	  //gonna have to think about this section

	  if(mc_main_ctrl){
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  }else{
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	  }

	  if(mc_pwreco_ctrl){
		  //closed power
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	  }else{
		  //open eco
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	  }


	  if(dirrection == true){
		  //closed forward
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, SET);
	  }else{
		  //open backward
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, RESET);
	  }

	  if(array_precharge == true){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, RESET);
	  }else{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);
	  }

	  if(array == true){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, RESET);
	  }else{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, SET);
	  }


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
  //left_turn_active = true;

  /* Infinite loop */
  for(;;)
  {
	  if (blinkers_active) {
		  if (signal_counter < 5) {
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
		  }
		  else {
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
		  }
		  signal_counter++;
		  signal_counter = signal_counter%10;
		  osDelay(100);
		  continue;
	  }

	  if(left_turn_active){
		  if(signal_counter < 5){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
		  }else{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
		  }
		  signal_counter++;

	  }else{
		  if(brakes_active){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
		  }
		  else{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
		  }
	  }

	  if(right_turn_active){
		  if(signal_counter < 5){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
		  }else{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
		  }
		  signal_counter++;
	  }
	  else{
		  if(brakes_active){
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
		  }
		  else{
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, RESET);
		  }
	  }

	  if(brakes_active){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, RESET); //sets center rear light (brake light)
	  }else{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, SET);
	  }
	  signal_counter = signal_counter%10;
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

	//Write HERE

	int HAL_CAN_BUSY = 0;
	uint64_t messages_sent = 0;

	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8] = { 0 };
	uint32_t TxMailbox = { 0 };


	TxHeader.IDE = CAN_ID_STD; // Standard ID (not extended)
	TxHeader.StdId = 0x2; // 11 bit Identifier
	TxHeader.RTR = CAN_RTR_DATA; // Std RTR Data frame
	TxHeader.DLC = 8; // 8 bytes being transmitted

	//Message ID 7 for VCU Contactor Power
	TxData[0] = 7;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

  /* Infinite loop */
  for(;;)
  {
	  //INA226_t *data = (INA226_t *)argument;
	  INA226_IVP.current = getCurrentAmp(&INA226_IVP);
	  INA226_IVP.power = getPowerWatt(&INA226_IVP);

	  //use union for power
	  fb.value = INA226_IVP.power;



	  //Store in CAN, power byte L and H
	  //ina226_data[0] = INA226_IVP.power & 0xFF;
	  //ina226_data[1] = (INA226_IVP.power >> 8) & 0xFF;

	  //Assign CAN message for Power
	  TxData[1] = fb.bytes[0];
	  TxData[2] = fb.bytes[1];
	  TxData[3] = fb.bytes[2];
	  TxData[4] = fb.bytes[3];

	  while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1));
	  HAL_StatusTypeDef status;
	  status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	  messages_sent++;
	  if (status == HAL_ERROR){
		  Error_Handler();
	  }
	  else if(status == HAL_BUSY){
		  HAL_CAN_BUSY++;
	  }

    osDelay(1);
  }
  /* USER CODE END Read_Sensors */
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

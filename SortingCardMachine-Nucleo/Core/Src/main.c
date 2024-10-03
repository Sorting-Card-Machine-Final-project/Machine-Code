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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __mOrder
{
  uint8_t tray; // the number of tray to go/ options: 0, 1, 2, 3;
  uint8_t flag_moreCards; // 1 true - more cards to sort. go back; 0 false no more cards. continoue;
  uint8_t flag_notEnd; // 1 true - go back for more rounds; 0 false ended. continue;
  uint8_t flag_start; // 1 true - start; 0 false don't start;
  uint8_t Error;
}mOrder;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 20
#define RX_DATA_SIZE 1

/***** STEPPER defines #######################################################*/
#define STEPS_PER_REVOLUTION 200 // Need to define
#define STEP_DELAY_MS 10 // Need to define
#define MOTOR_TASK_PRIORITY osPriorityNormal
#define MOTOR_TASK_STACK_SIZE 128
#define FORWARS 1
#define BACKWARDS 0
#define MAX_STEPS 1000 // Need to define
/***** Tray Positions defines ################################################*/
#define FIRST_LEVEL_POS 1 // Lowest level of Sorting Tray  //UPDATE!!
#define SECOND_LEVEL_POS 2 // Lowest level of Sorting Tray  //UPDATE!!
#define THIRD_LEVEL_POS 3 // Lowest level of Sorting Tray  //UPDATE!!
#define FOURTH_LEVEL_POS 4 // Lowest level of Sorting Tray  //UPDATE!!
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/*
 * htim3 = main tray pusher
 * htim4 = return machanizem
*/
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t tempBuffer[2*RX_DATA_SIZE];
int16_t trayPosition = 0;
mOrder theMessege;
uint8_t taskSuspendFlag = 1; // 1 true - task running; 0 false task suspended;

uint8_t bufferIndexIT = 0;
uint8_t bufferIndexMain = 0;

int j = 0; // Delete
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
int getMessegeInterpret(uint8_t* buffer); // The function can return Struct of the information when we decife the struct
int getMessegeInterpret2(uint8_t* buffer); // Delete at the end
void calibration(); // The calibration of the Sorting Tray, Change the position to Zero at the end
void retrivigCards(); // The functuin  retriving the cards from the Sorting Tray back to the Feeding Tray
void StepperMove (int stepsToDir);
int calculateStepsToLevel(int level_to_go); // calculate the steps to the next level
void pullingHandlePush(); // Pushing the cards back to the Feeding Tray
void cardPushSpin(); // Pushing one card from the main deck
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
  //HAL_UART_Transmit(&huart2,(uint8_t *) "Start running", sizeof("Start running"), 100);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  //Two ways to change DutyCycle
  TIM3->CCR1 = 512; //1st way
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 256); //2nd way

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_Delay(1000);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

  /*                             Section 2                                    */

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  calibration();

  do{ // To wait for the Start Command
	  getMessegeInterpret(rxBuffer);
	  HAL_Delay(500);
  }while(!theMessege.flag_start);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(theMessege.flag_moreCards && theMessege.flag_start){ //start and more cards
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//Starting the PWM of the roller motor

		  StepperMove(calculateStepsToLevel(theMessege.tray));//move Sorting Tray to position
		  cardPushSpin();//starting one loop of the PWM of the pushing DC motor

		  HAL_Delay(1000); // To make sure the card at the place

		  getMessegeInterpret(rxBuffer); // Get the next messege
  	  }
	  else if(!theMessege.flag_moreCards && theMessege.flag_start){ // no more cards and start
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);// Stop PWM of the DC motor of the roller
		  retrivigCards();//pulling back cards to the Feeding tray
		  getMessegeInterpret(rxBuffer); // Get the next messege
	  }
	  else if(!theMessege.flag_start){ // Not start = end
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);//Turn on end indicator light
		  HAL_Delay(500);
		  // add pin for restart - 1 restart, 0, don't restart.
		  //Add if when restart to do calibration and read message to start again.
	  }

	  HAL_Delay(100);
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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
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
  sConfigOC.Pulse = 256;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1023;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 256;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|STEPPER2_DIR_Pin|STEPPER2_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEPPER1_STEP_Pin|STEPPER1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin STEPPER2_DIR_Pin STEPPER2_STEP_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|STEPPER2_DIR_Pin|STEPPER2_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STEPPER1_STEP_Pin STEPPER1_DIR_Pin */
  GPIO_InitStruct.Pin = STEPPER1_STEP_Pin|STEPPER1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SWITCH1_Pin SWITCH2_Pin */
  GPIO_InitStruct.Pin = SWITCH1_Pin|SWITCH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/***** UART Interrupt Function ################################################*/
/*
  This function starting when messege is starting to arrive
  Then the function pushing the messege to xQueueUART
*/
/**
  * @brief  This function starting when messege is starting to arrive. Then the function pushing the messege to xQueueUART
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

  if (huart->Instance == USART1){

	  memcpy(rxBuffer + bufferIndexIT, tempBuffer, RX_DATA_SIZE);
	  bufferIndexIT += RX_DATA_SIZE;
	  if (bufferIndexIT + RX_DATA_SIZE >= RX_BUFFER_SIZE) bufferIndexIT = 0;
	  HAL_UART_Receive_IT(huart, tempBuffer, RX_DATA_SIZE);
  }

}

/***** calibration Function #################################################*/
/**
  *@brief function moves the Sorting tray until the switch is close
  *@note the function changes the trayPosition global variable
  *@retval int status: 1 OK, 0 Error
*/
void calibration(){

  int switchFlag = HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin);
  HAL_GPIO_WritePin(STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin, BACKWARDS); // UP to the top position

  for (uint16_t i = 0; i < MAX_STEPS && !switchFlag; i++)
  {
    HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_SET);
    HAL_Delay(STEP_DELAY_MS); // Delay to allow stepper driver to register step
    HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(STEP_DELAY_MS); // Delay between steps
    switchFlag = HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin);
  }

  trayPosition = 0;
}
/***** Retriving Function #################################################*/
/**
  *@brief The functuin  retriving the cards from the Sorting Tray back to the Feeding Tray
  *@note The function calls calibration function
  *@retval None
*/
void retrivigCards(){
  calibration();

  for (uint8_t i = 1; i <= 4; i++)
  {
	StepperMove(calculateStepsToLevel(i));
    pullingHandlePush();
  }

}
/***** calculation of steps to level Function ##############################*/
/**
  *@brief calculate the steps to the next level
  *@param level_to_go: The level to go to
  *@note the current level is the global trayPosition
  *@retval steps to go. positive is down, negative is up // Can be changed.
*/
int calculateStepsToLevel(int level_to_go){
  switch (level_to_go)
  {
  case 1:
    return FIRST_LEVEL_POS - trayPosition;
    break;
  case 2:
    return SECOND_LEVEL_POS - trayPosition;
    break;
  case 3:
    return THIRD_LEVEL_POS - trayPosition;
    break;
  case 4:
    return FOURTH_LEVEL_POS - trayPosition;
    break;

  default:
    return 0;
    break;
  }
}

/***** Stepper Move Function ##############################################*/
/**
  *@brief moving the Sorting Tray to position
  *@param stepsToDir: the number of steps to the next position.
  *@retval  None
*/
void StepperMove (int stepsToDir){
  uint32_t steps;
  uint8_t direction;
  if(stepsToDir > 0){
    direction = GPIO_PIN_SET;
    steps = stepsToDir;
  } else {
    direction = GPIO_PIN_RESET;
    steps = stepsToDir * -1;
  }

  HAL_GPIO_WritePin(STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin, direction); // (direction ? GPIO_PIN_SET : GPIO_PIN_RESET)

  for (uint32_t i = 0; i < steps; i++)
  {
    HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_SET);
    HAL_Delay(STEP_DELAY_MS); // Delay to allow stepper driver to register step
    HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(STEP_DELAY_MS); // Delay between steps

    trayPosition += direction ? 1 : (-1) ;
  }
}
/***** Pulling Handle Push Function ########################################*/
/**
  *@brief Pushing the cards back to the Feeding Tray
  *@retval  None
*/
void pullingHandlePush(){
  int steps = 100; //! edit to the right steps number
  for (int8_t dir = 1; dir >= 0 ; dir--)
  {
    HAL_GPIO_WritePin(STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin, dir);
    for (uint32_t i = 0; i < steps; i++)
    {
      HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_SET);
      HAL_Delay(STEP_DELAY_MS); // Delay to allow stepper driver to register step
      HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_RESET);
      HAL_Delay(STEP_DELAY_MS); // Delay between steps
    }
  }

}

/**
  *@brief Pushing one card from the main deck
  *@retval  None
*/
void cardPushSpin(){
  int switchFlag = HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  for (uint8_t i = 0; i <= 255 && !switchFlag; i++)
  {
    HAL_Delay(10);
    switchFlag = HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin);
  }

  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
}

int getMessegeInterpret2(uint8_t* buffer){

	for ( ; bufferIndexMain == bufferIndexIT ; ){
		HAL_UART_Transmit(&huart2, (uint8_t*) "Waiting for message", sizeof("Waiting for message"), 1000);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
	uint8_t msg8_t= *(buffer + bufferIndexMain);

	theMessege.tray = msg8_t & 0x03;  // 1st and 2nd bits
	theMessege.flag_moreCards = (msg8_t >> 2) & 0x01; // 3rd bit
	theMessege.flag_notEnd = (msg8_t >> 3) & 0x01; // 4th bit
	theMessege.flag_start = (msg8_t >> 4) & 0x01; // 5th bit
	theMessege.Error = (msg8_t >> 5) & 0x07; // 6th to 8th bits

	bufferIndexMain += RX_DATA_SIZE;
	if (bufferIndexMain >= RX_BUFFER_SIZE) bufferIndexMain = 0;

	return 1;
}

int getMessegeInterpret(uint8_t* buffer){
	uint8_t msg8_t;
	switch (j) {
		case 0:
			msg8_t = 0x14; // start
			break;
		case 1:
			msg8_t = 0x14;
			break;
		case 2:
			msg8_t = 0x15;
				break;
		case 3:
			msg8_t = 0x16;
				break;
		case 4:
			msg8_t = 0x15;
				break;
		case 5:
			msg8_t = 0x17;
				break;
		case 6:
			msg8_t = 0x14;
				break;
		case 7:
			msg8_t = 0x14;
				break;
		case 8:
			msg8_t = 0x15;
				break;
		case 9:
			msg8_t = 0x12;
				break;
		case 10:
			msg8_t = 0x01;
				break;
		default:
			break;
	}
	j++;

	theMessege.tray = msg8_t & 0x03;  // 1st and 2nd bits
	theMessege.flag_moreCards = (msg8_t >> 2) & 0x01; // 3rd bit
	theMessege.flag_notEnd = (msg8_t >> 3) & 0x01; // 4th bit
	theMessege.flag_start = (msg8_t >> 4) & 0x01; // 5th bit
	theMessege.Error = (msg8_t >> 5) & 0x07; // 6th to 8th bits

	bufferIndexMain += RX_DATA_SIZE;
	if (bufferIndexMain >= RX_BUFFER_SIZE) bufferIndexMain = 0;

	return 1;
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

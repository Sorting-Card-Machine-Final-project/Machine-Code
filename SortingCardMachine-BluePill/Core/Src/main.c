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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 2

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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */
QueueHandle_t xQueueUART;
uint8_t rxBuffer[RX_BUFFER_SIZE];
unit8_t pvBuffer[RX_BUFFER_SIZR];
int16_t trayPosition = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument)


/* USER CODE BEGIN PFP */
// !Need to write messegeInterpret function
int messegeInterpret(uint8_t* buffer); // The function can return Struct of the information when we decife the struct 
int callibration(); // The Callibration of the Sorting Tray, Change the position to Zero at the end
void retrivigCards(); // The functuin  retriving the cards from the Sorting Tray back to the Feeding Tray
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
  xQueueUART = xQueueCreate( 2, sizeof(uint8_t) ); // Can be 1 for 8bits messege
/*
	The Queue needs to be on uint16_t for 16 bits messege
	*/
  if(xQueueUART == NULL){
    //xQueueUART was not created
    //Can handle the error or send a messege to rpi(RaspberryPi)
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	TIM2->CCR1 = 512;
  TIM2->CCR2 = 256;
  HAL_TIM_PWM_START(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_START(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_STOP(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_STOP(&htim2, TIM_CHANNEL_2);
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // Start all of the timers - TIM2 with two channels
  //Need to add al PWM functions
  
  
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
	//Start xsechduler of FreeRTOS

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1023;
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
  sConfigOC.Pulse = 512;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.BaudRate = 9600;
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
  HAL_UART_Receive_IT(&huart1, rxBuffer, RX_BUFFER_SIZE);
  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, STEPPER1_DIR_Pin|STEPPER1_STEP_Pin|STEPPER2_DIR_Pin|STEPPER2_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : switch1_Pin switch2_Pin */
  GPIO_InitStruct.Pin = switch1_Pin|switch2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STEPPER1_DIR_Pin STEPPER1_STEP_Pin STEPPER2_DIR_Pin STEPPER2_STEP_Pin */
  GPIO_InitStruct.Pin = STEPPER1_DIR_Pin|STEPPER1_STEP_Pin|STEPPER2_DIR_Pin|STEPPER2_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
   if(xQueueSendFromISR(xQueueUART, rxBuffer, NULL) == errQUEUE_FULL){
    //Queue is full - need to send a messege about it.
   }
    //HAL_UART_Receive(huart, &rxBuffer, RX_BUFFER_SIZE, HAL_MAX_DELAY);  // The function HAL_UART_Receive_IT already fill rxBuffer
    /*
    Here we need to deside what we are looking for at the messeges
    For example we have first 7 bits that reffer to the suit and number (4 bits of number - 13 options, 2 bits of suit - 4 options)
    We need to add the cell to go (2 bits),
    and acknowledge 

    The place for the interpertation can be when the program pull it from the Queue
    */

  }

  HAL_UART_Receive_IT(huart, rxBuffer, RX_BUFFER_SIZE);
}


/***** Callibration Function #################################################*/
/** 
  *@brief function moves the Sorting tray until the switch is close
  *@note the function changes the trayPosition global variable
  *@retval int status: 1 OK, 0 Error
*/
int callibration(){

  int switchFlag = HAL_GPIO_ReadPin(switch1_GPIO_Port, switch1_Pin);
  HAL_GPIO_WritePin(STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin, BACKWARDS); // UP to the top position
  
  for (uint16_t i = 0; i < MAXSTEPS && switchFlag; i++)
  {
    HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS)); // Delay to allow stepper driver to register step
    HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS)); // Delay between steps
    switchFlag = HAL_GPIO_ReadPin(switch1_GPIO_Port, switch1_Pin);
  }
  
  trayPosition = 0;
}
/***** Retriving Function #################################################*/
/** 
  *@brief The functuin  retriving the cards from the Sorting Tray back to the Feeding Tray
  *@note The function calls callibration function
  *@retval None
*/
void retrivigCards(){
  callibration();

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
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS)); // Delay to allow stepper driver to register step
    HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS)); // Delay between steps

    trayPosition += direction ? 1 : (-1) ;
  }
}
/***** Pulling Handle Push Function ########################################*/
/** 
  *@brief Pushing the cards back to the Feeding Tray
  *@retval  None
*/
void pullingHandlePush(){
  int steps = 100 //! edit to the right steps number
  for (uint8_t dir = 1; dir >= 0 ; dir--)
  {
    HAL_GPIO_WritePin(STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin, dir); 
    for (uint32_t i = 0; i < steps; i++)
    {
      HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_SET);
      vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS)); // Delay to allow stepper driver to register step
      HAL_GPIO_WritePin(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin, GPIO_PIN_RESET);
      vTaskDelay(pdMS_TO_TICKS(STEP_DELAY_MS)); // Delay between steps
    }
  }

}

/** 
  *@brief Pushing one card from the main deck
  *@retval  None
*/
void cardPushSpin(){
  int switchFlag = 0; // switch is open
	TIM2->CCR1 = 100;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  for (uint8_t i = 0; i <= 255 && !switchval; i++)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
    switchFlag = HAL_GPIO_ReadPin(switch1_GPIO_Port, switch1_Pin);
  }

  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  This is the main(). all the code is here.
  * @note	The main() function is only for running the RTOS and the GPIO,TIM
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  
  /*
  Few Things about this function.
  Most of it need to be inside the infinte loop
  At the end of the loop (each time) the function will be suspended and a flag(add one) will indicate it
  }
  The UART interrupt will wake the function again, so everything needs to be inside.
  
  */
  
  HAL_GPIO_WritePin(LED_STEP_GPIO_Port, LED_STEP_Pin, GPIO_PIN_RESET);
  calibration();

  for(uint_8 i = 0; i < 15 && xQueueReceive(xQueueUART, pvBuffer, pdMS_TO_TICKS(10)) == pdFalse; i++){
    vTaskDelay(pdMS_TO_TICKS(1000));
  } // Order to start the process
  
  /* Infinite loop */
  for(;;)
  {
    mainLoop1:
    
    HAL_TIM_PWM_START(&htim2, TIM_CHANNEL_2)//Starting the PWM of the roller motor
  
    xQueueReceive(xQueueUART, pvBuffer, pdMS_TO_TICKS(100)); //???pull from xQueue -> if empty Delay(1000);
    messegeInterpret(pvBuffer); // TODO: Not complited. Need to write the function 
    /*interpent the messege -> Build the function
    The function needs to interpent the messege and to put the values on a new Struct(global)
    */
    
    //move Sorting Tray to position
    cardPushSpin();//starting one loop of the PWM of the pushing DC motor
  
    vTaskDelay(pdMS_TO_TICKS(1000));// To make sure the card at the place
  
    //check for new messege from pi. Or another card(move back toSorting tray move)
    // or end of pile(continue)
    //start
    xQueueReceive(xQueueUART, pvBuffer, pdMS_TO_TICKS(100));
    /*
    TODO: If statment about what to do:
    another card will be "goto mainloop;"
    end of pile will just continue
    */
    //end
  
    pullingHandlePush();//pulling back cards to the Feeding tray
  
    //check messege from pi -> done or another round
    xQueueReceive(xQueueUART, pvBuffer, pdMS_TO_TICKS(100));
    /*
    TODO: If statment about what to do:
    another card will be "goto mainloop;"
    done will just continue
    */
  
    HAL_TIM_PWM_STOP(&htim2, TIM_CHANNEL_2);// Stop PWM of the DC motor of the roller
  
    HAL_GPIO_WritePin(LED_STEP_GPIO_Port, LED_STEP_Pin, GPIO_PIN_SET);//Turn on end indicator light
  
    /* USER CODE END 5 */
	}
}

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


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 2

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxBuffer[RX_BUFFER_SIZE];
/* USER CODE END PV */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
QueueHandle_t xQueueUART;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  xQueueUART = xQueueCreate( 2, sizeof(uint8_t) );

  /* USER CODE END 1 */

}


static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, rxBuffer, RX_BUFFER_SIZE);
  /* USER CODE END USART1_Init 2 */

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (huart->Instance == USART1){
    //HAL_UART_Receive(huart, &rxBuffer, RX_BUFFER_SIZE, HAL_MAX_DELAY);  // The function HAL_UART_Receive_IT already fill rxBuffer
    /*
    Here we need to deside what we are looking for at the messeges
    For example we have first 7 bits that reffer to the suit and number (4 bits of number - 13 options, 2 bits of suit - 4 options)
    We need to add the cell to go (2 bits),
    and acknowledge 

    The place for the extracktion can be when the program pull it from the Queue
    */

   if(xQueueSendFromISR(xQueueUART, rxBuffer, NULL) == errQUEUE_FULL){
    //Queue is full - need to send a messege about it.
   }
  }

  HAL_UART_Receive_IT(huart, rxBuffer, RX_BUFFER_SIZE);
}
/* USER CODE END 4 */

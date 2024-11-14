/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ECEN-361-Lab-08  Queues
  *
  * BYU-Idaho
  * Fall-2023 :   Lynn Watson
  *
  * See the  STUDENTS EDITABLE places below to modify for submission
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MultiFunctionShield.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <ctype.h>
#include "queue.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define QUEUE_SIZE 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ASCII_Char_Queue */
osMessageQueueId_t ASCII_Char_QueueHandle;
uint8_t ASCII_Char_QueueBuffer[ 50 * sizeof( uint8_t ) ];
osStaticMessageQDef_t ASCII_Char_QueueControlBlock;
const osMessageQueueAttr_t ASCII_Char_Queue_attributes = {
  .name = "ASCII_Char_Queue",
  .cb_mem = &ASCII_Char_QueueControlBlock,
  .cb_size = sizeof(ASCII_Char_QueueControlBlock),
  .mq_mem = &ASCII_Char_QueueBuffer,
  .mq_size = sizeof(ASCII_Char_QueueBuffer)
};
/* Definitions for RandomSymbolTimer */
osTimerId_t RandomSymbolTimerHandle;
const osTimerAttr_t RandomSymbolTimer_attributes = {
  .name = "RandomSymbolTimer"
};
/* Definitions for lowercaseTimer */
osTimerId_t lowercaseTimerHandle;
const osTimerAttr_t lowercaseTimer_attributes = {
  .name = "lowercaseTimer"
};
/* Definitions for ProcessQueueTimer */
osTimerId_t ProcessQueueTimerHandle;
const osTimerAttr_t ProcessQueueTimer_attributes = {
  .name = "ProcessQueueTimer"
};
/* Definitions for ProcessSemaphore */
osSemaphoreId_t ProcessSemaphoreHandle;
const osSemaphoreAttr_t ProcessSemaphore_attributes = {
  .name = "ProcessSemaphore"
};
/* USER CODE BEGIN PV */

const osThreadAttr_t bigStackTask_attributes = {
  .name = "bigStackTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Here are pointers to the tasks so I can suspend/resume */
osThreadId_t bigTaskHandle;
const osThreadAttr_t bigTask_attributes = {
  .name = "bigTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Display_Queue_Status_Task_Handle;
osThreadId_t Process_Queue_Task_Handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void Add_Random_Symbols_to_Queue(void *argument);
void Add_Random_lowercase_to_Queue(void *argument);
void PQTimer_CB(void *argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Process_Queue_Task(void *argument);
void Display_Queue_Status_Task(void *argument);
void process_button_Task(void *argument);
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
void Random_Symbols_is_Running_Task(void *argument);
void Random_lowercase_is_Running_Task(void *argument);
// void Read_and_Transmit_Task();
// void Receive_and_Print_Task();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*** Globals *********/
uint8_t RX_Buffer[BUFFER_SIZE] = {0};
uint8_t recvd_data; // byte in from USART

/*** Timings for the producers and consumers ****************/
/**************  Producers **********************************/
int Random_Symbol_Timer_Speed = 1500;     /* A symbol every 1.5-second */
int Random_lowercase_Timer_Speed = 1800;  /* A lowercase every  1.8 second */
/**************  Consumer  **********************************/
int read_pacing_delay = 2000;		// Millisec to wait before taking each character
/************************************************************/



/*Switch 3 */
bool resetQueue=false;
osStatus_t timer_status;
uint8_t button_pushed = 0;			// button to go to processes
bool  randoms_running = true;		// toggle with Switch S3

char get_random_char(int bottom,int top)
	{
	uint8_t rand_char;
	rand_char =  rand() % (top-bottom);
	/* the rand is in the range, so index it from the bottom */
	return rand_char + bottom;

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
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  // Start timer
  HAL_TIM_Base_Start_IT(&htim17);							// LED SevenSeg cycle thru them
  MultiFunctionShield_Clear();								// Clear the 7-seg display
  Clear_LEDs();												// Clear the lights
  printf("\033\143");
  printf("\033[6;3HHello\r\n");
  printf("\033\143");
  printf("Welcome to ECEN-361 Lab-07\n\r\n\r");
  printf("Button_1:  Toggles Symbols     Button_2:  Toggles Lowercase \n\r\n\r");
  HAL_UART_Receive_IT(&huart2,&recvd_data,1); //start next data receive interrupt
/**
 * Note that Timer-1 Channel 1 goes to our MultiBoard D3, and it's Negative True output
 *
 */



  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ProcessSemaphore */
  ProcessSemaphoreHandle = osSemaphoreNew(1, 1, &ProcessSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of RandomSymbolTimer */
  RandomSymbolTimerHandle = osTimerNew(Add_Random_Symbols_to_Queue, osTimerPeriodic, NULL, &RandomSymbolTimer_attributes);

  /* creation of lowercaseTimer */
  lowercaseTimerHandle = osTimerNew(Add_Random_lowercase_to_Queue, osTimerPeriodic, NULL, &lowercaseTimer_attributes);

  /* creation of ProcessQueueTimer */
  ProcessQueueTimerHandle = osTimerNew(PQTimer_CB, osTimerPeriodic, NULL, &ProcessQueueTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /** Start sending random symbols **/
  timer_status = osTimerStart(RandomSymbolTimerHandle,Random_Symbol_Timer_Speed);
  timer_status = osTimerStart(lowercaseTimerHandle,Random_lowercase_Timer_Speed);

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ASCII_Char_Queue */
  ASCII_Char_QueueHandle = osMessageQueueNew (50, sizeof(uint8_t), &ASCII_Char_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadNew(Random_Symbols_is_Running_Task, "Random_Symbols_is_Running_Task", &defaultTask_attributes);
  osThreadNew(Random_lowercase_is_Running_Task, "Random_lowercase_is_Running_Task", &defaultTask_attributes);
  osThreadNew(Process_Queue_Task,"Process_Queue_Task", &defaultTask_attributes);
  osThreadNew(Display_Queue_Status_Task,"DisplayQueueStatus" , &defaultTask_attributes);
  osThreadNew(process_button_Task,"Process Button Task" , &bigTask_attributes);

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
		// Read_and_Transmit_Task();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim4.Init.Prescaler = 8;
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
  sConfigOC.Pulse = 16000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8000 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 800-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_D1_Pin|LED_D2_Pin|LED_D3_Pin|SevenSeg_CLK_Pin
                          |SevenSeg_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SevenSeg_LATCH_GPIO_Port, SevenSeg_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LM35_IN_Pin */
  GPIO_InitStruct.Pin = LM35_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LM35_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_1_Pin Button_2_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin|Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D1_Pin LED_D2_Pin */
  GPIO_InitStruct.Pin = LED_D1_Pin|LED_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D3_Pin SevenSeg_CLK_Pin SevenSeg_DATA_Pin */
  GPIO_InitStruct.Pin = LED_D3_Pin|SevenSeg_CLK_Pin|SevenSeg_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_3_Pin */
  GPIO_InitStruct.Pin = Button_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SevenSeg_LATCH_Pin */
  GPIO_InitStruct.Pin = SevenSeg_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SevenSeg_LATCH_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */







/************  Task-Creation-Part-B *****************/
/*
 * Make D2 show that random symbols are being put on
 *      D3 show that random lowercase are being put on
 */

void Random_Symbols_is_Running_Task(void *argument)
	{ while(true)
		{
			 if (osTimerIsRunning(RandomSymbolTimerHandle))
				  HAL_GPIO_TogglePin(LED_D2_GPIO_Port,LED_D2_Pin);
			 else		/* turn it off */
				  HAL_GPIO_WritePin(LED_D2_GPIO_Port,LED_D2_Pin,GPIO_PIN_SET);
			  osDelay(300);
		}
	}
void Random_lowercase_is_Running_Task(void *argument)
	{ while(true)
		{
			 if (osTimerIsRunning(lowercaseTimerHandle))
				  HAL_GPIO_TogglePin(LED_D2_GPIO_Port,LED_D3_Pin);
			 else		/* turn it off */
				  HAL_GPIO_WritePin(LED_D2_GPIO_Port,LED_D3_Pin,GPIO_PIN_SET);
			  osDelay(300);
		}
	}





void Process_Queue_Task(void *argument)
	{
	char got_char;
	uint8_t msg_prio =100;
	/*
	 *
	 * This process consumes items in the queue, but at a determined
	 * rate.   Pacing matches a real-situation where the downstream
	 * consumer can't take things as fast as they may come.
	 * There are various ways to pace the consumption, but to be simple
	 * we'll just use the OS and osDelay to hold off on the loop
	 */
	while(true)
		{
		if (randoms_running)
			{
			if (osMessageQueueGet(ASCII_Char_QueueHandle, &got_char, &msg_prio, (uint32_t) 1) == osOK)
				{
				int queueCount = osMessageQueueGetCount (ASCII_Char_QueueHandle);
				volatile char *point = (char *) (ASCII_Char_QueueBuffer + queueCount);
				*point = ' ';
				}
			osDelay(read_pacing_delay);
			}
		}
	}







void Display_Queue_Status_Task(void *argument)
	{
	int queueCount;
	int queueSize = sizeof(ASCII_Char_QueueBuffer);
	while (true)
		{
		/*
		 *  If Empty: "----"
		 *  If Full:  "FULL"
		 *  else Display the Count
		 */
		queueCount = osMessageQueueGetCount (ASCII_Char_QueueHandle);
		if (queueCount ==0)
			{   /* Display Empty*/
			MultiFunctionShield_Display(10000);   /* greater than 9999 puts the '----' */
			}
		else if (queueCount == (uint8_t)queueSize)
			{
			/* Display Full */
			// MultiFunctionShield_Display(880);
			Display_the_FULL();
			}
		else
			{
			MultiFunctionShield_Display((uint16_t) queueCount);
			}
		osDelay(10);
		}
	}



void Read_and_Transmit_Task()
	{
	uint8_t receive_byte;
	uint8_t bytes_in =0;
	uint8_t xmitmsg[] = "\n\rInput Line to Send ->";
	uint8_t sndmsg[] = "\n\rSending -> ";
	uint8_t *xmitmsg_ptr = xmitmsg;
	uint8_t *sndmsg_ptr = sndmsg;

		bytes_in = 0;
		receive_byte = 0;
		HAL_UART_Transmit(&huart2, xmitmsg_ptr, 23, HAL_MAX_DELAY);

		/* This task reads a line from the Serial/USB port and
		 * transmits out thru SPI
		 * Note that this is polling!  One byte at a time.  Very inefficient
		 */
		while (receive_byte != '\r')
		{
			while (HAL_UART_Receive(&huart2, &receive_byte, 1,10) != HAL_OK) osDelay(1);
			/* Now we have a byte, if it's a carriage return, send the string
			 * If not, put it on the buffer
			 */
			RX_Buffer[bytes_in] = receive_byte;
			HAL_UART_Transmit(&huart2, &RX_Buffer[bytes_in++] , 1, HAL_MAX_DELAY);  //echo each one as it's typed
		}

		RX_Buffer[bytes_in++] = '\n'; // Add a line_feed
		// Tell the User what we got and what we're sending
		HAL_UART_Transmit(&huart2, sndmsg_ptr, 13, HAL_MAX_DELAY);
		//HAL_UART_Transmit(&huart2, receive_buffer_ptr, bytes_in, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, RX_Buffer, bytes_in, HAL_MAX_DELAY);


	}



/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


void process_button_Task(void *arguments)
	{
	/*
	 * The whole reason we don't just do these things inside the ISR itself is
	 * that queue operations can't be done from within an ISR.  This makes sense
	 * because the OS wouldn't be able to handle push/pop from the queue from other
	 * processes if operations were being done in an ISR too.
	 */
	char got_char = '\0';
	char * got_char_ptr = &got_char;
	uint8_t msg_prio =100;
	char queue_dump[QUEUE_SIZE+1]; // in case we have to add an extra '\0' to print
	while(true)
		{
		switch(button_pushed)
			{
			case 0:
				osDelay(1);
			break;

					// HAL_NVIC_EnableIRQ(EXTI4_IRQn);
					// HAL_NVIC_DisableIRQ(EXTI1_IRQn);
			case 1:
				/* Button_1  is START/STOP the random symbols (! .... 0)*/
				 if (osTimerIsRunning(RandomSymbolTimerHandle))
					timer_status = osTimerStop(RandomSymbolTimerHandle);
				else
					{
					timer_status = osTimerStart(RandomSymbolTimerHandle,Random_Symbol_Timer_Speed);
					HAL_NVIC_EnableIRQ(EXTI1_IRQn);
					}
			break;

			case 2: /* Button_2  is START/STOP the lowercase (! .... 0)*/
				 if (osTimerIsRunning(lowercaseTimerHandle))
					timer_status = osTimerStop(lowercaseTimerHandle);
				else
					{
					timer_status = osTimerStart(lowercaseTimerHandle,Random_lowercase_Timer_Speed);
					HAL_NVIC_EnableIRQ(EXTI4_IRQn);
					}
			break;

			case 3:
				{
				int q=0;

				/* ****************** STUDENT EDITABLE CODE START ******************* */
				/*
				 * This button ISR toggles the Queue Running
				 * If we're already halted, then restart the consumer and producers
				 * If we're running, then halt
				 *
				 *
				 */
				if (randoms_running)     //Start the HALT here
					{

				 /*
				  *
				  * To halt -- you'll need to:
				   * 1.) Shut off the producers
				   * 		Two of them produce on a timer, so stop the timers
				   * 		One of the producers is just the keyboard so you can
				   * 		assume the person quits typing.
				   *
				   * 2.) Stop the consumer -- This can be done with the
				   *     randoms_running variable
				   */
					timer_status = osTimerStop(RandomSymbolTimerHandle);
					                    timer_status = osTimerStop(lowercaseTimerHandle);

					                    randoms_running = false;    //done for you
				  /*
				   *
				   * 3.) Flush the Queue
				   * 		(You might want to print it out before resetting -- printf's below)
				   */
					printf("QUEUE INPUT HALTED    Flush below:\n\r");
					printf("QUEUE   0        1         2         3         4         5\n\r");
					printf("        12345678901234567890123456789012345678901234567890\n%c%c",'\r','\r');
					while (osMessageQueueGet(ASCII_Char_QueueHandle, got_char_ptr, &msg_prio, (uint32_t) 1) == osOK)
						queue_dump[q++] = got_char;
					queue_dump[q] ='\0';		// to be a string, has to be terminated in a null
					printf("        %s\n\r",queue_dump);
				   /*
				    * 4.) Reset the Queues via the FreeRTOS command
				   */

					/*
					 * ???
					 */

					}
				else  // were halted so restart
					{
					// Cleared out the shadow to print for the string display next time
					printf("QUEUE Resumed\n\r\r");
					// Turn on the consumer
					randoms_running = true;
					// Turn on the random producers

					/*
					 * ???
					 */
					timer_status = osTimerStart(RandomSymbolTimerHandle,Random_Symbol_Timer_Speed);
					                    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
					                    timer_status = osTimerStart(lowercaseTimerHandle,Random_lowercase_Timer_Speed);
					                    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

					}
				 /* ******************* STUDENT EDITABLE CODE STOP ******************* */
			}
			break;
			}
			button_pushed = 0;
		    osDelay(20);  // Time to make sure the switch is debounced
		}
	}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	// All three buttons generate GPIO  interrupts
	switch(GPIO_Pin)
		{
		case Button_1_Pin:
			button_pushed = 1;
			break;
		case Button_2_Pin:
			button_pushed = 2;
			break;
		case Button_3_Pin:
			button_pushed = 3;
			break;
		}

	}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
    if (resetQueue)
		{
		osMessageQueueReset (ASCII_Char_QueueHandle);
		resetQueue = false;
		}

  }
  /* USER CODE END 5 */
}

/* Add_Random_Symbols_to_Queue function */
void Add_Random_Symbols_to_Queue(void *argument)
{
  /* USER CODE BEGIN Add_Random_Symbols_to_Queue */
	char rand_sym = get_random_char('!','/');
	osMessageQueuePut(ASCII_Char_QueueHandle, &rand_sym, 100, 0U);
  /* USER CODE END Add_Random_Symbols_to_Queue */
}

/* Add_Random_lowercase_to_Queue function */
void Add_Random_lowercase_to_Queue(void *argument)
{
  /* USER CODE BEGIN Add_Random_lowercase_to_Queue */
/* ****************** STUDENT EDITABLE CODE START ******************* */

	char rand_letter = get_random_char('a','z');
	osMessageQueuePut(ASCII_Char_QueueHandle, &rand_letter, 100, 0U);

/* ****************** STUDENT EDITABLE CODE END ******************* */
  /* USER CODE END Add_Random_lowercase_to_Queue */
}

/* PQTimer_CB function */
void PQTimer_CB(void *argument)
{
  /* USER CODE BEGIN PQTimer_CB */

  /* USER CODE END PQTimer_CB */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	  if (htim == &htim17 ) { MultiFunctionShield__ISRFunc(); }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	}

  /****  Character received from UART ****/
  //UART 2 receive complete callback

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	if (huart == &huart2 )
		{
		uint8_t upper;
		upper = toupper(recvd_data);

		// HAL_UART_Transmit(&huart2, &upper ,1, HAL_MAX_DELAY);  //echo each one as it's typed

		// char myrand = get_random_char('a','z');
		//printf("\n\r\t[%c]\n\r",myrand);
		// Get cursor position
		// printf('\033[6n');
		srand((unsigned) recvd_data);
		osMessageQueuePut(ASCII_Char_QueueHandle, &upper, 100, 0U);
		HAL_UART_Receive_IT(&huart2,&recvd_data,1); //start next data receive interrupt
		// USART_ClearITPendingBit(&huart2, USART_IT);
		}



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

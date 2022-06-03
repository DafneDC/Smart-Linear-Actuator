/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "cores_communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

union CAN_ID {
	uint32_t CAN_ID_reg;
	struct {
		uint32_t CAN_SourceAddress: 8;
		uint32_t CAN_PDUS: 8;
		uint32_t CAN_PDUF: 8;
		uint32_t CAN_DP: 1;
		uint32_t CAN_R: 1;
		uint32_t CAN_Priority: 6;
	};
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for myTask01 */
osThreadId_t myTask01Handle;
const osThreadAttr_t myTask01_attributes = {
  .name = "myTask01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

FDCAN_FilterTypeDef sFilterConfig;

uint8_t tx_buff[2];

int speed = 5; 	// mm/s
int PosMax = 100; 	//mm
int PosMin = 0; 	//mm

int Contador = 0;

unsigned int buffer[8];

bool flagPos = false;
bool flagError = false;
bool flagErrorData = false;

uint8_t FMI,SPN;

int PosError = 3;

int TimePos0 = 5;

union CAN_ID CAN_ID_TxHeader;

uint32_t CAN_ID_TxHeader_CAS;

int Sensor = 0;
int Deseado = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */
uint32_t Recibir_Datos(uint8_t RxData[8]);
void Mandar_Datos(uint8_t TxData[8]);

void Abrir_Motor(int distance);
void Cerrar_Motor(int distance);
void Apagar_Motor();

bool Checar_Motor();
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
	  int32_t timeout;

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	  /* Wait until CPU2 boots and enters in stop mode or timeout*/
	  timeout = 0xFFFF;
	  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	  if (timeout < 0)
	    {
	  	  Error_Handler();
	    }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
  /*When System initialization is finished, cortex-M7 will realease Cortex-M4 by means of HSEM notification*/
   /*HW semaphore Clock enable*/
    __HAL_RCC_HSEM_CLK_ENABLE();
   /* Take HSEM */
    HAL_HSEM_FastTake(HSEM_ID_0);
    /*Release HSEM in order to notify the CPU2 (CM4) */
    HAL_HSEM_Release(HSEM_ID_0,0);
    /*wait until CPU2 wakes up from  stop mode */
    timeout = 0xFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
    if (timeout < 0)
    {
  	  Error_Handler();
    }

/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  core_share_init();
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
  /* creation of myTask01 */
  myTask01Handle = osThreadNew(StartTask01, NULL, &myTask01_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 0x1;
  hfdcan1.Init.NominalSyncJumpWidth = 0x8;
  hfdcan1.Init.NominalTimeSeg1 = 0x1F;
  hfdcan1.Init.NominalTimeSeg2 = 0x8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
/* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  //sFilterConfig.FilterID1 = 0x321;
  //sFilterConfig.FilterID2 = 0x7FF;
  sFilterConfig.FilterID1 = 0x1111111;
  sFilterConfig.FilterID2 = 0x2222222;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    /* Start Error */

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
	  /* Notification Error */
  }


  /* Prepare Tx Header */
  //TxHeader.Identifier = 0x1111111;
  //TxHeader.Identifier =  CAN_ID_TxHeader_CAS;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|Motor_IN4_Pin|Motor_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD3_Pin Motor_IN4_Pin Motor_IN3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|Motor_IN4_Pin|Motor_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Input_1_Pin Input_2_Pin Input_3_Pin Input_4_Pin */
  GPIO_InitStruct.Pin = Input_1_Pin|Input_2_Pin|Input_3_Pin|Input_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Mandar_Datos(uint8_t TxData[8]){
	/* Start the Transmission process */
	    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
	    {
	      /* Transmission request Error */
	      Error_Handler();
	    }
}

uint32_t Recibir_Datos(uint8_t RxData[8]){
	/* Start the Reception process */
		uint32_t RxID = 0xFF;
		/* Retrieve Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
		{
			//if ((RxHeader.Identifier == 0x255) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
			  // Priority = 7
			  // R = 0 ; DP = 0
			  // PGN = 0xFFF8
			  // Source Address = 0x47
			if(RxHeader.Identifier == 0x255)//if(RxHeader.Identifier == 0x1CFFF847)
			{
				RxID = RxHeader.Identifier;
				RxHeader.DataLength = FDCAN_DLC_BYTES_0;
			}
		}
		else
		{
			/* Reception Error */
		}

		return RxID;
}

void Abrir_Motor(int distance){
	HAL_GPIO_WritePin(GPIOB, Motor_IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, Motor_IN4_Pin, GPIO_PIN_RESET);

	int time = (distance/speed) * 1000;

	for(int i = 0; i < time; i++){
		if(flagError == false && flagErrorData == false){
			HAL_Delay(1);
		}
	}
	//HAL_Delay(time);

	Apagar_Motor();
}


void Cerrar_Motor(int distance){
	HAL_GPIO_WritePin(GPIOB, Motor_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Motor_IN4_Pin, GPIO_PIN_SET);

	int time = (distance/speed) * 1000;

	for(int i = 0; i < time; i++){
		if(flagError == false && flagErrorData == false){
			HAL_Delay(1);
		}
	}
	//HAL_Delay(time);

	Apagar_Motor();
}

void Apagar_Motor(){
	HAL_GPIO_WritePin(GPIOB, Motor_IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Motor_IN4_Pin, GPIO_PIN_RESET);
}

bool Checar_Motor(){

	HAL_Delay(100);

	int Sensor = buffer[2];
	int Deseado = RxData[0];

	int error = Deseado - Sensor;

	if(error > PosError){
		return true;
	}else if (error < (PosError * -1)){
		return true;
	}else{
		return false;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN 5 */

	/* -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.-
	 * -.- Thread 01:
	 * -.- 		- Recibe mensaje CAN
	 * -.-		- Estira o contrae el actuador dependiendo del mensaje CAN
	 * -.-		- Req 1.7: El dispositivo regresa a 0mm después de 5s
	 * -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.-
	 */

  /* Infinite loop */
  for(;;)
  {
	  //Led de debug

	  // -.- Posición deseada en RxData[0] y en mm -.-
	  uint32_t RxID = Recibir_Datos(RxData);

	  // -.- Lectura del sensor -.-
	  // La lectura la hace el core m4 -> buffer [2]
	  get_from_M4(buffer);

	  if (RxID == 0xFF && flagPos == false && flagError == false && flagErrorData == false){ //id para no nuevo mensajes y que el motor esta bien
		  if(Contador == TimePos0){
			  Contador = 0;
			  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,1);

			  Sensor = buffer[2];

			  bool flagAx = Checar_Motor();

			  if(Sensor > PosError && flagAx == false){
				  RxData[0] = 0;
				  Cerrar_Motor(Sensor);
			  }/*else if(flagAx == true){
				  flagPos = flagAx;
			  }*/

		  }else{
			  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
			  Contador++;
		  }

		  HAL_Delay(1000);

	  }else if(RxID == 0x255){ //recibio mensaje de esp
		  Contador = 0;
		  Sensor = buffer[2];
		  Deseado = RxData[0];

		  int error = Deseado - Sensor;

		  if(error > PosError){
			  if(Deseado > PosMax){
				  Apagar_Motor();
				  flagErrorData = true;
				  FMI = 0x1;
				  SPN = 190;
			  }else{
				  flagErrorData = false;
				  Abrir_Motor(error);
				  flagPos = Checar_Motor();
			  }
		  }else if (error < -PosError){
			  if(Deseado < PosMin){
				  Apagar_Motor();
				  flagErrorData = true;
				  FMI = 0x2;
				  SPN = 190;
			  }else{
				  flagErrorData = false;
				  Cerrar_Motor(error * -1);
				  flagPos = Checar_Motor();
			  }
		  }else{
			  flagErrorData = false;
			  Apagar_Motor();
			  flagPos = Checar_Motor();
			  HAL_Delay(100);
		  }
	  }else if(RxID == 0xFF && flagPos == true && flagError == false && flagErrorData == false){
		  Contador = 0;
		  Sensor = buffer[2];
		  Deseado = RxData[0];

		  int error = Deseado - Sensor;

		  if(error > PosError){
			  Abrir_Motor(error);
			  flagPos = Checar_Motor();
		  }else if (error < -PosError){
			  Cerrar_Motor(error * -1);
			  flagPos = Checar_Motor();
		  }else{
			  Apagar_Motor();
			  flagPos = Checar_Motor();
			  HAL_Delay(100);
		  }
	  }/*else if(flagError == true){
		  if(Sensor > PosError){
			  RxData[0] = 0;
			  Cerrar_Motor(Sensor);
		  }else{
			  flagPos = Checar_Motor();
		  }
	  }*/else{
		  Contador = 0;
		  flagPos = Checar_Motor();
	  }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */

	/* -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.-
	 * -.- Thread 02:
	 * -.- 		- Manda mensaje CAN al ESP8266
	 * -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.- -.-
	 */

  /* Infinite loop */

  for(;;)
  {
	  //Led para Debug

	  //Leer datos mandados por M4
	  get_from_M4(buffer);

	  int In1 = HAL_GPIO_ReadPin(Input_1_GPIO_Port, Input_1_Pin);

	  if(In1== 1){
		  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 1);
		  flagError = true;
		  SPN = 168;
		  FMI = 4;
	  } else if (HAL_GPIO_ReadPin(Input_2_GPIO_Port, Input_2_Pin)){
		  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 1);
		  flagError = true;
		  SPN = 168;
		  FMI = 5;
	  } else if (HAL_GPIO_ReadPin(Input_3_GPIO_Port, Input_3_Pin)){
		  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 1);
		  flagError = true;
		  SPN = 143;
		  FMI = 6;
	  } else if (HAL_GPIO_ReadPin(Input_4_GPIO_Port, Input_4_Pin)){
		  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 1);
		  flagError = true;
		  SPN = 143;
		  FMI = 7;
	  } else {
		  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, 0);
		  flagError = false;
	  }


	  if(flagError == false && flagErrorData == false){
		  CAN_ID_TxHeader.CAN_Priority = 1;
		  CAN_ID_TxHeader.CAN_R = 0;
		  CAN_ID_TxHeader.CAN_DP = 0;
		  CAN_ID_TxHeader.CAN_PDUF = 0xF0;
		  CAN_ID_TxHeader.CAN_PDUS = 0x32;
		  CAN_ID_TxHeader.CAN_SourceAddress = 0x48;

		  assert(sizeof(TxHeader.Identifier) == sizeof(CAN_ID_TxHeader));
		  memcpy(&TxHeader.Identifier, &CAN_ID_TxHeader, sizeof(CAN_ID_TxHeader));

		  //Sensor read
		  TxData[0] = buffer[2];

		  //SPN
		  TxData[1] = 0xBF;
		  TxData[2] = 0x16;
	  }else{
		  CAN_ID_TxHeader.CAN_Priority = 6;
		  CAN_ID_TxHeader.CAN_R = 0;
		  CAN_ID_TxHeader.CAN_DP = 0;
		  CAN_ID_TxHeader.CAN_PDUF = 0xFE;
		  CAN_ID_TxHeader.CAN_PDUS = 0xCA;
		  CAN_ID_TxHeader.CAN_SourceAddress = 0x48;

		  assert(sizeof(TxHeader.Identifier) == sizeof(CAN_ID_TxHeader));
		  memcpy(&TxHeader.Identifier, &CAN_ID_TxHeader, sizeof(CAN_ID_TxHeader));

		  //FMI
		  TxData[0] = FMI;

		  //SPN
		  if (SPN == 190){
			  TxData[1] = 190;
			  TxData[2] = 0;
		  }else if (SPN == 168){
			  TxData[1] = 168;
			  TxData[2] = 0;
		  }else if (SPN == 143){
			  TxData[1] = 0x34;
			  TxData[2] = 0x04;
		  }

	  }

	  Mandar_Datos(TxData);

	  HAL_Delay(1000);
  }
  /* USER CODE END StartTask02 */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

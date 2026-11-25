/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TSSI Firmware
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
FDCAN_HandleTypeDef hfdcan1;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */

volatile uint8_t bms_fault = 0;
volatile uint8_t imd_fault = 0;

volatile uint32_t bms_fault_timestamp_ms = 0;
volatile uint32_t imd_fault_timestamp_ms = 0;

static uint8_t can_counter = 0;
static uint32_t last_can_tx_ms = 0;

/* Timestamp event queue */
#define TIMESTAMP_QUEUE_SIZE 8U

typedef struct
{
    uint8_t source;
    uint8_t fault_state;
    uint32_t timestamp_ms;
} TimestampEvent;

static volatile TimestampEvent timestamp_queue[TIMESTAMP_QUEUE_SIZE];

static volatile uint8_t timestamp_queue_head = 0;
static volatile uint8_t timestamp_queue_tail = 0;
static volatile uint8_t timestamp_queue_count = 0;
static volatile uint8_t timestamp_queue_overflow = 0;

static TimestampEvent pending_event;
static uint8_t pending_event_valid = 0U;

/* Temporary CAN settings for onboarding demonstration */
#define CAN_MESSAGE_ID       0x123
#define CAN_TIMESTAMP_ID     0x124
#define CAN_HEARTBEAT_MS     500U

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
static void QueueTimestampEvent(uint8_t source,
                                uint8_t fault_state,
                                uint32_t timestamp);
static uint8_t GetNextTimestampEvent(TimestampEvent *event);

static void UpdateFaultState(GPIO_TypeDef *port,
                             uint16_t pin,
                             volatile uint8_t *fault_flag);

static void SendCanHeartbeat(void);
static uint8_t SendCanTimestampEvent(const TimestampEvent *event);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void UpdateFaultState(GPIO_TypeDef *port,
                             uint16_t pin,
                             volatile uint8_t *fault_flag)
{
    *fault_flag = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);
}
static void QueueTimestampEvent(uint8_t source,
                                uint8_t fault_state,
                                uint32_t timestamp)
{
    if (timestamp_queue_count < TIMESTAMP_QUEUE_SIZE)
    {
        timestamp_queue[timestamp_queue_head].source = source;
        timestamp_queue[timestamp_queue_head].fault_state = fault_state;
        timestamp_queue[timestamp_queue_head].timestamp_ms = timestamp;

        timestamp_queue_head =
            (timestamp_queue_head + 1U) % TIMESTAMP_QUEUE_SIZE;

        timestamp_queue_count++;
    }
    else
    {
        timestamp_queue_overflow = 1U;
    }
}

/* ADD THE NEW FUNCTION HERE */
static uint8_t GetNextTimestampEvent(TimestampEvent *event)
{
    uint8_t event_available = 0U;

    HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);

    if (timestamp_queue_count > 0U)
    {
        event->source =
            timestamp_queue[timestamp_queue_tail].source;

        event->fault_state =
            timestamp_queue[timestamp_queue_tail].fault_state;

        event->timestamp_ms =
            timestamp_queue[timestamp_queue_tail].timestamp_ms;

        timestamp_queue_tail =
            (timestamp_queue_tail + 1U) % TIMESTAMP_QUEUE_SIZE;

        timestamp_queue_count--;

        event_available = 1U;
    }

    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    return event_available;
}

/* SendCanHeartbeat() comes after this */
static void SendCanHeartbeat(void)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    uint8_t error_code = 0;

    /* Determine overall error code */
    if (bms_fault && imd_fault)
    {
        error_code = 3;
    }
    else if (bms_fault)
    {
        error_code = 1;
    }
    else if (imd_fault)
    {
        error_code = 2;
    }

    /* Build the 8-byte CAN message */
    TxData[0] = bms_fault;
    TxData[1] = imd_fault;
    TxData[2] = error_code;
    TxData[3] = can_counter;

    /* Bytes 4-7 remain zero */

    /* Configure CAN message header */
    TxHeader.Identifier = CAN_MESSAGE_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    /* Transmit only when a FIFO slot is available */
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U)
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(
                &hfdcan1, &TxHeader, TxData) == HAL_OK)
        {
            /* Counter cycles from 0 through 8 */
            can_counter++;

            if (can_counter > 8U)
            {
                can_counter = 0;
            }
        }
    }
}
static uint8_t SendCanTimestampEvent(const TimestampEvent *event)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    TxData[0] = event->source;
    TxData[1] = event->fault_state;

    TxData[2] = (uint8_t)event->timestamp_ms;
    TxData[3] = (uint8_t)(event->timestamp_ms >> 8);
    TxData[4] = (uint8_t)(event->timestamp_ms >> 16);
    TxData[5] = (uint8_t)(event->timestamp_ms >> 24);

    TxHeader.Identifier = CAN_TIMESTAMP_ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0U)
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(
                &hfdcan1, &TxHeader, TxData) == HAL_OK)
        {
            return 1U;
        }
    }

    return 0U;
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
  MX_FDCAN1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

    /* Read initial fault signal states */
    UpdateFaultState(BMS_FAULT_GPIO_Port,
                     BMS_FAULT_Pin,
                     &bms_fault);

    UpdateFaultState(IMD_FAULT_GPIO_Port,
                     IMD_FAULT_Pin,
                     &imd_fault);

    /* Initialize timestamps */
    bms_fault_timestamp_ms = HAL_GetTick();
    imd_fault_timestamp_ms = HAL_GetTick();

    /* Start FDCAN peripheral */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    last_can_tx_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        /* Send CAN heartbeat every 500 ms */
        if ((HAL_GetTick() - last_can_tx_ms) >= CAN_HEARTBEAT_MS)
        {
            last_can_tx_ms = HAL_GetTick();
            SendCanHeartbeat();
        }

        /* Send timestamp diagnostics when a fault signal changes */
        /* Send queued timestamp events over CAN */
        if (!pending_event_valid)
        {
            pending_event_valid = GetNextTimestampEvent(&pending_event);
        }

        if (pending_event_valid)
        {
            if (SendCanTimestampEvent(&pending_event) == 1U)
            {
                pending_event_valid = 0U;
            }
        }
        /* Refresh independent watchdog */
        HAL_IWDG_Refresh(&hiwdg);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 2;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : BMS_FAULT_Pin IMD_FAULT_Pin */
  GPIO_InitStruct.Pin = BMS_FAULT_Pin|IMD_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief GPIO External Interrupt Callback
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMS_FAULT_Pin)
    {
        UpdateFaultState(
            BMS_FAULT_GPIO_Port,
            BMS_FAULT_Pin,
            &bms_fault);

        bms_fault_timestamp_ms = HAL_GetTick();

        QueueTimestampEvent(0U,
                            bms_fault,
                            bms_fault_timestamp_ms);
    }
    else if (GPIO_Pin == IMD_FAULT_Pin)
    {
        UpdateFaultState(
            IMD_FAULT_GPIO_Port,
            IMD_FAULT_Pin,
            &imd_fault);

        imd_fault_timestamp_ms = HAL_GetTick();

        QueueTimestampEvent(1U,
                            imd_fault,
                            imd_fault_timestamp_ms);
    }
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
#ifdef USE_FULL_ASSERT
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

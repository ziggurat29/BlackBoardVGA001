/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#if HAVE_USBCDC
#include "usbd_cdc.h"	//just for the XXX_USBCDC_PresenceHack()
#endif
#include "system_interfaces.h"
#include "serial_devices.h"
#include "util_circbuff2.h"

#include "lamps.h"
#include "task_notification_bits.h"

#include "task_monitor.h"

#include "video.h"


//get the debug cycle counter as a high-resolution clock
uint32_t inline getCyCnt() { return DWT->CYCCNT; }


#ifndef COUNTOF
#define COUNTOF(arr) (sizeof(arr)/sizeof(arr[0]))
#endif

#if ! HAVE_UART1 && ! HAVE_USBCDC
#error You must set at least one of HAVE_UART1 HAVE_USBCDC to 1 in project settings
#endif

//This controls whether we use the FreeRTOS heap implementation to also provide
//the libc malloc() and friends.
#define USE_FREERTOS_HEAP_IMPL 1


//resource usage statistics collected in default task for production tuning
#ifdef DEBUG
volatile size_t g_nHeapFree;
volatile size_t g_nMinEverHeapFree;
#if HAVE_UART1
volatile int g_nMaxUART1TxQueue;
volatile int g_nMaxUART1RxQueue;
#endif
#if HAVE_USBCDC
volatile int g_nMaxCDCTxQueue;
volatile int g_nMaxCDCRxQueue;
#endif
volatile int g_nMinStackFreeDefault;
volatile int g_nMinStackFreeMonitor;
#endif

#if USE_FREERTOS_HEAP_IMPL

#if configAPPLICATION_ALLOCATED_HEAP
//we define our heap (to be used by FreeRTOS heap_4.c implementation) to be
//exactly where we want it to be.
__attribute__((aligned(8))) 
uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif
//we implemented a 'realloc' for a heap_4 derived implementation
extern void* pvPortRealloc( void* pvOrig, size_t xWantedSize );
//we implemented a 'heapwalk' function
typedef int (*CBK_HEAPWALK) ( void* pblk, uint32_t nBlkSize, int bIsFree, void* pinst );
extern int vPortHeapWalk ( CBK_HEAPWALK pfnWalk, void* pinst );

//'wrapped functions' for library interpositioning
//you must specify these gcc (linker-directed) options to cause the wrappers'
//delights to be generated:

// -Wl,--wrap,malloc -Wl,--wrap,free -Wl,--wrap,realloc -Wl,--wrap,calloc -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_calloc_r

//hmm; can I declare these 'inline' and save a little code and stack?
void* __wrap_malloc ( size_t size ) { return pvPortMalloc ( size ); }
void __wrap_free ( void* pv ) { vPortFree ( pv ); }
void* __wrap_realloc ( void* pv, size_t size ) { return pvPortRealloc ( pv, size ); }

void* __wrap__malloc_r ( struct _reent* r, size_t size ) { return pvPortMalloc ( size ); }
void __wrap__free_r ( struct _reent* r, void* pv ) { vPortFree ( pv ); }
void* __wrap__realloc_r ( struct _reent* r, void* pv, size_t size ) { return pvPortRealloc ( pv, size ); }

#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//I wanted the task stuff (especially FreeRTOS stack) to be in CCM, freeing the
//main memory for things that might need to interact with peripherals;
//especially DMA.

//The other stuff doesn't /have/ to be there, but might as well get it out of
//main SRAM since the CCRAM is otherwise going unused.


//XXX generated code hack; we put things right where we want them to be, above.
//If you get some linker errors after you modify the CubeMX project, it might
//be that it added stuff during generation, below, and that is conditioned-out.
//So just copy whatever it is above.
#if 1

DAC_HandleTypeDef hdac;// __ccram;	//XXX maybe I can, maybe I can't; haven't tried yet
DMA_HandleTypeDef hdma_dac1;// __ccram;
DMA_HandleTypeDef hdma_dac2;// __ccram;

RTC_HandleTypeDef hrtc __ccram;
//DMA 'handles' cannot be in CCM because the DMA controller will access some members and CCM is inaccessible to the DMA controllers.
SD_HandleTypeDef hsd;// __ccram;
DMA_HandleTypeDef hdma_sdio_rx;// __ccram;
DMA_HandleTypeDef hdma_sdio_tx;// __ccram;
SPI_HandleTypeDef hspi1 __ccram;
TIM_HandleTypeDef htim1;// __ccram;
TIM_HandleTypeDef htim4;// __ccram;
DMA_HandleTypeDef hdma_tim1_up;// __ccram;
UART_HandleTypeDef huart1 __ccram;

osThreadId defaultTaskHandle __ccram;
uint32_t defaultTaskBuffer[ 128 ] __ccram;
osStaticThreadDef_t defaultTaskControlBlock __ccram;

#else
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
DMA_HandleTypeDef hdma_dac2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */
//XXX end generate code hack
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

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
	//enable the core debug cycle counter to be used as a precision timer
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	//set up that the video-related timers will stop when debugging
	__HAL_DBGMCU_FREEZE_TIM4();
	__HAL_DBGMCU_FREEZE_TIM1();

	//do a dummy alloc to cause the heap to be init'ed and so the memory stats as well
	vPortFree ( pvPortMalloc ( 0 ) );

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

#if HAVE_USBCDC
	//if you get a linker fail on the following, it is because some manual
	//changes to:
	//  .\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Inc\usbd_cdc.h
	//  .\Middlewares\ST\STM32_USB_Device_Library\Class\CDC\Src\usbd_cdc.c
	//  .\Src\usbd_cdc_if.c
	//must be applied.  There are backups of those files to help with that.
	//This has to be done manually, because the changes are in tool generated
	//code that gets overwritten when you re-run STM32CubeMX.  The nature of
	//those changes are such that when they are overwritten, you will still
	//be able to build but stuff won't work at runtime.  This hack will cause
	//the build to fail if you forget to merge those changes back on, thus
	//prompting you to do so.
	//There is a #fixups/fixup.bat to help with this.
	//Sorry for the inconvenience, but I don't think there is any better way
	//of making it obvious that this chore simply must be done.
	XXX_USBCDC_PresenceHack();	//this does nothing real; do not delete
#endif

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	//turn on the low power regulator so the backup sram will indeed be battery backed
	HAL_PWR_EnableBkUpAccess();	//ensure access to the backup domain, if not already
	__HAL_RCC_PWR_CLK_ENABLE();	//ensure the power clock is on, if not already
	HAL_PWREx_EnableBkUpReg();	//turn on the backup regulator and wait for stabilization
	//leave access to the backup domain active so we can freely fiddle with rtc, etc

	//enable the backup sram clock, and just leave it running so we can access freely
	__HAL_RCC_BKPSRAM_CLK_ENABLE();	//enable the backup sram clock

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
	//I don't know why I have to do this, but if I don't the following Init
	//and SetXXX functions will fail, seemingly randomly.
	__HAL_RCC_RTC_ENABLE();
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	//if we've already initted the clock, do not reset the time/date
	if ( hrtc.Instance->ISR & RTC_FLAG_INITS )
	{
		__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	}
	else
	{
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x1;
  sDate.Year = 0x20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	}

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1055;
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
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 128;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 216;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 16016;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 115200;
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

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_D2_Pin|LED_D3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, R0_Pin|R1_Pin|R2_Pin|G0_Pin 
                          |G1_Pin|G2_Pin|B0_Pin|B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY1_Pin KEY0_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BOGO_CARDDETECT_Pin */
  GPIO_InitStruct.Pin = BOGO_CARDDETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOGO_CARDDETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D2_Pin LED_D3_Pin */
  GPIO_InitStruct.Pin = LED_D2_Pin|LED_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : FLASH_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin G0_Pin 
                           G1_Pin G2_Pin B0_Pin B1_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|G0_Pin 
                          |G1_Pin|G2_Pin|B0_Pin|B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_CK_Pin */
  GPIO_InitStruct.Pin = PS2_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PS2_CK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_DATA_Pin */
  GPIO_InitStruct.Pin = PS2_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PS2_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_Pin */
  GPIO_InitStruct.Pin = VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(VSYNC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */


//====================================================


//this is made a function simply for tidiness, and locals lifetime
void __startWorkerTasks ( void )
{
	//kick off the monitor thread, which handles the user interactions
	{
	osThreadStaticDef(taskMonitor, thrdfxnMonitorTask, osPriorityNormal, 0, COUNTOF(g_tbMonitor), g_tbMonitor, &g_tcbMonitor);
	g_thMonitor = osThreadCreate(osThread(taskMonitor), NULL);
	}
}


//====================================================
//miscellaneous hooks of our creation



//SSS sync this with interface binding, below
#if HAVE_USBCDC

//well-discplined serial clients will assert DTR, and we
//can use that as an indication that a client application
//opened the port.
//NOTE:  These lines are often also set to an initial state
//by the host's driver, so do not consider these to be
//exclusively an indication of a client connecting.  Hosts
//usually will deassert these signals when this device
//enumerates.  Lastly, there is no guarantee that a client
//will assert DTR, so it's not 100% guarantee, just a pretty
//good indicator.
//NOTE:  we are in an ISR at this time
void USBCDC_DTR ( int bAssert )
{
	Monitor_ClientConnect ( bAssert );
}

//(unneeded)
//void USBCDC_RTS ( int bAssert ) { }
#elif HAVE_UART1
#endif




//SSS sync this with interface binding, below
#if HAVE_USBCDC
void USBCDC_DataAvailable ( void )
#elif HAVE_UART1
void UART1_DataAvailable ( void )
#endif
{
	//this notification is required because our Monitor is implemented with the
	//non-blocking command interface, so we need to know when to wake and bake.
	Monitor_DAV();
}


//SSS sync this with interface binding, below
#if HAVE_USBCDC
void USBCDC_TransmitEmpty ( void )
#elif HAVE_UART1
void UART1_TransmitEmpty ( void )
#endif
{
	//we don't really need this, but here's how you do it
	Monitor_TBMT();
}



//====================================================
//FreeRTOS hooks



void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	called if a stack overflow is detected. */
	volatile int i = 0;
	(void)i;
}



void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created. It is also called by various parts of the
	demo application. If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	volatile int i = 0;
	(void)i;
}



//====================================================
//EXTI support


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
/*
	switch ( GPIO_Pin )
	{
		case KEY0_Pin:
		break;
		case KEY1_Pin:
		break;
		case PS2_CK_Pin:
		break;
		default:
			//XXX que?
		break;
	}
*/
}




#if 0

//SPI1 Init for W25Q16 Flash 2MB; 50 MHz max, so we divide down to 42 MHz
static void MX_SPI1_Init_Flash(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
	HAL_SPI_DeInit(&hspi1);
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}
#endif



//====================================================
//hooks related to video support


//TIM4 OC2 is used to signal start of scan line
//TIM4 OC3 is used to signal end of scan line
//XXX may become DMA transfer complete
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
	//currently we implicitly know this is from TIM4
//	if (htim->Instance == TIM4) {

	//if OC2; start of video
	//XXX already cleared by caller if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET)
	if ( HAL_TIM_ACTIVE_CHANNEL_2 == htim->Channel )
	{
		//XXX in trigger mode, we shouldn't have to do this stuff
		volatile int i = 0;
		(void)i;
	}

	//if OC3; end of video
	//XXX already cleared by caller if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
	if ( HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel )
	{
		//first, clear the OC2 since the TIM seems level-triggered
		uint16_t saveCCMR1 = TIM4->CCMR1;
		saveCCMR1 &= ~TIM_CCMR1_OC2M;
		saveCCMR1 |= (TIM_OCMODE_FORCED_INACTIVE << 8U);	//'4'
		TIM4->CCMR1 = saveCCMR1;

		//now turn off TIM1 so as to be ready for next trigger
		//disable slave move; we can't stop the timer unless we do this first
		uint16_t saveSMCR = TIM1->SMCR;
		TIM1->SMCR = 0;
		//TIM1->SR &= ~(TIM_SR_TIF);
		TIM1->CR1 &= ~(TIM_CR1_CEN);	//stop clocking; effectively waiting for next trigger
		//TIM1->CR1 |= ~(TIM_CR1_URS);	//update request source
		TIM1->CNT = 0;	//in trigger mode, we must explicitly reset
		TIM1->SMCR = saveSMCR;	//re-enable slave mode

		//set back to go active when we match again, and thus trigger once more
		saveCCMR1 &= ~TIM_CCMR1_OC2M;
		saveCCMR1 |= (TIM_OCMODE_ACTIVE << 8U);	//'1'
		TIM4->CCMR1 = saveCCMR1;


		//XXX do VGA state machine
		volatile int i = 0;
		(void)i;
//		HAL_GPIO_TogglePin(VSYNC_GPIO_Port, VSYNC_Pin);	//testing; see the interrupt

	}

//	}
//	else
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

	//

	// Write to Backup SRAM with 32-Bit Data
#if 0
	for (uint32_t i = 0x0; i < 0x100; i += 4) {
		*(__IO uint32_t*)(BKPSRAM_BASE + i) = i;
	}
#endif

#if 0
	// Check the written Data
	for (uint32_t i = 0x0; i < 0x100; i += 4) {
		if ((*(__IO uint32_t*)(BKPSRAM_BASE + i)) != i){
			Error_Handler();
		}
	}
#endif

	//crank up serial ports
#if HAVE_UART1
	UART1_Init();	//UART1, alternative monitor
#endif
#if HAVE_USBCDC
	USBCDC_Init();	//CDC == monitor
#endif

	//bind the interfaces to the relevant devices
	//these 'HAVE_xxx' macros are in the preprocessor defs of the project
	//SSS the following logic must be kept in sync with logic up above
#if HAVE_USBCDC
	//we'll prefer the USB CDC if we've defined support for both
	g_pMonitorIOIf = &g_pifCDC;		//monitor is on USB CDC
#elif HAVE_UART1
	g_pMonitorIOIf = &g_pifUART1;	//monitor is on UART1
#endif

#if 0
	{
		/*
		NOTE: There are some goofy defines in fatfs.h/.c in generated code:

		uint8_t retSD;    // Return value for SD
		char SDPath[4];   // SD logical drive path
		FATFS SDFatFS;    // File system object for SD logical drive
		FIL SDFile;       // File object for SD
		
		The last two are not used anywhere in the 'middleware', so feel free
		to ignore them and define your own.  if you use 'data sections' in the
		linker config, they will be discarded.

		The first two are unfortunately used, so you have to work harder to
		make them go away.  But... they are only used in MX_FATFS_Init(), which
		in turn only does FATFS_LinkDriver(), so you could do some magicry to
		make those go away, too.
		*/

		//Register the file system object to the FatFs module
		//note; the SDFatFS and SDPath are in fatfs.h (why?)
		SDPath[0] = '0';
		SDPath[1] = ':';
		SDPath[2] = '\0';
		FRESULT fr = f_mount ( &SDFatFS, (TCHAR const*)SDPath, 1 );
		if ( fr != FR_OK )
		{
			//will be FR_NOT_READY if no disk
			//FatFs Initialization Error
			Error_Handler();
		}
		else
		{
			//Open the text file object with read access
			//note; the SDFile is in fatfs.h (why?)
			if ( f_open ( &SDFile, "dummy.txt", FA_READ ) != FR_OK)
			{
				//'dummy.txt' file Open for read Error
				Error_Handler();
			}
			else
			{
				//Read data from the text file
				uint32_t bytesread;
				uint8_t rtext[100];
				fr = f_read ( &SDFile, rtext, sizeof(rtext), (void *)&bytesread );
				if ( ( bytesread == 0 ) || ( fr != FR_OK ) )
				{
					//'dummy.txt' file Read or EOF Error
					Error_Handler();
				}
				else
				{
					//YYY something
				}

				/* Close the open text file */
				f_close ( &SDFile );
			}

			//Open the text file object with write access
			//note; the SDFile is in fatfs.h (why?)
			if ( f_open ( &SDFile, "dummy2.txt", FA_CREATE_ALWAYS | FA_WRITE | FA_READ ) != FR_OK)
			{
				//'dummy2.txt' file Open for write Error
				Error_Handler();
			}
			else
			{
				//Write data to the text file
				uint32_t byteswritten;
				static char szText[] = "lalalililulu\n";
				fr = f_write ( &SDFile, szText, strlen(szText), (void *)&byteswritten );
				if ( ( byteswritten == 0 ) || ( fr != FR_OK ) )
				{
					//'dummy.txt' file Write error or disk full
					Error_Handler();
				}
				else
				{
					//YYY something
				}

				/* Close the open text file */
				f_close ( &SDFile );
			}



			f_mount ( NULL, (TCHAR const*)"", 0 );
		}
	}
#endif


	{
		uint32_t start, end;
		start = getCyCnt();
		//if ( 0 != dsptest_fft() )
		//{
		//	Error_Handler();
		//}
		end = getCyCnt();
		volatile uint32_t nDur = end - start;
		volatile int i = 0;
	}

	//set up the video subsystem
	Video_Initialize();

/*
	//Infinite loop
	for(;;)
	{
		_ledToggleD2();
		_ledToggleD3();
		osDelay(500);
	}
*/

	//this must be done to get the PWM output started
	//'ideal' 800x600x60 timings if we can have a 160 MHz clock
	htim4.Instance->PSC = 1;		//divide (160/2) /(1+1) = 40 MHz pix clock
	//htim4.Instance->ARR = 1055;		//line time+1 = (128+88+800+40) pix 26.4 us
	//htim4.Instance->CCR1 = 128;		//hsync
	//htim4.Instance->CCR2 = 216;		//pix start
	//htim4.Instance->CCR3 = 1016;	//pix end
	//but 160 MHz means we can't have USB.  Attempt to approximate with 168 MHz; +5%
	htim4.Instance->ARR = 1107;		//line time+1 = 26.38 us
	htim4.Instance->CCR1 = 134;
	htim4.Instance->CCR2 = 227;
//htim4.Instance->CCR2 = 300;
	htim4.Instance->CCR3 = 1067;
//htim4.Instance->CCR3 = 700;
	//set the hsync polarity as needed (positive)
	htim4.Instance->CCER = ( htim4.Instance->CCER & ~TIM_CCER_CC1P ) | 0;

/*
	//640x480x60 approximation at 168 MHz
	htim4.Instance->ARR = 1334;		//line time+1 = 31.76 us, 31.48 kHz
	htim4.Instance->CCR1 = 160;
	htim4.Instance->CCR2 = 240;
	htim4.Instance->CCR3 = 1308;

	//set the hsync polarity as needed (negative)
	htim4.Instance->CCER = ( htim4.Instance->CCER & ~TIM_CCER_CC1P ) | TIM_CCER_CC1P;
*/

	HAL_TIM_Base_Start(&htim4); //Starts the state machine generation
	if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)	//Starts the HSYNC signal generation
	{
		Error_Handler();	//horror
	}
	if (HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)	//Starts the start of scan trigger
	{
		Error_Handler();	//horror
	}
	if (HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3) != HAL_OK)	//Starts the end of scan interrupt
	{
		Error_Handler();	//horror
	}

//XXX
	//HAL_TIM_Base_Start(&htim1); //Starts the pix clock generation
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)	//Starts the pix clock test signal generation
	{
		Error_Handler();	//horror
	}

	//light some lamps on a countdown
	LightLamp ( 1000, &g_lltD2, _ledOnD2 );
	LightLamp ( 2000, &g_lltD3, _ledOnD3 );

	//start up worker threads
	__startWorkerTasks();

	//================================================
	//temporary test crap
	{
	volatile size_t nPushed;

#if HAVE_USBCDC
#elif HAVE_UART1 && defined(DEBUG)
	//the uart1 monitor is for my debugging convenience, but it doesn't have a
	//'client connected' event, so squirt out a string to make it obvious we
	//are live
	g_pifUART1._transmitCompletely ( &g_pifUART1, "Hi, there!\r\n> ", -1, 1000 );
#endif

	(void) nPushed;
	nPushed = 0;	//(just for breakpoint)
	}

	//================================================
	//continue running this task
	//This task, the 'default' task, was generated by the tool, and it's easier
	//to just keep it than to fight the tool to destroy it (though some of that
	//fighting can be made a little easier if it was dynamically allocated,
	//then just exited).  We repurpose it after init to handle the lamps and
	//periodically sample performance data (useful for tuning pre-release).

	//Infinite loop
	uint32_t msWait = 1000;
	for(;;)
	{
		//wait on various task notifications
		uint32_t ulNotificationValue;
		BaseType_t xResult = xTaskNotifyWait( pdFALSE,	//Don't clear bits on entry.
				0xffffffff,	//Clear all bits on exit.
				&ulNotificationValue,	//Stores the notified value.
				pdMS_TO_TICKS(msWait) );
		if( xResult == pdPASS )
		{
			//the lights have changed
			if ( ulNotificationValue & TNB_LIGHTSCHANGED )
			{
				//YYY could do something, but we don't need to
			}
		}
		else	//timeout on wait
		{
			//YYY could do things to do on periodic idle timeout
		}

#ifdef DEBUG
		//these are to tune the freertos heap size; if we have a heap
#if USE_FREERTOS_HEAP_IMPL
		g_nHeapFree = xPortGetFreeHeapSize();
		g_nMinEverHeapFree = xPortGetMinimumEverFreeHeapSize();
#else
		g_nMinEverHeapFree = (char*)platform_get_last_free_ram( 0 ) - (char*)platform_get_first_free_ram( 0 );
#endif
#if HAVE_UART1
		g_nMaxUART1TxQueue = UART1_txbuff_max();
		g_nMaxUART1RxQueue = UART1_rxbuff_max();
#endif
#if HAVE_USBCDC
		g_nMaxCDCTxQueue = CDC_txbuff_max();
		g_nMaxCDCRxQueue = CDC_rxbuff_max();
#endif
		//free stack space measurements
		g_nMinStackFreeDefault = uxTaskGetStackHighWaterMark ( defaultTaskHandle );
		g_nMinStackFreeMonitor = uxTaskGetStackHighWaterMark ( g_thMonitor );
		//XXX others
#endif
		
		//turn out the lights, the party's over
		uint32_t now = HAL_GetTick();
		uint32_t remMin = 0xffffffff;	//nothing yet
		ProcessLightOffTime ( now, &remMin, &g_lltD2, _ledOffD2 );
		ProcessLightOffTime ( now, &remMin, &g_lltD3, _ledOffD3 );

		//don't wait longer than 3 sec
		if ( remMin > 3000 )
			remMin = 3000;
		
		msWait = remMin;
	}

  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM12 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM4) {
		volatile int i = 0;
		(void)i;
		//XXX probably not going to do this way anymore
	}
	else
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM12) {
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
	volatile int i = 0;
	(void)i;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	volatile int i = 0;
	(void)i;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

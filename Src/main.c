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

RTC_HandleTypeDef hrtc __ccram;
//DMA 'handles' cannot be in CCM because the DMA controller will access some members and CCM is inaccessible to the DMA controllers.
SD_HandleTypeDef hsd;// __ccram;
DMA_HandleTypeDef hdma_sdio_rx;// __ccram;
DMA_HandleTypeDef hdma_sdio_tx;// __ccram;
UART_HandleTypeDef huart1 __ccram;

osThreadId defaultTaskHandle __ccram;
uint32_t defaultTaskBuffer[ 128 ] __ccram;
osStaticThreadDef_t defaultTaskControlBlock __ccram;

#else
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_PWR_EnableBkUpAccess();
  LL_RCC_ForceBackupDomainReset();
  LL_RCC_ReleaseBackupDomainReset();
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {
    
  }
  LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
  LL_RCC_EnableRTC();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();  
  };
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

  LL_DAC_InitTypeDef DAC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**DAC GPIO Configuration  
  PA4   ------> DAC_OUT1
  PA5   ------> DAC_OUT2 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* DAC DMA Init */
  
  /* DAC1 Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* DAC2 Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_7);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* DAC interrupt Init */
  NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(TIM6_DAC_IRQn);

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC channel OUT1 config 
  */
  DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
  DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
  DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_DISABLE;
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_1, &DAC_InitStruct);
  /** DAC channel OUT2 config 
  */
  LL_DAC_Init(DAC, LL_DAC_CHANNEL_2, &DAC_InitStruct);
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

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI1 GPIO Configuration  
  PB3   ------> SPI1_SCK
  PB4   ------> SPI1_MISO
  PB5   ------> SPI1_MOSI 
  */
  GPIO_InitStruct.Pin = FLASH_SCK_Pin|FLASH_MISO_Pin|FLASH_MOSI_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* SPI1 interrupt Init */
  NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(SPI1_IRQn);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 DMA Init */
  
  /* TIM1_UP Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_5, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_5, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_5, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_5);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_5, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_5, LL_DMA_MBURST_SINGLE);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_5, LL_DMA_PBURST_SINGLE);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 3;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 2;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerInput(TIM1, LL_TIM_TS_ITR3);
  LL_TIM_SetSlaveMode(TIM1, LL_TIM_SLAVEMODE_TRIGGER);
  LL_TIM_DisableIT_TRIG(TIM1);
  LL_TIM_DisableDMAReq_TRIG(TIM1);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 1;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1055;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 128;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_ACTIVE;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 216;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 1016;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_OC2REF);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  LL_TIM_OC_DisablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration  
  PB6   ------> TIM4_CH1 
  */
  GPIO_InitStruct.Pin = HSYNC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(HSYNC_GPIO_Port, &GPIO_InitStruct);

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

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Stream5_IRQn);
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
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LED_D2_Pin|LED_D3_Pin);

  /**/
  LL_GPIO_SetOutputPin(FLASH_CS_GPIO_Port, FLASH_CS_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOE, R0_Pin|R1_Pin|R2_Pin|G0_Pin 
                          |G1_Pin|G2_Pin|B0_Pin|B1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(VSYNC_GPIO_Port, VSYNC_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE3);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTE, LL_SYSCFG_EXTI_LINE4);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE6);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_3;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_4;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_6;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(KEY1_GPIO_Port, KEY1_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(KEY0_GPIO_Port, KEY0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinPull(PS2_CK_GPIO_Port, PS2_CK_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(KEY1_GPIO_Port, KEY1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(KEY0_GPIO_Port, KEY0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(PS2_CK_GPIO_Port, PS2_CK_Pin, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = BOGO_CARDDETECT_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(BOGO_CARDDETECT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_D2_Pin|LED_D3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|G0_Pin 
                          |G1_Pin|G2_Pin|B0_Pin|B1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = PS2_DATA_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(PS2_DATA_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = VSYNC_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(VSYNC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI3_IRQn);
  NVIC_SetPriority(EXTI4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI4_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(EXTI9_5_IRQn);

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



void EXTI_3_ISR ( void )
{
	//KEY1
}

void EXTI_4_ISR ( void )
{
	//KEY0
}

void EXTI_6_ISR ( void )
{
	//PS2_CK
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

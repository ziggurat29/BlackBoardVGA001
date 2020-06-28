//==============================================================
//impl

#include "video.h"
#include "main.h"
#include "stm32f4xx_hal.h"


extern TIM_HandleTypeDef htim1;// __ccram;
extern TIM_HandleTypeDef htim4;// __ccram;
extern DMA_HandleTypeDef hdma_tim1_up;// __ccram;



//XXX it's not yet clear if the HAL is going to be suitable for this
//implementation.  I'm a little concerned it will add a bit too much
//latency for this timing critical stuff.  I'm also not sure that all
//features are exposed (e.g. interrupt on tim4 oc ch 3? as opposed
//to reload, which seems to be the out-of-box interrupt supported).
//I might go semi-low-level on many of these resources.


//resources:
//PB7:  gpio used as vsync
//PB6:  used as HSYNC from TIM4CH1
//TIM4:  horizontal line timing; drives the state machine
//  OC1:  used in PWM mode to create the HSYNC pulse
//  OC2:  used to trigger the start of video out; set as TRGO
//  OC3:  used to interrupt at end of scan line (XXX may change to the DMA complete interrupt)
//        this gives the maximum amount of time to prep the next line into the scanline buffer

//TIM1:  drive dma pixel clock; set in slave mode, gated, from ITR3 (TIM4)
//  OC1:  used in PWM mode to provide pix clock for testing
//DMA2CH5:  move pixels to port E high byte


//__ram2:  a 16 kiB memory section that has a separate port on the bus matrix,
//         and so can be (carefully) used with DMA2 to have contention free
//         access during video out


//HAL-based init code copied here for reference
//This was when I was using TIM to drive the pixel clock; TIM4  triggers TIM1
//to start sending out pixels.
//For some reason I can only get a max of about 600 horz, though.  I did verify
//that TIM1 is clocking at full (168 MHz) speed, but I guess the DMA just can't
//keep up.
#if 0

//config alt func gpio relevant to the timers we use
void XXX_HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration    
    PA8     ------> TIM1_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }
  else if(htim->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration    
    PD13     ------> TIM4_CH2
    PD14     ------> TIM4_CH3
    PB6     ------> TIM4_CH1 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HSYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(HSYNC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }

}

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

static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, R0_Pin|R1_Pin|R2_Pin|G0_Pin 
                          |G1_Pin|G2_Pin|B0_Pin|B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : R0_Pin R1_Pin R2_Pin G0_Pin 
                           G1_Pin G2_Pin B0_Pin B1_Pin */
  GPIO_InitStruct.Pin = R0_Pin|R1_Pin|R2_Pin|G0_Pin 
                          |G1_Pin|G2_Pin|B0_Pin|B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : VSYNC_Pin */
  GPIO_InitStruct.Pin = VSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(VSYNC_GPIO_Port, &GPIO_InitStruct);

}

#endif






//XXX this first implementation is going to realize 800x600 timing.  It might
//not actually be 800x600 resolution, though -- not sure if I can crank out
//800 pix

//XXX this first implementation's video timing will necessarily be an
//approximation because I am running the system clock at full-bore of 168 MHz
//(which is necessary if I want to be able to do USB due to the 48 MHz
//requirement.  If USB is forewent, then we can run the system at 160 MHz, and
//ostensibly create exact timing.  The 168 MHz is 5% faster.  I can fix the
//sync timings within less than a percent by tweaking the count values, but
//the pixel clock (driven by DMA) can't be adjusted.  Since VGA is analog,
//this is OK, but will have the effect of compressing the X dimension a bit.
//We'll see how bad this looks.



//define line vsync first
#define VGA_FIRST_VSYNC 1
//define line vsync last
#define VGA_LAST_VSYNC 5
//define visible first
#define VGA_FIRST_VISIBLE 28
//define visible last (resets line counter)
#define VGA_LAST_VISIBLE 628

//line counter
volatile unsigned int g_nThisVidLine __ccram;
//state
typedef enum VgaState
{
	VGA_BLANK = 0x00,		//no vid, no prep scan
	VGA_SETUP = 0x01,		//no vid, prep scan
	VGA_ACTIVE = 0x03,		//vid, prep scan
	VGA_STOPPING = 0x02,	//vid, no prep scan
} VgaState;
VgaState g_eVgaState __ccram;


//scan buffer (in sram2); round up line length to word, and add a word that we will clear
//#define VGA_SCAN_LINE_WORDS ((800+sizeof(uint32_t)-1)/sizeof(uint32_t))
//XXX max I can currently get for some reason that needs to be explored
#define VGA_SCAN_LINE_WORDS ((600+sizeof(uint32_t)-1)/sizeof(uint32_t))
uint32_t g_abyScan[VGA_SCAN_LINE_WORDS+1] __ram2;

//XXX hack for testing
void _test_patternScanBuffer ( void )
{
	//  15 14 13 12  11 10  9  8
	//  b1 b0 g2 g1  g0 r2 r1 r0
	g_abyScan[0] = 0x00ff00ff;	//first visible word

	//middle visible words
	for ( size_t nIdx = 1; nIdx < VGA_SCAN_LINE_WORDS-1; ++nIdx )
	{
		g_abyScan[nIdx] = 0x00000000;
	}

	g_abyScan[VGA_SCAN_LINE_WORDS-1] = 0xff00ff00;	//last visible word

	g_abyScan[VGA_SCAN_LINE_WORDS] = 0x00000000;	//final word always black
}

//(this will not fit, of course)
//frame buffer has a word one the left and right to make some drawing routines easier by avoiding clipping logic
//uint32_t g_abyFB[1+(VGA_SCAN_LINE_LENGTH+1)+1][600];


//====================================================
//hooks related to video support


//TIM4 OC3 is used to signal end of scan line
//XXX may become DMA transfer complete; probably can't since that's only on visible lines
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
	//currently, we implicitly know this is from TIM4 OC3

	//if OC3; end of video
	//if (htim->Instance == TIM4) {
	//XXX already cleared by caller if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
	//if ( HAL_TIM_ACTIVE_CHANNEL_3 == htim->Channel )
	{
		if ( g_eVgaState & 0x02)	//arm for video out?
		{
			//first, force OC2 inactive since the ITRx seems level-triggered and there is no 'auto reset' mode
			uint32_t saveCCMR1 = TIM4->CCMR1;
			saveCCMR1 &= ~TIM_CCMR1_OC2M_Msk;
			saveCCMR1 |= (4 << TIM_CCMR1_OC2M_Pos);	//'TIM_OCMODE_FORCED_INACTIVE'
			TIM4->CCMR1 = saveCCMR1;

			//now that's off, turn off TIM1 to effectively re-arm for next trigger
			uint32_t saveSMCR = TIM1->SMCR;
			TIM1->SMCR = 0;	//disable slave mode; we can't stop the timer unless we do this first
			TIM1->CR1 &= ~(TIM_CR1_CEN);	//stop clocking; effectively waiting for next trigger
			TIM1->CNT = 0;	//in trigger mode, we must explicitly reset
			TIM1->SMCR = saveSMCR;	//re-enable slave mode

			//This dorky sequence causes any pending DRQ's to be removed prior
			//setting up for another transfer.  Otherwise we'd get one transfer
			//immediately, rather than waiting for the trigger event later.
			__HAL_TIM_DISABLE_DMA ( &htim1, TIM_DMA_UPDATE );
			__HAL_TIM_ENABLE_DMA ( &htim1, TIM_DMA_UPDATE );

			//set back to go active when we match again, and thus trigger once more
			saveCCMR1 &= ~TIM_CCMR1_OC2M_Msk;
			saveCCMR1 |= (1 << TIM_CCMR1_OC2M_Pos);	//'TIM_OCMODE_ACTIVE'
			TIM4->CCMR1 = saveCCMR1;

			//start the DMA (eventually, when TIM1 gets triggered to start)
			HAL_DMA_Start_IT(&hdma_tim1_up, 
					(uint32_t)&g_abyScan[0], 
					(uint32_t)0x40021015, 	//port E, high byte
					sizeof(g_abyScan));
		}

		if ( g_eVgaState & 0x01)	//signal to prep subsequent scan line?
		{
//XXX
		}

		//do VGA state machine
		unsigned int nNextVidLine = g_nThisVidLine + 1;
		if ( VGA_FIRST_VSYNC == nNextVidLine || VGA_LAST_VSYNC == nNextVidLine )
		{
			//we toggle for simplicity with different polarities, but note your init to ensure it starts correctly!
			HAL_GPIO_TogglePin(VSYNC_GPIO_Port, VSYNC_Pin);
		}
		else if ( VGA_FIRST_VISIBLE-1 == nNextVidLine )
		{
			g_eVgaState = VGA_SETUP;	//xition
//XXX prep
		}
		else if ( VGA_FIRST_VISIBLE == nNextVidLine )
		{
			g_eVgaState = VGA_ACTIVE;	//xition
		}
		else if ( VGA_LAST_VISIBLE-1 == nNextVidLine )
		{
			g_eVgaState = VGA_STOPPING;	//xition
		}
		else if ( VGA_LAST_VISIBLE == nNextVidLine )
		{
			g_eVgaState = VGA_BLANK;	//xition
			nNextVidLine = 0;

			//turn off the output compare and pix clock

			//first, force OC2 inactive since the ITRx seems level-triggered and there is no 'auto reset' mode
			uint32_t saveCCMR1 = TIM4->CCMR1;
			saveCCMR1 &= ~TIM_CCMR1_OC2M_Msk;
			saveCCMR1 |= (4 << TIM_CCMR1_OC2M_Pos);	//'TIM_OCMODE_FORCED_INACTIVE'
			TIM4->CCMR1 = saveCCMR1;

			//now that's off, turn off TIM1 to effectively re-arm for next trigger
			uint32_t saveSMCR = TIM1->SMCR;
			TIM1->SMCR = 0;	//disable slave mode; we can't stop the timer unless we do this first
			TIM1->CR1 &= ~(TIM_CR1_CEN);	//stop clocking; effectively waiting for next trigger
			TIM1->CNT = 0;	//in trigger mode, we must explicitly reset
			TIM1->SMCR = saveSMCR;	//re-enable slave mode

			__HAL_TIM_DISABLE_DMA ( &htim1, TIM_DMA_UPDATE );
		}
		g_nThisVidLine = nNextVidLine;
	}

}



//====================================================
//DMA support
//	The DMA2 channel 5 is attached to the TIM1 'update' (i.e. rollover) action,
//and we use that to clock out our pixels.  The DMA will directly fetch byte
//pixels from memory and push them into the PORTE high byte.

//XXX I don't think I'm going to wind up using these, but they are handy for tests


void HAL_DMA_CB_XferCpl(DMA_HandleTypeDef* hdma)
{
	volatile int i = 0;
	(void)i;
	_ledToggleD2();
}

void HAL_DMA_CB_HalfCpl(DMA_HandleTypeDef* hdma)
{
	volatile int i = 0;
	(void)i;
}

void HAL_DMA_CB_Error(DMA_HandleTypeDef* hdma)
{
	volatile int i = 0;
	(void)i;
}

void HAL_DMA_CB_Abort(DMA_HandleTypeDef* hdma)
{
	volatile int i = 0;
	(void)i;
}



//====================================================




//called once at reset to get things ready; must be done first
void Video_Initialize ( void )
{
	g_nThisVidLine = 0;	//(must explicitly init since ccram)
	g_eVgaState = VGA_BLANK;	//(must explicitly init since ccram)
	HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_RESET);	//must set because we toggle; moreover this is for positive polarity
	
	_test_patternScanBuffer();	//hack for initial testing

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
	htim4.Instance->CCR3 = 1067;
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

	//XXX start the line timing scheme
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

	//XXX start the pixel timing scheme
	//HAL_TIM_Base_Start(&htim1); //Starts the pix clock generation
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)	//Starts the pix clock test signal generation
	{
		Error_Handler();	//horror
	}

	//register DMA callbacks
	//XXX probably not going to use; but they are handy now for testing
	HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_CPLT_CB_ID, HAL_DMA_CB_XferCpl);
	HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_HALFCPLT_CB_ID, HAL_DMA_CB_HalfCpl);
	HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_ERROR_CB_ID, HAL_DMA_CB_Error);
	HAL_DMA_RegisterCallback(&hdma_tim1_up, HAL_DMA_XFER_ABORT_CB_ID, HAL_DMA_CB_Abort);
}



//shut everything down; maybe for a mode change
void Video_Uninitialize ( void )
{
}




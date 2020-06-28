//==============================================================
//impl

#include "video.h"
#include "main.h"
#include "lamps.h"




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
//this is run at the highest priority, and may /not/ call FreeRTOS functions
void TIM4OC3_ISR(void)
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
		TIM1->DIER &= ~TIM_DMA_UPDATE;	//disable dma on update
		TIM1->DIER |= TIM_DMA_UPDATE;	//enable dma on update

		//set back to go active when we match again, and thus trigger once more
		saveCCMR1 &= ~TIM_CCMR1_OC2M_Msk;
		saveCCMR1 |= (1 << TIM_CCMR1_OC2M_Pos);	//'TIM_OCMODE_ACTIVE'
		TIM4->CCMR1 = saveCCMR1;

		//setup for the transfer
		LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_5, (uint32_t)&g_abyScan[0]);
		LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_5, 0x40021015);
		LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_5, sizeof(g_abyScan));

		//clear all interrupt flags
		DMA2->HIFCR = DMA_HIFCR_CHTIF5|DMA_HIFCR_CTCIF5|DMA_HIFCR_CTEIF5|DMA_HIFCR_CDMEIF5|DMA_HIFCR_CFEIF5;

		//start the DMA (eventually, when TIM1 gets triggered to start)
		LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_5);
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
		LL_GPIO_TogglePin(VSYNC_GPIO_Port, VSYNC_Pin);
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

		TIM1->DIER &= ~TIM_DMA_UPDATE;	//disable dma on update
	}
	g_nThisVidLine = nNextVidLine;

}



//====================================================




//called once at reset to get things ready; must be done first
void Video_Initialize ( void )
{
	g_nThisVidLine = 0;	//(must explicitly init since ccram)
	g_eVgaState = VGA_BLANK;	//(must explicitly init since ccram)
	LL_GPIO_ResetOutputPin(VSYNC_GPIO_Port,VSYNC_Pin);	//must set because we toggle; moreover this is for positive polarity

	_test_patternScanBuffer();	//hack for initial testing

	//this must be done to get the PWM output started
	//'ideal' 800x600x60 timings if we can have a 160 MHz clock
	TIM4->PSC = 1;		//divide (160/2) /(1+1) = 40 MHz pix clock
	//TIM4->ARR = 1055;		//line time+1 = (128+88+800+40) pix 26.4 us
	//TIM4->CCR1 = 128;		//hsync
	//TIM4->CCR2 = 216;		//pix start
	//TIM4->CCR3 = 1016;	//pix end
	//but 160 MHz means we can't have USB.  Attempt to approximate with 168 MHz; +5%
	TIM4->ARR = 1107;		//line time+1 = 26.38 us
	TIM4->CCR1 = 134;
	TIM4->CCR2 = 227;
	TIM4->CCR3 = 1067;
	//set the hsync polarity as needed (positive)
	TIM4->CCER = ( TIM4->CCER & ~TIM_CCER_CC1P ) | 0;

/*
	//640x480x60 approximation at 168 MHz
	TIM4->ARR = 1334;		//line time+1 = 31.76 us, 31.48 kHz
	TIM4->CCR1 = 160;
	TIM4->CCR2 = 240;
	TIM4->CCR3 = 1308;

	//set the hsync polarity as needed (negative)
	TIM4->CCER = ( TIM4->CCER & ~TIM_CCER_CC1P ) | TIM_CCER_CC1P;
*/

	//start the line timing scheme
	LL_TIM_EnableCounter(TIM4);	//Starts the state machine generation
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	SET_BIT(TIM4->CCER, TIM_CCER_CC1E);	//Starts the HSYNC signal generation
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE);	//Starts the start of scan trigger
	LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FROZEN);
	LL_TIM_EnableIT_CC3(TIM4);	//Starts the end of scan interrupt

	//start the pixel timing scheme
	LL_TIM_EnableCounter(TIM1); //Starts the pix clock generation
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);

}



//shut everything down; maybe for a mode change
void Video_Uninitialize ( void )
{
}




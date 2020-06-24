//==============================================================
//impl

#include "video.h"
#include "main.h"
#include "stm32f4xx_hal.h"



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
//  OC2:  used to trigger the start of video out
//  OC3:  used to trigger the end-of video out (XXX may change to the DMA complete interrupt)
//        this gives the maximum amount of time to prep the next line into the scanline buffer

//TIM1:  drive dma pixel clock
//DMA2CH5:  move pixels

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





//called once at reset to get things ready; must be done first
void Video_Initialize ( void )
{
}


//shut everything down; maybe for a mode change
void Video_Uninitialize ( void )
{
}


/*
	//testing tweaking the vsync line
	{ HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_SET); }
	{ HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_RESET); }
*/


//==============================================================
//impl

#include "video.h"
#include "main.h"
#include "stm32f4xx_hal.h"



//called once at reset to get things ready; must be done first
void Video_Initialize ( void )
{
}


/*
	//testing tweaking the vsync line
	{ HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_SET); }
	{ HAL_GPIO_WritePin(VSYNC_GPIO_Port, VSYNC_Pin, GPIO_PIN_RESET); }
*/


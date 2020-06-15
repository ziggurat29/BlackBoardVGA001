//==============================================================
//This declares stuff related to indicator lamps.
//This module is part of the BlackBoardVGA001 project.

#ifndef __LAMPS_H
#define __LAMPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


//these are the simple 'on/off' methods for each of the boards lamps
//the board has two leds; they are both colored green, so we just refer to them by silkscreen
void _ledOnD2 ( void );
void _ledOffD2 ( void );
void _ledToggleD2 ( void );

void _ledOnD3 ( void );
void _ledOffD3 ( void );
void _ledToggleD3 ( void );


//these structs are used in the calls below and maintain state for the one-shot functions
typedef struct LED_LightTime LED_LightTime;
extern LED_LightTime g_lltD2;
extern LED_LightTime g_lltD3;


//these methods provide 'one-shot' lighting for a period of time.
//this method is intended to be used anywhere, to light a lamp for a period of time
void LightLamp ( uint32_t durMS, LED_LightTime* pllt, void(*pfnOn)(void) );

//this method is intended to be used only in the default task, to maintain and
//turn off the lamp after it's time has expired.
void ProcessLightOffTime ( uint32_t now, uint32_t* pnRemMin, LED_LightTime* pllt, void(*pfnOff)(void) );



#ifdef __cplusplus
}
#endif

#endif

//==============================================================
//This declares stuff related to video output.
//This module is part of the BlackBoardVGA001 project.

#ifndef __VIDEO_H
#define __VIDEO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


//called once at reset to get things ready; must be done first
void Video_Initialize ( void );


#ifdef __cplusplus
}
#endif

#endif



#include "task_monitor.h"
#include "command_processor.h"
#include "task_notification_bits.h"
#include "BlackBoard001_commands.h"

#include <string.h>


//the task that runs an interactive monitor on the USB data
#ifndef __ccram
#define __ccram
#endif
osThreadId g_thMonitor __ccram = NULL;
uint32_t g_tbMonitor[ 128 ] __ccram;
osStaticThreadDef_t g_tcbMonitor __ccram;


const IOStreamIF* g_pMonitorIOIf __ccram = NULL;	//the IO device to which the monitor is attached



//====================================================
//Monitor task
//The monitor is a command processing interface attached to the USB CDC virtual
//serial port.  It processes incoming commands from the user.



void Monitor_Initialize ( void )
{
	//(XXX nothing yet)
}



//client connect; optional
void Monitor_ClientConnect ( int bIsConnect )
{
	if ( bIsConnect )
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR ( g_thMonitor, TNB_MON_CLIENT_CONNECT, eSetBits, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
	else
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR ( g_thMonitor, TNB_MON_CLIENT_DISCONNECT, eSetBits, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}



//DAV; data is available on the stream interface
void Monitor_DAV ( void )
{
	//YYY you could use this opportunity to signal an event
	//Note, this is called at ISR time
	if ( NULL != g_thMonitor )	//only if we have a notificand
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR ( g_thMonitor, TNB_MON_DAV, eSetBits, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}



//TBMT; 
void Monitor_TBMT ( void )
{
	if ( NULL != g_thMonitor )	//only if we have a notificand
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR ( g_thMonitor, TNB_MON_TBMT, eSetBits, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}



//implementation for the command processor; bind IO to a stream


void thrdfxnMonitorTask ( void const* argument )
{
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
			//if we got a new client connection, do a greeting
			if ( ulNotificationValue & TNB_MON_CLIENT_CONNECT )
			{
				CWCMD_SendGreeting ( g_pMonitorIOIf );
				CWCMD_SendPrompt ( g_pMonitorIOIf );
			}
			if ( ulNotificationValue & TNB_MON_DAV )
			{
				//we use the non-blocking version in this notification loop
				CMDPROC_process_nb ( g_pMonitorIOIf, g_aceCommands, g_nAceCommands );
			}
			if ( ulNotificationValue & TNB_MON_SETMODE )
			{
				//extract parameter field as mode
				//enum MONMODE eMode = (enum MONMODE)( ulNotificationValue & 0x0000ffff );
				//XXX III
			}
		}
	}
}




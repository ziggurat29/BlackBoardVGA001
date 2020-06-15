//==============================================================
//This provides implementation for the commands relevant for the
//BlackBoard001 project.

#ifndef __BLACKBOARD001_COMMANDS_H
#define __BLACKBOARD001_COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "command_processor.h"


extern const CmdProcEntry g_aceCommands[];
extern const size_t g_nAceCommands;

//send the 'greeting' when a client first connects
void CWCMD_SendGreeting ( const IOStreamIF* pio );

//send the 'prompt' that heads a command line
void CWCMD_SendPrompt ( const IOStreamIF* pio );


#ifdef __cplusplus
}
#endif

#endif

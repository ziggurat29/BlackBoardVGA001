//==============================================================
//This provides implementation for the commands relevant for the
//BlackBoard001 project.
//impl

#include "BlackBoard001_commands.h"
#include "cmsis_os.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "util_altlib.h"


#ifndef COUNTOF
#define COUNTOF(arr) (sizeof(arr)/sizeof(arr[0]))
#endif



//forward decl command handlers
static CmdProcRetval cmdhdlHelp ( const IOStreamIF* pio, const char* pszszTokens );
static CmdProcRetval cmdhdlSet ( const IOStreamIF* pio, const char* pszszTokens );
static CmdProcRetval cmdhdlReboot ( const IOStreamIF* pio, const char* pszszTokens );
static CmdProcRetval cmdhdlDump ( const IOStreamIF* pio, const char* pszszTokens );

#ifdef DEBUG
static CmdProcRetval cmdhdlDiag ( const IOStreamIF* pio, const char* pszszTokens );
#endif



//the array of command descriptors our application supports
const CmdProcEntry g_aceCommands[] = 
{
	{ "set", cmdhdlSet, "set a setting value, or list all settings" },
	{ "reboot", cmdhdlReboot, "restart the board" },
	{ "dump", cmdhdlDump, "dump memory; [addr] [count]" },
#ifdef DEBUG
	{ "diag", cmdhdlDiag, "show diagnostic info (DEBUG build only)" },
#endif

	{ "help", cmdhdlHelp, "get help on a command; help [cmd]" },
};
const size_t g_nAceCommands = COUNTOF(g_aceCommands);



//========================================================================
//command helpers (XXX probably break out for general use)


static void _cmdPutChar ( const IOStreamIF* pio, char c )
{
	pio->_transmitCompletely ( pio, &c, 1, TO_INFINITY );
}


static void _cmdPutString ( const IOStreamIF* pio, const char* pStr )
{
	size_t nLen = strlen ( pStr );
	pio->_transmitCompletely ( pio, pStr, nLen, TO_INFINITY );
}


static void _cmdPutCRLF ( const IOStreamIF* pio )
{
	_cmdPutString ( pio, "\r\n" );
}


static void _cmdPutInt ( const IOStreamIF* pio, long val, int padding )
{
	char ach[16];
	my_itoa_sortof ( ach, val, padding );
	_cmdPutString ( pio, ach );
}


static void _cmdPutFloat ( const IOStreamIF* pio, float val )
{
	char ach[20];
	my_ftoa ( ach, val );
	_cmdPutString ( pio, ach );
}


//simple parser of a hex byte (two chars assumed)
static uint8_t _parseHexByte ( const char* pszToken )
{
	uint8_t val;

	val = 0;
	for ( int nIter = 0; nIter < 2;  ++nIter )
	{
		val <<= 4;
		if ( *pszToken <= '9' )
		{
			val += (*pszToken - '0');
		}
		else if ( *pszToken <= 'F' )
		{
			val += (*pszToken - 'A' + 10);
		}
		else
		{
			val += (*pszToken - 'a' + 10);
		}
		++pszToken;
	}
	return val;
}


//simple parser of an integer value (can be hex with '0x' prefix)
static uint32_t _parseInt ( const char* pszToken )
{
	uint32_t val;

	val = 0;
	//see if it starts with 0x meaning 'hex'
	if ( '0' == pszToken[0] && ( 'x' == pszToken[1] || 'X' == pszToken[1] ) )
	{
		pszToken += 2;
		while ( '\0' != *pszToken )
		{
			val <<= 4;
			if ( *pszToken <= '9' )
			{
				val += (*pszToken - '0');
			}
			else if ( *pszToken <= 'F' )
			{
				val += (*pszToken - 'A' + 10);
			}
			else
			{
				val += (*pszToken - 'a' + 10);
			}
			++pszToken;
		}
	}
	else
	{
		//otherwise, interpret it as decimal
		while ( '\0' != *pszToken )
		{
			val *= 10;
			val += (*pszToken - '0');
			++pszToken;
		}
	}

	return val;
}



//purge a string of anything other than digits
static int _cramDigits ( char* pszDest, const char* pszSrc )
{
	char* pszAt = pszDest;
	while ( 1 )
	{
		if ( '\0' == *pszSrc )
		{	//end; copy, do not advance
			*pszAt = *pszSrc;
			break;
		}
		else if ( isdigit ( *pszSrc ) )
		{	//keep; copy, advance
			*pszAt = *pszSrc;
			++pszAt;
			++pszSrc;
		}
		else
		{	//skip; advance only source
			++pszSrc;
		}
	}
	return pszAt - pszDest;	//return length
}



static int _setDate ( const IOStreamIF* pio, const char* pszDate )
{
	int nDateLen = strlen ( pszDate );
	//check for too long for fixed size buffers
	if ( nDateLen > 10 )
	{
		_cmdPutString ( pio, "date requires yyyy-mm-dd\r\n" );
		return 0;
	}
	char achDate[11];
	nDateLen = _cramDigits ( achDate, pszDate );
	if ( 8 != nDateLen )
	{
		_cmdPutString ( pio, "date requires yyyy-mm-dd\r\n" );
		return CMDPROC_ERROR;
	}

	LL_RTC_DateTypeDef RTC_DateStruct = {0};
	RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_SUNDAY;	//(arbitrary)
	RTC_DateStruct.Day = my_atoul ( &achDate[6], NULL );
	achDate[6] = '\0';
	RTC_DateStruct.Month = my_atoul ( &achDate[4], NULL );
	achDate[4] = '\0';
	RTC_DateStruct.Year = my_atoul ( &achDate[0], NULL ) - 2000;
	LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_DateStruct);

	return 1;
}


static int _setTime ( const IOStreamIF* pio, const char* pszTime )
{
	int nTimeLen = strlen ( pszTime );
	//check for too long for fixed size buffers
	if ( nTimeLen > 8 )
	{
		_cmdPutString ( pio, "time requires hh:mm:ss\r\n" );
		return 0;
	}
	char achTime[9];
	nTimeLen = _cramDigits ( achTime, pszTime );
	if ( 6 != nTimeLen && 4 != nTimeLen )	//(we accept without seconds)
	{
		_cmdPutString ( pio, "time requires hh:mm:ss\r\n" );
		return CMDPROC_ERROR;
	}

	LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
	//careful:  the following works only because an empty field == zero
	RTC_TimeStruct.Seconds = my_atoul ( &achTime[4], NULL );
	achTime[4] = '\0';
	RTC_TimeStruct.Minutes = my_atoul ( &achTime[2], NULL );
	achTime[2] = '\0';
	RTC_TimeStruct.Hours = my_atoul ( &achTime[0], NULL );
	LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, &RTC_TimeStruct);

	return 1;
}



static char _nybbleToChar ( uint8_t nyb )
{
	char ret = nyb + '0';
	if ( nyb > 9 )
		ret += 'a' - '9' - 1;
	return ret;
}



static void _putHexUint32 ( const IOStreamIF* pio, uint32_t val )
{
	char ach[9];
	char* pach = &ach[8];	//end
	*pach = '\0';
	do	//easiest to go backwards
	{
		--pach;
		*pach = _nybbleToChar ( val & 0xf );
		val >>= 4;
	} while ( pach != ach );
	_cmdPutString ( pio, ach );
}




//========================================================================


//send the 'greeting' when a client first connects
void CWCMD_SendGreeting ( const IOStreamIF* pio )
{
	_cmdPutString ( pio, "Welcome to the BlackBoardVGA001 Command Processor\r\n" );
}


//send the 'prompt' that heads a command line
void CWCMD_SendPrompt ( const IOStreamIF* pio )
{
	_cmdPutString ( pio, "> " );
}


//========================================================================
//simple command handlers


static CmdProcRetval cmdhdlHelp ( const IOStreamIF* pio, const char* pszszTokens )
{
	//get next token; we will get help on that
	int nIdx;
	if ( NULL != pszszTokens && '\0' != *pszszTokens &&
		-1 != ( nIdx = CMDPROC_findProcEntry ( pszszTokens, g_aceCommands, g_nAceCommands ) ) )
	{
		//emit help information for this one command
		_cmdPutString ( pio, g_aceCommands[nIdx]._pszHelp );
		_cmdPutCRLF(pio);
	}
	else
	{
		//if unrecognised command
		if ( NULL != pszszTokens && '\0' != *pszszTokens )
		{
			_cmdPutString ( pio, "The command '" );
			_cmdPutString ( pio, pszszTokens );
			_cmdPutString ( pio, "' is not recognized.\r\n" );
		}

		//list what we've got
		_cmdPutString ( pio, "help is available for:\r\n" );
		for ( nIdx = 0; nIdx < g_nAceCommands; ++nIdx )
		{
			_cmdPutString ( pio, g_aceCommands[nIdx]._pszCommand );
			_cmdPutCRLF(pio);
		}
	}

	CWCMD_SendPrompt ( pio );
	return CMDPROC_SUCCESS;
}


static CmdProcRetval cmdhdlSet ( const IOStreamIF* pio, const char* pszszTokens )
{
	const char* pszSetting = pszszTokens;
	if ( NULL == pszSetting )
	{
		//list all settings and their current value

		//RTC date time
		_cmdPutString ( pio, "datetime:  " );
		_cmdPutInt ( pio, __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetYear(RTC)) + 2000, 4 );
		_cmdPutChar ( pio, '-' );
		_cmdPutInt ( pio, __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetMonth(RTC)), 2 );
		_cmdPutChar ( pio, '-' );
		_cmdPutInt ( pio, __LL_RTC_CONVERT_BCD2BIN(LL_RTC_DATE_GetDay(RTC)), 2 );
		_cmdPutChar ( pio, ' ' );
		_cmdPutInt ( pio, __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC)), 2 );
		_cmdPutChar ( pio, ':' );
		_cmdPutInt ( pio, __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC)), 2 );
		_cmdPutChar ( pio, ':' );
		_cmdPutInt ( pio, __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetSecond(RTC)), 2 );
		_cmdPutCRLF(pio);

		CWCMD_SendPrompt ( pio );
		return CMDPROC_SUCCESS;
	}

	//next, get the 'value' which all settings must have at least one
	const char* pszValue;
	pszValue = CMDPROC_nextToken ( pszSetting );
	if ( NULL == pszValue )
	{
		_cmdPutString ( pio, "set " );
		_cmdPutString ( pio, pszSetting );
		_cmdPutString ( pio, " requires a setting value\r\n" );
		CWCMD_SendPrompt ( pio );
		return CMDPROC_ERROR;
	}


	if ( 0 == strcmp ( "date", pszSetting ) )
	{
		_setDate ( pio, pszValue );	//(error message already emitted)
	}
	else if ( 0 == strcmp ( "time", pszSetting ) )
	{
		_setTime ( pio, pszValue );	//(error message already emitted)
	}
	else if ( 0 == strcmp ( "datetime", pszSetting ) )
	{
		const char* pszTime;
		pszTime = CMDPROC_nextToken ( pszValue );
		if ( NULL == pszTime )
		{
			_cmdPutString ( pio, "datetime requires yyyy-mm-dd hh:mm:ss\r\n" );
			CWCMD_SendPrompt ( pio );
			return CMDPROC_ERROR;
		}

		if ( _setDate ( pio, pszValue ) )	//(error message already emitted)
		{
			_setTime ( pio, pszTime );
		}
	}

	else
	{
		_cmdPutString ( pio, "error:  the setting " );
		_cmdPutString ( pio, pszSetting );
		_cmdPutString ( pio, "is not a valid setting name\r\n" );
		CWCMD_SendPrompt ( pio );
		return CMDPROC_ERROR;
	}

	_cmdPutString ( pio, "done\r\n" );

	CWCMD_SendPrompt ( pio );
	return CMDPROC_SUCCESS;
}



static CmdProcRetval cmdhdlReboot ( const IOStreamIF* pio, const char* pszszTokens )
{
	_cmdPutString( pio, "rebooting\r\n" );
	osDelay ( 500 );	//delay a little to let all that go out before we reset
	NVIC_SystemReset();
	return CMDPROC_SUCCESS;
}



#ifdef DEBUG

//diagnostic variables in main.c
extern volatile size_t g_nHeapFree;
extern volatile size_t g_nMinEverHeapFree;
#if HAVE_UART1
extern volatile int g_nMaxUART1TxQueue;
extern volatile int g_nMaxUART1RxQueue;
#endif
#if HAVE_USBCDC
extern volatile int g_nMaxCDCTxQueue;
extern volatile int g_nMaxCDCRxQueue;
#endif
extern volatile int g_nMinStackFreeDefault;
extern volatile int g_nMinStackFreeMonitor;

#define USE_FREERTOS_HEAP_IMPL 1
#if USE_FREERTOS_HEAP_IMPL
//we implemented a 'heapwalk' function
typedef int (*CBK_HEAPWALK) ( void* pblk, uint32_t nBlkSize, int bIsAlloc, void* pinst );
extern int vPortHeapWalk ( CBK_HEAPWALK pfnWalk, void* pinst );


//heapwalk suspends all tasks, so cannot do io in the callback.  we collect all
//stats into a structure.
typedef struct HeapInfo
{
	int nFree;				//free blocks
	size_t nSmallestFree;	//smallest free
	size_t nLargestFree;	//largest free
	int nAlloc;				//alloc blocks
	size_t nSmallestAlloc;	//smallest alloc
	size_t nLargestAlloc;	//largest alloc
} HeapInfo;


int StatsHWcbk ( void* pblk, uint32_t nBlkSize, int bIsAlloc, void* pinst )
{
	HeapInfo* hi = (HeapInfo*) pinst;

	if ( bIsAlloc )
	{
		if ( hi->nAlloc > 0 )
		{
			++hi->nAlloc;
			if ( nBlkSize < hi->nSmallestAlloc )
			{
				hi->nSmallestAlloc = nBlkSize;
			}
			if ( nBlkSize > hi->nLargestAlloc )
			{
				hi->nLargestAlloc = nBlkSize;
			}
		}
		else
		{
			hi->nAlloc = 1;
			//init the fields with this first block info
			hi->nSmallestAlloc = nBlkSize;
			hi->nLargestAlloc = nBlkSize;
		}
	}
	else
	{
		if ( hi->nFree > 0 )
		{
			++hi->nFree;
			if ( nBlkSize < hi->nSmallestFree )
			{
				hi->nSmallestFree = nBlkSize;
			}
			if ( nBlkSize > hi->nLargestFree )
			{
				hi->nLargestFree = nBlkSize;
			}
		}
		else
		{
			hi->nFree = 1;
			//init the fields with this first block info
			hi->nSmallestFree = nBlkSize;
			hi->nLargestFree = nBlkSize;
		}
	}

	return 1;	//keep walking
}



#endif


static CmdProcRetval cmdhdlDiag ( const IOStreamIF* pio, const char* pszszTokens )
{
	//list what we've got
	_cmdPutString ( pio, "diagnostic vars:\r\n" );

	_cmdPutString ( pio, "Heap: size: " );
	_cmdPutInt ( pio, configTOTAL_HEAP_SIZE, 0 );
	_cmdPutString ( pio, ", free now: " );
	_cmdPutInt ( pio, g_nHeapFree, 0 );
	_cmdPutString ( pio, ", used: " );
	_cmdPutInt ( pio, configTOTAL_HEAP_SIZE - g_nHeapFree, 0 );
	_cmdPutString ( pio, ", min free ever: " );
	_cmdPutInt ( pio, g_nMinEverHeapFree, 0 );
	_cmdPutCRLF(pio);

	//collect heap alloc stats
	HeapInfo hi;
	hi.nFree = 0;	//sentinel to cause init with first block
	hi.nAlloc = 0;	//sentinel to cause init with first block
	vPortHeapWalk ( StatsHWcbk, &hi );
	//emit datas
	_cmdPutString ( pio, "  #free: " );
	_cmdPutInt ( pio, hi.nFree, 0 );
	if ( hi.nFree > 0 )
	{
		_cmdPutString ( pio, ", smallest: " );
		_cmdPutInt ( pio, (int)hi.nSmallestFree, 0 );
		_cmdPutString ( pio, ", largest: " );
		_cmdPutInt ( pio, (int)hi.nLargestFree, 0 );
	}
	_cmdPutString ( pio, "\r\n  #alloc: " );
	_cmdPutInt ( pio, hi.nAlloc, 0 );
	if ( hi.nAlloc > 0 )
	{
		_cmdPutString ( pio, ", smallest: " );
		_cmdPutInt ( pio, (int)hi.nSmallestAlloc, 0 );
		_cmdPutString ( pio, ", largest: " );
		_cmdPutInt ( pio, (int)hi.nLargestAlloc, 0 );
	}
	_cmdPutCRLF(pio);

#if HAVE_UART1
	_cmdPutString ( pio, "UART1 max RX queue: " );
	_cmdPutInt ( pio, g_nMaxUART1RxQueue, 0 );
	_cmdPutString ( pio, ", max TX queue: " );
	_cmdPutInt ( pio, g_nMaxUART1TxQueue, 0 );
	_cmdPutCRLF(pio);
#endif

#if HAVE_USBCDC
	_cmdPutString ( pio, "CDC max RX queue: " );
	_cmdPutInt ( pio, g_nMaxCDCRxQueue, 0 );
	_cmdPutString ( pio, ", max TX queue: " );
	_cmdPutInt ( pio, g_nMaxCDCTxQueue, 0 );
	_cmdPutCRLF(pio);
#endif

	_cmdPutString ( pio, "Task: Default: min stack free: " );
	_cmdPutInt ( pio, g_nMinStackFreeDefault*sizeof(uint32_t), 0 );
	_cmdPutCRLF(pio);

	_cmdPutString ( pio, "Task: Monitor: min stack free: " );
	_cmdPutInt ( pio, g_nMinStackFreeMonitor*sizeof(uint32_t), 0 );
	_cmdPutCRLF(pio);


	//show various memory regions of interest
	//XXX maybe add more PROVIDEs in linker script to avoid these carnal constants
	extern char _sccmram asm("_sccmram");
	extern char _eccmram asm("_eccmram");
	_cmdPutString ( pio, "ccm: 0x" );
	_putHexUint32 ( pio, (uint32_t)&_sccmram );
	_cmdPutString ( pio, " - 0x1000ffff, used " );
	_cmdPutInt ( pio, &_eccmram - &_sccmram, 0 );
	_cmdPutString ( pio, " rem " );
	_cmdPutInt ( pio, (char*)0x10010000 - &_eccmram, 0 );
	_cmdPutCRLF(pio);

	extern char _sdata asm("_sdata");
	extern char _end asm("_end");
	_cmdPutString ( pio, "sram1: 0x" );
	_putHexUint32 ( pio, (uint32_t)&_sdata );
	_cmdPutString ( pio, " - 0x2001bfff, used " );
	_cmdPutInt ( pio, &_end - &_sdata, 0 );
	_cmdPutString ( pio, " rem " );
	_cmdPutInt ( pio, (char*)0x2001c000 - &_end, 0 );
	_cmdPutCRLF(pio);
	
	extern char _edata asm("_edata");
	extern char _sbss asm("_sbss");
	extern char _ebss asm("_ebss");
	_cmdPutString ( pio, "  data @ 0x" );
	_putHexUint32 ( pio, (uint32_t)&_sdata );
	_cmdPutString ( pio, "[" );
	_cmdPutInt ( pio, &_edata - &_sdata, 0 );
	_cmdPutString ( pio, "], bss @ 0x" );
	_putHexUint32 ( pio, (uint32_t)&_sbss );
	_cmdPutString ( pio, "[" );
	_cmdPutInt ( pio, &_ebss - &_sbss, 0 );
	_cmdPutString ( pio, "], rest @ 0x" );
	_putHexUint32 ( pio, (uint32_t)&_end );
	_cmdPutString ( pio, "[" );
	_cmdPutInt ( pio, (char*)0x2001c000 - &_end, 0 );
	_cmdPutString ( pio, "]\r\n" );
	
	extern uint8_t ucHeap;
	_cmdPutString ( pio, "  heap @ 0x" );
	_putHexUint32 ( pio, (uint32_t)&ucHeap );
	_cmdPutString ( pio, "[" );
	_cmdPutInt ( pio, configTOTAL_HEAP_SIZE, 0 );
	_cmdPutString ( pio, "]\r\n" );

	extern char _sram2 asm("_sram2");
	extern char _eram2 asm("_eram2");
	_cmdPutString ( pio, "sram2: 0x" );
	_putHexUint32 ( pio, (uint32_t)&_sram2 );
	_cmdPutString ( pio, " - 0x2001ffff, used ");
	_cmdPutInt ( pio, &_eram2 - &_sram2, 0 );
	_cmdPutString ( pio, " rem " );
	_cmdPutInt ( pio, (char*)0x20020000 - &_eram2, 0 );
	_cmdPutCRLF(pio);

	extern char _sbkram asm("_sbkram");
	extern char _ebkram asm("_ebkram");
	_cmdPutString ( pio, "bkram: 0x" );
	_putHexUint32 ( pio, (uint32_t)&_sbkram );
	_cmdPutString ( pio, " - 0x40024fff, used " );
	_cmdPutInt ( pio, &_ebkram - &_sbkram, 0 );
	_cmdPutString ( pio, " rem " );
	_cmdPutInt ( pio, (char*)0x40025000 - &_ebkram, 0 );
	_cmdPutCRLF(pio);

	//extern char _etext asm("_etext");
	extern char __fini_array_end asm("__fini_array_end");
	_cmdPutString ( pio, "flash: 0x08000000" );
	_cmdPutString ( pio, " - 0x08080000, used " );
	_cmdPutInt ( pio, &__fini_array_end - (char*)0x08000000, 0 );
	_cmdPutString ( pio, " rem " );
	_cmdPutInt ( pio, (char*)0x08080000 - &__fini_array_end, 0 );
	_cmdPutCRLF(pio);



	CWCMD_SendPrompt ( pio );
	return CMDPROC_SUCCESS;
}
#endif



//========================================================================
//'dump' command handler


static char _printableChar ( char ch )
{
	if ( ( ch < ' ' ) || ( ch > 0x7f ) ) ch='.';
	return ch;
}


static CmdProcRetval cmdhdlDump ( const IOStreamIF* pio, const char* pszszTokens )
{
	const char* pszStartAddress;
	const char* pszCount;
	uint32_t nStartAddr;
	uint32_t nCount;
	const uint8_t* pby;
	uint32_t nIdx;

	pszStartAddress = pszszTokens;
	if ( NULL == pszStartAddress )
	{
		_cmdPutString ( pio, "dump requires an address\r\n" );
		CWCMD_SendPrompt ( pio );
		return CMDPROC_ERROR;
	}
	pszCount = CMDPROC_nextToken ( pszStartAddress );
	if ( NULL == pszCount )
	{
		_cmdPutString ( pio, "dump requires a count\r\n" );
		CWCMD_SendPrompt ( pio );
		return CMDPROC_ERROR;
	}

	//parse address
	nStartAddr = _parseInt ( pszStartAddress );

	//parse count
	nCount = _parseInt ( pszCount );

	if ( nCount < 1 )
	{
		_cmdPutString ( pio, "too few bytes to dump.  1 - 8192.\r\n" );
		CWCMD_SendPrompt ( pio );
		return CMDPROC_ERROR;
	}
	if ( nCount > 8192 )
	{
		_cmdPutString ( pio, "too many bytes to dump.  1 - 8192.\r\n" );
		CWCMD_SendPrompt ( pio );
		return CMDPROC_ERROR;
	}

	//OK, now we do the hex dump
	_cmdPutString ( pio, "          00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\r\n" );
	_cmdPutString ( pio, "--------  -----------------------------------------------  ----------------\r\n" );
	pby = (const uint8_t*)nStartAddr;
	for ( nIdx = 0; nIdx < nCount; )
	{
		int nIter;
		int nToDo = nCount - nIdx;
		if ( nToDo > 16 )
			nToDo = 16;

		//first, do the address
		uint32_t nThisAddr = nStartAddr + nIdx;
		for ( nIter = 0; nIter < 8; ++nIter )
		{
			_cmdPutChar ( pio, _nybbleToChar ( (uint8_t) ( nThisAddr >> 28 ) ) );
			nThisAddr <<= 4;
		}
		_cmdPutString ( pio, "  " );
		
		//now do the hex part
		for ( nIter = 0; nIter < nToDo; ++nIter )
		{
			_cmdPutChar ( pio, _nybbleToChar ( pby[nIdx+nIter] >> 4 ) );
			_cmdPutChar ( pio, _nybbleToChar ( pby[nIdx+nIter] & 0x0f ) );
			_cmdPutChar ( pio, ' ' );
		}
		for ( ; nIter < 16; ++nIter )
		{
			_cmdPutString ( pio, "   " );
		}
		_cmdPutChar ( pio, ' ' );
		
		//now do the text part
		for ( nIter = 0; nIter < nToDo; ++nIter )
		{
			_cmdPutChar ( pio, _printableChar ( pby[nIdx+nIter] ) );
		}
		for ( ; nIter < 16; ++nIter )
		{
			_cmdPutChar ( pio, ' ' );
		}

		//finished!
		_cmdPutCRLF(pio);

		nIdx += nToDo;
	}

	CWCMD_SendPrompt ( pio );
	return CMDPROC_SUCCESS;
}



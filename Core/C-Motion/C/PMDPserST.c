//
//  PMDPSerST.c -- STM32 serial interface peripheral functions
//
//  Performance Motion Devices, Inc.
//

#include <stdio.h>
#include <stdlib.h>


#include "PMDtypes.h"
#include "PMDecode.h"
#include "PMDperiph.h"
#include "PMDPfunc.h"
#include "PMDsys.h"

// only need to include this if diagnostics mode is used
#include "PMDdiag.h"
#include "PMDserST.h"
#include "stm32h7xx_hal.h"

static BYTE  parList[] = { NOPARITY, ODDPARITY, EVENPARITY, MARKPARITY, SPACEPARITY };
static BYTE stopList[] = { ONESTOPBIT, TWOSTOPBITS };
static long baudList[] = { 1200, 2400, 9600, 19200, 57600, 115200, 230400, 460800 };

//extern UartHandle;
UART_HandleTypeDef UartHandleTest;

void FormatGetLastError()
{
#if(0)
	LPTSTR lpMsgBuf;
    DWORD dw = GetLastError(); 
  
    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | 
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        dw,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0, NULL );
    LocalFree(lpMsgBuf);
#endif
}

//********************************************************
static PMDresult PMDPCOM_Close(PMDPeriphHandle* hPeriph)
{
    if ( hPeriph->handle != (void *)INVALID_HANDLE_VALUE )
    {
       // CloseHandle( hPeriph->handle );
        hPeriph->handle = (void *)INVALID_HANDLE_VALUE;
    }
    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDPCOM_SetConfig(PMDPeriphHandle* hPeriph, PMDparam baud, PMDSerialParity parity, PMDSerialStopBits stopbits)
{

    PMD_PERIPHCONNECTED(hPeriph)
	UART_HandleTypeDef * TempUartHandle;

    if (baud <= PMDSerialBaud460800) // support the use of PMDSerialBaud enum instead of actual baud.
        baud = baudList[baud];

      /*##-1- Configure the UART peripheral ######################################*/
      /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
      /* UART configured as follows:
          - Word Length = 8 Bits
          - Stop Bit = One Stop bit
          - Parity = None
          - BaudRate = 115200 baud
          - Hardware flow control disabled (RTS and CTS signals) */

      TempUartHandle=hPeriph->handle;
      TempUartHandle->Init.BaudRate     = baud;
      TempUartHandle->Init.WordLength   = UART_WORDLENGTH_8B;
      TempUartHandle->Init.StopBits     = stopbits;
      TempUartHandle->Init.Parity       = parity;  //UART_PARITY_NONE;
      TempUartHandle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
      TempUartHandle->Init.Mode         = UART_MODE_TX_RX;
      TempUartHandle->Init.OverSampling = UART_OVERSAMPLING_16;
      TempUartHandle->Init.ClockPrescaler = 0;
      TempUartHandle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

      return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDPCOM_SetTimeout(PMDPeriphHandle* hPeriph, DWORD msec)
{

#if(0)
	COMMTIMEOUTS timeouts;

    PMD_PERIPHCONNECTED(hPeriph)

    // A value of zero for ReadIntervalTimeout indicates that interval timeouts are not used.
    timeouts.ReadIntervalTimeout         = 0;
    timeouts.ReadTotalTimeoutMultiplier  = 10;
    timeouts.ReadTotalTimeoutConstant    = msec;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant   = 0;
    if (msec == 0)
    {
		// A value of MAXDWORD, combined with zero values for both the ReadTotalTimeoutConstant and ReadTotalTimeoutMultiplier members, specifies that the read operation 
		// is to return immediately with the bytes that have already been received, even if no bytes have been received.
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
    }

    // If SetCommTimeouts fails, the return value is zero. To get extended error information, call GetLastError. 
    if (SetCommTimeouts( hPeriph->handle, &timeouts ))
        return PMD_ERR_OK;
#endif
    return PMD_ERR_CommandError;
}


//********************************************************
// The timeout parameter is not used when sending serial data because handshaking is not used.
static PMDresult PMDPCOM_Send(PMDPeriphHandle* hPeriph, const void* data, PMDparam nCount, PMDparam timeoutms)
{

    PMD_PERIPHCONNECTED(hPeriph)


    if (nCount > 0)
    {
    	if(HAL_UART_Transmit(hPeriph->handle, (uint8_t*)data, nCount, 5000)!= HAL_OK)
    	{
    		//        FormatGetLastError();
            return PMD_ERR_PortWrite;
    	}

    }
    return PMD_ERR_OK;
}

//********************************************************
static PMDresult PMDPCOM_Receive(PMDPeriphHandle* hPeriph, void *data, PMDparam nCount, PMDparam *pnreceived, PMDparam timeoutms)
{
    *pnreceived = 0;
    PMD_PERIPHCONNECTED(hPeriph)
    HAL_StatusTypeDef result;

    if (nCount > 0)
    {
   //     PMDPCOM_SetTimeout(hPeriph, timeoutms);
    	result=HAL_UART_Receive(hPeriph->handle, (uint8_t *)data, nCount, 50);
        if(result==HAL_ERROR)  return PMD_ERR_ReadError;
        if(result==HAL_TIMEOUT)  return PMD_ERR_Timeout;
    }

    *pnreceived=nCount; // true if HAL_UART_Receive returns 0;
    return PMD_ERR_OK;
}

//********************************************************
PMDresult PMDPCOM_FlushRecv(PMDPeriphHandle* hPeriph)
{
    PMD_PERIPHCONNECTED(hPeriph)
    uint8_t flush[20];
    HAL_StatusTypeDef result;

    result=HAL_UART_Receive(hPeriph->handle, (uint8_t *)flush, 20, 5);  //clears over flow flag
    result=HAL_UART_Receive(hPeriph->handle, (uint8_t *)flush, 20, 5);  //2nd time

	if(result) return (PMDresult) result;
    else return PMD_ERR_OK;
}

//********************************************************
static void PMDPCOM_Init(PMDPeriphHandle* hPeriph)
{
    // set the interface type 
    hPeriph->type = InterfaceSerial;

    hPeriph->transport.Close    = PMDPCOM_Close;
    hPeriph->transport.Send     = PMDPCOM_Send;
    hPeriph->transport.Receive  = PMDPCOM_Receive;
    hPeriph->transport.Read     = NULL;
    hPeriph->transport.Write    = NULL;
}

//*************************************************************************************
void ReportError()
{
#if(0)
char szErrorMsg[256];
DWORD ErrorCode;

    ErrorCode = GetLastError();
    if (ErrorCode != ERROR_SUCCESS)
    {
        FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM, 
                  NULL, 
                  ErrorCode, 
                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), 
                  szErrorMsg, 
                  sizeof(szErrorMsg), 
                  NULL);
                  puts(szErrorMsg);
    }

#endif

}

// Open Serial comm port
PMDresult PMDPCOM_Open(PMDPeriphHandle* hPeriph, PMDparam portnum, PMDparam baud, PMDSerialParity parity, PMDSerialStopBits stopbits)
{

	UART_HandleTypeDef* UartHandleptr;
	UartHandleptr=(UART_HandleTypeDef *) malloc(sizeof(UART_HandleTypeDef));

	PMDPCOM_Init(hPeriph);
    hPeriph->handle=UartHandleptr;
    UartHandleptr->Instance = (USART_TypeDef *) portnum;

    if( hPeriph->handle == (void *) INVALID_HANDLE_VALUE )
    {

        return PMD_ERR_OpeningPort;
    }
    if (PMD_ERR_OK != PMDPCOM_SetConfig(hPeriph, baud, parity, stopbits))
    {
        PMDPCOM_Close(hPeriph);
        return PMD_ERR_ConfiguringPort;
    }



    if(HAL_UART_DeInit(UartHandleptr) != HAL_OK)
    {
        	return PMD_ERR_ConfiguringPort;
    }
    if(HAL_UART_Init(UartHandleptr) != HAL_OK)
    {
          return PMD_ERR_ConfiguringPort;
    }


    return PMD_ERR_OK;
}

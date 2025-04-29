//
//  PMDPspiST.c -- SPI interface command/data transfer functions for STM32.
//  Performance Motion Devices, Inc.
//
//  The SPI port is a variable however the assignment of PD14 for Chipselect and PB8 for HostStatus has been hardcoded.
//  The STM32 Nucleo H743ZI2 connections are:
//  CN7-2 HOSTSTATUS
//  CN7-8 Ground
//  CN7-10 SPICLK
//  CN7-12 MISO
//  CN7-14 MOSI
//  CN7-16 Select (SPIEnable)
//
//  TLK 4/25/25
//

#include "PMDRPtypes.h"
#include "PMDperiph.h"
#include "PMDecode.h"
#include "PMDsys.h"
#include "PMDPfunc.h"


#define BUFFERSIZE 20


#include <stdlib.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
//#define INVALID_HANDLE_VALUE -1;

typedef struct tagPMDSPI_IOData {

    int DeviceHandle;
    SPI_HandleTypeDef SpiHandle;
    
    PMDuint16 NumBitsPerSample;
    PMDint32 ClockPhase;
    PMDint32 ClockPhaseRead;
    PMDint32 ClockPolarity;

    PMDuint16 ClockRate;
    PMDuint32 ChipSelect;
    BOOL bUseScript;
    BOOL bByteSwap;
    BOOL bUseHostStatus;
    long Timeout;
    PMDuint8 bVerbose;
    PMDuint8 ReadData[TOTAL_PACKET_LENGTH];
    char     ResourceName[256];
} PMDSPI_IOData;


PMDresult result;


PMDresult PMDPSPI_Close(PMDPeriphHandle* hPeriph)
{
    PMDSPI_IOData* SPItransport_data = (PMDSPI_IOData*)hPeriph->transport_data;
    if (SPItransport_data != NULL)
    {
        hPeriph->handle = (void*) INVALID_HANDLE_VALUE;
        free(SPItransport_data);
        hPeriph->transport_data = NULL;
    }
    return PMD_ERR_OK;
}



// HostSPIStatus is connected to PB8
int GetHostSPIStatus(PMDPeriphHandle* hPeriph)
{

	PMDSPI_IOData* SpiIO = (PMDSPI_IOData*)hPeriph->transport_data;

	switch(SpiIO->ChipSelect)
	{
	case 1:
		return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	case 2:
		return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
	default:
		return -1;
	}
}

// HostSPIEnable is connected to D14
void AssertHostSPIEnable(PMDSPI_IOData* SpiIO)
{
	switch(SpiIO->ChipSelect)
	{
	case 1:
			return HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,RESET);
	case 2:
			return HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,RESET);
	default:
			return;
	}
}

void DeAssertHostSPIEnable(PMDSPI_IOData* SpiIO)
{
	switch(SpiIO->ChipSelect)
		{
		case 1:
				return HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,SET);
		case 2:
				return HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,SET);
		default:
				return;
		}
}

// Data provided in pWriteData and retrieved using pReadData is organized in big endian format.
PMDresult PMDSPI_WriteWords(SPI_HandleTypeDef* htemp, int chipselect, PMDuint16 *WriteData, int nwords, PMDuint16 *ReadData)
{

    result=HAL_SPI_TransmitReceive(htemp, (uint8_t*)(WriteData), (uint8_t *)(ReadData), nwords, 5000);
    if(result) return PMD_ERR_CommunicationsError;

    return PMD_ERR_OK;
}


// for getting HostSPIStatus
static PMDresult PMDPSPI_Read(PMDPeriphHandle* hPeriph, void* pData, PMDparam offset, PMDparam length)
{
    PMDuint16 DIOstate;
    PMDint32 StatusCode;
    PMDint32 ReadData;
    ReadData=GetHostSPIStatus(hPeriph);
    StatusCode=0;
    DIOstate = (PMDuint16)ReadData;
    *(PMDuint16*)pData = DIOstate;
    
    return (StatusCode == 0) ? PMD_ERR_OK : PMD_ERR_PortRead;
}



/// The protocol described below only applies to MC5x113 and MC7x113s
/*  
Sending a MC58113 command
word    Host                        MC58113
------------------------------------------------------
        Assert ~HostSPIEnable   
1       8b axis 8b opcode           0
2       8b rsv  8b checksum         0
3       16b argument 1 (optional)   0
4       16b argument 2 (optional)   0
5       16b argument 3 (optional)   0
        De-assert ~HostSPIEnable    

        wait for HostSPIReady signal

Receiving a response
word    Host                        MC58113
------------------------------------------------------
        Assert ~HostSPIEnable   
1       0                           8b checksum  8b command status
2       0                           16b response 1 (optional)
3       0                           16b response 2 (optional)
4       0                           16b response 3 (optional)
        De-assert ~HostSPIEnable    
*/




//********************************************************
PMDresult PMDPSPI_SendReceive(PMDPeriphHandle* hPeriph, PMDuint8 *WriteData, PMDparam nCount, PMDuint8 *ReadData)
{
    PMDSPI_IOData* SPItransport_data = (PMDSPI_IOData*)hPeriph->transport_data;
    PMDint32 StatusCode;
    PMDuint32 WriteSize = nCount;
    PMDuint32 ReadSize;
    PMDuint8 outbuf[TOTAL_PACKET_LENGTH];
    PMDuint8 inbuf[TOTAL_PACKET_LENGTH];
    PMDuint32 i;

    PMD_PERIPHCONNECTED(hPeriph)

    if (SPItransport_data->NumBitsPerSample == 16 && WriteSize % 2)
        WriteSize++;
    if (WriteSize > MAX_PACKET_DATA_LENGTH)
        return PMD_ERR_CommandError;


    if (SPItransport_data->bByteSwap)
    {
        for (i = 0; i < nCount; i+=2)
        {
            outbuf[i] = (PMDuint8)(WriteData[i+1]);
            outbuf[i + 1] = (PMDuint8) (WriteData[i]);
        }
    }
    else
        memcpy(outbuf, WriteData, nCount);


    AssertHostSPIEnable(SPItransport_data);

    StatusCode=PMDSPI_WriteWords(hPeriph->handle, SPItransport_data->ChipSelect, (PMDuint16 *) outbuf, nCount/2, (PMDuint16 *)inbuf);

    DeAssertHostSPIEnable(SPItransport_data);

    ReadSize=WriteSize;

    if (0==StatusCode && ReadData)
    {
        if (SPItransport_data->NumBitsPerSample == 16)
        {
            assert(ReadSize % 2 == 0);
        }
        if (SPItransport_data->bByteSwap)
        {
            for (i = 0; i < ReadSize; i+=2)
            {
                ReadData[i] = (PMDuint8)(inbuf[i+1]);
                ReadData[i + 1] = (PMDuint8) (inbuf[i]);
            }
        }
        else 
            memcpy(ReadData, inbuf, ReadSize);

    }
    return (StatusCode == 0) ? PMD_ERR_OK : PMD_ERR_PortWrite;
}

//********************************************************
PMDresult PMDPSPI_Send(PMDPeriphHandle* hPeriph, const void *data, PMDparam nCount, PMDparam timeoutms)
{
    PMDSPI_IOData* SPItransport_data = (PMDSPI_IOData*)hPeriph->transport_data;
    return PMDPSPI_SendReceive(hPeriph, (PMDuint8*)data, nCount, SPItransport_data->ReadData);
}

//********************************************************
PMDresult PMDPSPI_Receive(PMDPeriphHandle* hPeriph, void *data, PMDparam nExpected, PMDparam *pnReceived, PMDparam timeoutms)
{
    PMDSPI_IOData* SPItransport_data = (PMDSPI_IOData*)hPeriph->transport_data;
    PMD_PERIPHCONNECTED(hPeriph)

    if (nExpected >= TOTAL_PACKET_LENGTH)
        return PMD_ERR_InvalidParameter;

     memcpy(data, SPItransport_data->ReadData, nExpected);
    *pnReceived = nExpected;

    return PMD_ERR_OK;
}

long PMDPSPI_SetTimeout(void *transport_data, long msec)
{
     long PreviousTimeout;

     PMDSPI_IOData* pSPItransport_data = (PMDSPI_IOData *)transport_data;
     PreviousTimeout = pSPItransport_data->Timeout;
     pSPItransport_data->Timeout = msec;
     return PreviousTimeout;
}

// Call into NI library to find and initialize the external device.
PMDresult PMDPSPI_InitPort(PMDPeriphHandle* hPeriph, PMDuint8 device, PMDuint8 chipselect, PMDuint8 mode, PMDuint8 datasize, PMDparam BitRateHz)
    
{
	PMDSPI_IOData* transport_data = (PMDSPI_IOData*) hPeriph->transport_data;
	int baud;
	SPI_TypeDef* port;

	//baud selection for example only.  Many system factors affect true SPI Clck rate
	switch (BitRateHz)
    {
         case 24000000:
        	 baud=SPI_BAUDRATEPRESCALER_2;
        	 break;
         case 12000000:
             baud=SPI_BAUDRATEPRESCALER_4;
             break;
         case 6000000:
             baud=SPI_BAUDRATEPRESCALER_8;
             break;
         case 3000000:
              baud=SPI_BAUDRATEPRESCALER_16;
              break;
         case 1500000:
              baud=SPI_BAUDRATEPRESCALER_32;
              break;
         case 750000:
              baud=SPI_BAUDRATEPRESCALER_64;
              break;
         case 375000:
              baud=SPI_BAUDRATEPRESCALER_128;
              break;
         default:
        	  baud=SPI_BAUDRATEPRESCALER_256;
    }

	switch (device)
	{
	     case 1:
	       	 port= SPI1;
	       	 break;
	     case 2:
	         port= SPI2;
	         break;
	     default:
	    	 port= SPI1;
	}


	SPI_HandleTypeDef* SpiHandle;
	SpiHandle=hPeriph->handle;

	transport_data->Timeout = 500;
	transport_data->NumBitsPerSample = datasize;
	transport_data->ClockPhase = !((mode >> 0) & 1); // SPImode is same as Magellan which is a non-standard TI SPI mode with CLKPhase inverted.
	transport_data->ClockPhaseRead = transport_data->ClockPhase;
	transport_data->ClockPolarity = (mode >> 1) & 1; //kNi845xSpiClockPolarityIdleLow;
	transport_data->ClockRate = (PMDuint16) BitRateHz;
	transport_data->ChipSelect = chipselect;
	transport_data->ResourceName[0] = 0;
	transport_data->bVerbose = 0;
	transport_data->bByteSwap = (datasize == 16); //A datasize value of 16 tells PMDspi.c to byte swap which is required with the NI SPI adapter.

	SpiHandle->Instance               = port;
    SpiHandle->Init.Mode              = SPI_MODE_MASTER;
    SpiHandle->Init.BaudRatePrescaler = baud;
    SpiHandle->Init.Direction         = SPI_DIRECTION_1LINE;
    SpiHandle->Init.CLKPhase          = SPI_PHASE_2EDGE;
    SpiHandle->Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle->Init.DataSize          = SPI_DATASIZE_16BIT;
    SpiHandle->Init.NSS				= SPI_NSS_SOFT;
    SpiHandle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle->Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle->Init.CRCPolynomial     = 0;
    SpiHandle->Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
    SpiHandle->Init.NSS               = SPI_NSS_SOFT;
    SpiHandle->Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
    SpiHandle->Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
    SpiHandle->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommended setting to avoid glitches */
    SpiHandle->Init.Mode = SPI_MODE_MASTER;
    SpiHandle->Init.FifoThreshold = SPI_FIFO_THRESHOLD_08DATA;
    SpiHandle->Init.TxCRCInitializationPattern=SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    SpiHandle->Init.RxCRCInitializationPattern=SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    SpiHandle->Init.MasterSSIdleness= SPI_MASTER_SS_IDLENESS_00CYCLE;
    SpiHandle->Init.MasterInterDataIdleness=SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    SpiHandle->Init.MasterReceiverAutoSusp=SPI_MASTER_RX_AUTOSUSP_DISABLE;
    SpiHandle->Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    SpiHandle->Init.IOSwap = SPI_IO_SWAP_DISABLE;

    PMDTaskWait(1);
    if(HAL_SPI_Init(SpiHandle) != HAL_OK)
    {
         /* Initialization Error */
         return PMD_ERR_OpeningPort;
    }

    DeAssertHostSPIEnable(transport_data);

    return PMD_ERR_OK;
}


//********************************************************
static void PMDPSPI_Init(PMDPeriphHandle* hPeriph)
{
    hPeriph->transport.Close    = PMDPSPI_Close;
    hPeriph->transport.Send     = PMDPSPI_Send;
    hPeriph->transport.Receive  = PMDPSPI_Receive;
    hPeriph->transport.Read     = PMDPSPI_Read; // used for reading digital input pins to get HostSPIStatus state
    hPeriph->transport.Write    = NULL;
    hPeriph->transport.ReceiveEvent = NULL;
}

//**************************************************************
PMDresult PMDPSPI_Open (PMDPeriphHandle* hPeriph, PMDuint8 port, PMDuint8 chipselect, PMDuint8 mode, PMDuint8 datasize, PMDparam bitrate)
{
    PMDresult result = PMD_NOERROR;
    PMDSPI_IOData* SPItransport_data;
    PMDparam BitRateHz;

    SPI_HandleTypeDef* SpiHandleptr;
    SpiHandleptr=(SPI_HandleTypeDef *) malloc(sizeof(SPI_HandleTypeDef));
    memset(SpiHandleptr, 0, sizeof(SPI_HandleTypeDef));
    PMDPSPI_Init(hPeriph);

    SPItransport_data = (PMDSPI_IOData*) malloc( sizeof( PMDSPI_IOData ) );
    memset(SPItransport_data, 0, sizeof(PMDSPI_IOData));


    // set the interface type 
    hPeriph->type = InterfaceSPI;
    hPeriph->transport_data = SPItransport_data;
    hPeriph->address = 0;
    hPeriph->handle = SpiHandleptr;


    BitRateHz = bitrate;

    result = PMDPSPI_InitPort(hPeriph, port, chipselect, mode, datasize, BitRateHz);

    if (result == PMD_NOERROR)
    {
 //       hPeriph->handle = PMD_CONNECTED_HANDLE; // set the periph handle as connected
        hPeriph->param = 1; // HostSPIStatus digital input number on the NI (1=DIO0)
    }


    return result;
}


//  PMDspi.c -- Motion Processor SPI protocol functions 
//
//  Performance Motion Devices, Inc.
//

#include "PMDperiph.h"
#include "PMDecode.h"
#include "PMDsys.h"
#include "PMDPfunc.h"

#define Printf PMDprintf
PMDMutexDefine(xMutexSPI)


/*  
Sending a command
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

#define MP_TIMEOUT         10


static PMDresult PMDSPI_WaitUntilReady(PMDPeriphHandle* hPeriph, PMDparam timeoutms, int bIsActive)
{
    unsigned long EndTime = PMDDeviceGetTickCount() + timeoutms;
    unsigned long CurrentTime;
    PMDuint16 DIOstate;
    int bActive;

    // If no PMDPIO_DIOn specified as HostSPIStatus then just return
    if ((hPeriph->param) == 0)
        return PMD_ERR_OK;
    
    do {
        PMDPeriphRead(hPeriph, &DIOstate, 0, 0);
        // HostSPIStatus is active low 
        // hPeriph->param is the PMDPIO_DIOn to read
        bActive = !(DIOstate & (hPeriph->param & 0xFF));
        if (bIsActive == bActive)
            return PMD_ERR_OK;
        CurrentTime = PMDDeviceGetTickCount();
    } while (CurrentTime < EndTime);


    return PMD_ERR_Timeout;
}

PMDresult PMDSPI_Send(void* transport_data, PMDuint8 xCt, PMDuint16* xDat, PMDuint8 rCt, PMDuint16* rDat)
{
    PMDDeviceHandle* hDevice = (PMDDeviceHandle*)transport_data;
    PMDPeriphHandle* hPeriph = hDevice->hPeriph;

    union buff 
    {
        WORD  w[8];
        BYTE  b[16];
    } txbuff, rxbuff;

    PMDresult result = PMD_ERR_OK;
    int  ProcessorError = 0;
    char  nbytes;
    char  nwords;
    int  i;
    int bByteSwap = ((hPeriph->param >> 8) == 8); // We need to byteswap the data if sending in 8-bit SPI mode.
    WORD sum;
    WORD carry;
    PMDuint32 WriteSize;
    PMDuint8* WriteData = txbuff.b;
    const PMDuint8  Nop[8] = {0,0,0,0,0,0,0,0};
    PMDuint32 ReadSize;
    PMDuint32 nRead;
    PMDuint8* ReadData = rxbuff.b;
    static const int bHostSPIStatusActive = 1; 

    // The 16-bit words sent to the chip is in big endian format.
    txbuff.w[0] = xDat[0];  // axis and opcode bytes
    txbuff.w[1] = 0;        // reserved and checksum bytes
    txbuff.w[2] = xDat[1];
    txbuff.w[3] = xDat[2];
    txbuff.w[4] = xDat[3];

    nwords = xCt + 1;
    nbytes = nwords * 2;

    // calculate checksum
    sum = 0xAA; // seed
    for( i=0; i<nbytes; i++ ) 
        sum += txbuff.b[i];

    carry = sum >> 8;
    sum = sum & 0xFF;
    sum += carry;
    carry = sum >> 8;
    sum = sum & 0xFF;
    sum += carry;
    sum = ~sum;
    txbuff.b[2] = (PMDuint8)sum; // checksum byte

    // byte swap data if SPI datasize is 8 bit
    if (bByteSwap)
    {
        for( i=0; i<nbytes; i+=2 )
        {
            char temp  = txbuff.b[i];
            txbuff.b[i] = txbuff.b[i+1];
            txbuff.b[i+1] = temp;
        }
    }

    if (!PMDMutexLock(xMutexSPI))
      return PMD_ERR_MutexTimeout;

    // Wait for the HostSPIStatus signal to go inactive (high) from the previous command.
    result = PMDSPI_WaitUntilReady(hPeriph, MP_TIMEOUT, !bHostSPIStatusActive);
      
    // If the HostSPIStatus signal stays active then we are out of sync.
    if (result == PMD_ERR_Timeout)
    {
        // do a read of any number of words to resync. 
        WriteSize = 2;
        Printf("SPI resync\n");
        result = PMDPeriphSend(hPeriph, Nop, WriteSize, MP_TIMEOUT);
    }

    // Send the command
    WriteSize = nbytes;
    result = PMDPeriphSend(hPeriph, WriteData, WriteSize, MP_TIMEOUT);

    // Wait for the HostSPIStatus signal to go active (low) to indicate the response is ready to be retrieved.
    result = PMDSPI_WaitUntilReady(hPeriph, MP_TIMEOUT, bHostSPIStatusActive);

    ReadSize = (rCt + 1) * 2; // +1 to include checksum word
    if (result == PMD_ERR_OK)
    {
        result = PMDPeriphSend(hPeriph, Nop, ReadSize, MP_TIMEOUT);
        result = PMDPeriphReceive(hPeriph, ReadData, &nRead, ReadSize, MP_TIMEOUT);
    }
    
    if (result == PMD_ERR_OK)
    {
        // byte swap return data if SPI datasize is 8 bit
        if (bByteSwap)
        {
            for( i=0; i<(int)ReadSize; i+=2 )
            {
                BYTE temp = rxbuff.b[i];
                rxbuff.b[i] = rxbuff.b[i+1];
                rxbuff.b[i+1] = temp;
            }
        }

        // The first byte in response is the motion processor command status 
        // A positive command status is # result words that are available 
        // A negative command status is an error code. 
        ProcessorError = 0;
        if ((PMDint8)(rxbuff.b[0]) < 0)
        {
            ProcessorError = -(PMDint8)(rxbuff.b[0]);
        }
        // invalid packet if # of result words is > 3
        if (rxbuff.b[0] > 3)
            result =  PMD_ERR_InvalidDataSize;

        if (result == PMD_ERR_OK)
        {
            sum = 0xAA;
            for( i=0; i<(int)ReadSize; i++ )
                sum += rxbuff.b[i];
            carry = sum >> 8;
            sum = sum & 0xFF;
            sum += carry;
            carry = sum >> 8;
            sum = sum & 0xFF;
            sum += carry;

            if( sum != 0xFF)
            {
              Printf("\r\nChecksum error in response data: ");
                for(i=0; i<(int)ReadSize; i++ )
                    Printf("  %04X", rxbuff.w[i] );
                Printf("Sum: %02X\r\n", sum );

                result = PMD_ERR_Checksum;
            }

            // copy return data skipping checksum word
            for( i=0; i<rCt; i++ )
            {
                rDat[i] = rxbuff.w[i+1];
            }
        }
    }
    PMDMutexUnlock(xMutexSPI);
    
    // report error if one occured
    if( ProcessorError )
        return (PMDErrorCode)ProcessorError;

    return result;
}

//********************************************************
PMDresult PMDSPI_Init()
{
    // only one SPI bus so only one mutex.
    if (xMutexSPI == NULL)
      xMutexSPI = PMDMutexCreate();
    if (xMutexSPI == NULL)
      return PMD_ERR_Memory;

    return PMD_NOERROR;
}

//********************************************************
PMDresult PMDSPI_DeInit(PMDDeviceHandle* hDevice)
{
    PMDMutexDelete(xMutexSPI);

    return PMD_NOERROR;
}


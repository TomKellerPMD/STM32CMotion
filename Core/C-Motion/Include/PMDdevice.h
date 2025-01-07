#ifndef _DEF_INC_PMDdevice
#define _DEF_INC_PMDdevice

//
//  PMDdevice.h -- Definitions for Device functions
//
//  Performance Motion Devices, Inc.
//

#include "PMDtypes.h"
#include "PMDRPtypes.h"
#include "PMDecode.h"
#include "PMDintf.h"
#include "PMDsys.h"

#include <string.h>


#pragma pack(4)

// Event structures
typedef struct tagPMDAxisEvent {
    PMDAxis             axis;
    PMDuint16           eventstatus;
} PMDEvent;

// Peripheral structures
struct tagPMDPeriphHandle;
typedef struct tagPMDPeriphTransport {
    PMDresult (*Close)       (struct tagPMDPeriphHandle *hPeriph);
    PMDresult (*Send)        (struct tagPMDPeriphHandle *hPeriph, const void *pData, PMDparam length, PMDparam timeout);
    PMDresult (*Receive)     (struct tagPMDPeriphHandle *hPeriph, void *pData, PMDparam length, PMDparam *pnreceived, PMDparam timeout);
    PMDresult (*Write)       (struct tagPMDPeriphHandle *hPeriph, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*Read)        (struct tagPMDPeriphHandle *hPeriph, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*ReceiveEvent)(struct tagPMDPeriphHandle *hPeriph, void *pData, PMDparam length, PMDparam *pnreceived, PMDparam timeout);
} PMDPeriphTransport;

typedef struct tagPMDPeriphHandle {
    void *              handle;         // device dependent handle
    void *              transport_data; // peripheral dependent data pointer
    PMDparam            address;        // peripheral dependent address or the PRP address if communicating to a remote peripheral over PRP
    PMDInterfaceType    type;
    PMDPeriphTransport  transport;
    PMDparam            param;          // peripheral dependent parameter which may or may not be used
} PMDPeriphHandle;


// Device structures
struct tagPMDDeviceHandle;
struct tagPMDMemoryHandle;
struct tagPMDMailboxHandle;

typedef struct tagPMDDeviceTransport {
    PMDresult (*SendCommand) (void*, PMDuint8 xCt, PMDuint16 *xDat, PMDuint8 rCt, PMDuint16 *rDat);
    // the following Memory access functions are for non-CME Prodigy cards with DPRAM
    PMDresult (*WriteMemory) (struct tagPMDMemoryHandle *hMemory, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*ReadMemory)  (struct tagPMDMemoryHandle *hMemory, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*WaitForEvent)(struct tagPMDDeviceHandle *hDevice, PMDEvent *pEvent, PMDparam timeout);
    PMDresult (*Reset)       (void*);
    PMDresult (*Close)       (struct tagPMDDeviceHandle *hDevice);
} PMDDeviceTransport;

typedef struct tagPMDDeviceHandle {
    void *              transport_data;
    PMDPeriphHandle *   hPeriph;        // handle to the peripheral that the device is connected to
    PMDDeviceType       type;
    PMDDeviceTransport  transport;
    PMDparam            address;        
} PMDDeviceHandle;


// Memory structures
typedef struct tagPMDMemoryTransport {
    PMDresult (*Write)       (struct tagPMDMemoryHandle *hMemory, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*Read)        (struct tagPMDMemoryHandle *hMemory, void *pData, PMDparam offset, PMDparam length);
    PMDresult (*Close)       (struct tagPMDMemoryHandle *hMemory);
    PMDresult (*Erase)       (struct tagPMDMemoryHandle *hMemory);
} PMDMemoryTransport;

typedef struct tagPMDMemoryHandle {
    PMDDeviceHandle *   hDevice;        // handle of the device containing the memory
    PMDDataSize         datasize;
    PMDparam            address;
    PMDparam            length;         // total size of memory
    PMDMemoryTransport  transport;
} PMDMemoryHandle;

// Mailbox structures
typedef struct tagPMDMailboxTransport {
    PMDresult (*Close)       (struct tagPMDMailboxHandle *hMailbox);
    PMDresult (*Send)        (struct tagPMDMailboxHandle *hMailbox, const void *pItem, PMDparam timeout);
    PMDresult (*Receive)     (struct tagPMDMailboxHandle *hMailbox, void *pItem, PMDparam timeout);
    PMDresult (*Peek)        (struct tagPMDMailboxHandle *hMailbox, void *pItem, PMDparam timeout);
    void *    reserved;
} PMDMailboxTransport;

typedef struct tagPMDMailboxHandle {
    PMDDeviceHandle *   hDevice;        // handle of the device containing the mailbox
    void *              transport_data; // device dependent data pointer
    PMDparam            address;        // PRP address
    PMDparam            mailboxID;      // mailbox identifier
    PMDparam            itemsize;
    PMDparam            reserved;
    PMDMailboxTransport transport;
} PMDMailboxHandle;


// Axis structures
typedef struct tagPMDCPTransport {
    PMDresult (*SendCommand)(void*, PMDuint8 xCt, PMDuint16 *xDat, PMDuint8 rCt, PMDuint16 *rDat);
} PMDCPTransport;

typedef struct tagPMDAxisHandle {
    PMDAxis             axis;
    void *              transport_data;
    PMDCPTransport      transport;
} PMDAxisHandle;

typedef PMDAxisHandle *PMDAxisInterface;
typedef void *PMDHandle;
typedef PMDHandle PMDEventHandle;


#pragma pack()

// Legacy function names
#define PMDMPDeviceOpen                 PMDPeriphOpenDeviceMP
#define PMDRPDeviceOpen                 PMDPeriphOpenDevicePRP
#define PMDTaskStart                    PMDCMETaskStart               
#define PMDTaskStop                     PMDCMETaskStop                
#define PMDTaskGetInfo                  PMDCMETaskGetInfo             
#define PMDTaskGetState                 PMDCMETaskGetState
#define PMDDeviceStoreUserCode          PMDCMEStoreUserCode     
#define PMDGetUserCodeFileChecksum      PMDCMEGetUserCodeChecksum     
#define PMDGetUserCodeFileDate          PMDCMEGetUserCodeDate     
#define PMDGetUserCodeFileName          PMDCMEGetUserCodeName     
#define PMDGetUserCodeFileVersion       PMDCMEGetUserCodeVersion
#define PMDDeviceSetConsole             PMDCMESetConsole
#define PMDMemoryOpen                   PMDDeviceOpenMemory


#if defined(__cplusplus)
extern "C" {
#endif

PMDCFunc PMDPeriphOpenDeviceMP          (PMDDeviceHandle *hDevice, PMDPeriphHandle *hPeriph);
PMDCFunc PMDPeriphOpenDevicePRP         (PMDDeviceHandle *hDevice, PMDPeriphHandle *hPeriph);
PMDCFunc PMDDeviceClose	                (PMDDeviceHandle *hDevice);
PMDCFunc PMDDeviceReset	                (PMDDeviceHandle *hDevice);
PMDCFunc PMDDeviceGetVersion            (PMDDeviceHandle *hDevice, PMDuint32 *major, PMDuint32 *minor);
PMDCFunc PMDDeviceGetInfo               (PMDDeviceHandle *hDevice, PMDDeviceInfo infoID, PMDuint16 option, PMDuint32 *value);
PMDCFunc PMDDeviceGetDefault            (PMDDeviceHandle *hDevice, PMDDefaults defaultcode, void *value, PMDDataSize valueSize);
PMDCFunc PMDDeviceSetDefault            (PMDDeviceHandle *hDevice, PMDDefaults defaultcode, void *value, PMDDataSize valueSize);
PMDCFunc PMDCMETaskGetState             (PMDDeviceHandle *hDevice, PMDTaskState *state);
PMDCFunc PMDCMETaskStart                (PMDDeviceHandle *hDevice, PMDparam taskparam);
PMDCFunc PMDCMETaskStop                 (PMDDeviceHandle *hDevice);
PMDCFunc PMDCMETaskGetInfo              (PMDDeviceHandle *hDevice, PMDuint8 tasknumber, PMDTaskInfo infoID, PMDint32 *value);
PMDCFunc PMDCMEStoreUserCode            (PMDDeviceHandle *hDevice, PMDuint8 *pdata, int length);
PMDCFunc PMDCMEGetUserCodeVersion       (PMDDeviceHandle *hDevice, PMDuint32 *version);
PMDCFunc PMDCMEGetUserCodeChecksum      (PMDDeviceHandle *hDevice, PMDuint32 *checksum);
PMDCFunc PMDCMEGetUserCodeName          (PMDDeviceHandle *hDevice, char* name);
PMDCFunc PMDCMEGetUserCodeDate          (PMDDeviceHandle *hDevice, char* date);
PMDCFunc PMDCMESetConsole               (PMDDeviceHandle *hDevice, PMDPeriphHandle *hPeriph);
PMDCFunc PMDDeviceNoOperation           (PMDDeviceHandle *hDevice);
PMDCFunc PMDDeviceGetResetCause         (PMDDeviceHandle *hDevice, PMDuint16 *resetcause, PMDuint16 resetmask);
PMDCFunc PMDDeviceGetFaultCode          (PMDDeviceHandle *hDevice, PMDFaultCode faultID, PMDuint32 *value);
PMDCFunc PMDDeviceGetSystemTime         (PMDDeviceHandle *hDevice, SYSTEMTIME *time);
PMDCFunc PMDDeviceSetSystemTime         (PMDDeviceHandle *hDevice, const SYSTEMTIME *time);
PMDCFunc PMDDeviceSetNodeID             (PMDDeviceHandle *hDevice, PMDuint8 nodeID, PMDuint8 DOsignal, PMDuint8 DIsignal, PMDuint8 DIsense);
                                        
PMDCFunc PMDWaitForEvent                (PMDDeviceHandle *hDevice, PMDEvent *eventdata, PMDparam timeout);
PMDCFunc PMDDeviceGetEvent              (PMDDeviceHandle *hDevice, PMDEvent *eventdata);
                                        
PMDCFunc PMDAxisOpen                    (PMDAxisHandle *hAxis, PMDDeviceHandle *hDevice, PMDAxis axis_number);
PMDCFunc PMDAtlasAxisOpen               (PMDAxisInterface hSourceAxis, PMDAxisInterface hAtlasAxis);
PMDCFunc PMDAxisClose                   (PMDAxisHandle *hAxis);
                                        
PMDCFunc PMDDeviceOpenMemory            (PMDMemoryHandle *hMemory, PMDDeviceHandle *hDevice, PMDDataSize datasize, PMDMemoryAddress memorytype);
                                        // offset and length are in units of datasize that the handle is opened with.
PMDCFunc PMDMemoryRead                  (PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length);
PMDCFunc PMDMemoryWrite                 (PMDMemoryHandle *hMemory, void *data, PMDuint32 offset, PMDuint32 length);
PMDCFunc PMDMemoryClose                 (PMDMemoryHandle *hMemory);
PMDCFunc PMDMemoryErase                 (PMDMemoryHandle *hMemory);
                                        
PMDCFunc PMDMailboxOpen                 (PMDMailboxHandle *hMailbox, PMDDeviceHandle *hDevice, PMDparam mailboxID, PMDparam depth, PMDparam itemsize);
PMDCFunc PMDMailboxSend                 (PMDMailboxHandle *hMailbox, const void *pItem, PMDparam timeout);
PMDCFunc PMDMailboxReceive              (PMDMailboxHandle *hMailbox, void *pItem, PMDparam timeout);
PMDCFunc PMDMailboxPeek                 (PMDMailboxHandle *hMailbox, void *pItem, PMDparam timeout);
PMDCFunc PMDMailboxClose                (PMDMailboxHandle *hMailbox);
                                        
PMDCFunc PMDEventOpen                   (PMDEventHandle *hEvent, PMDEventType type, PMDparam var1, PMDparam var2, PMDparam var3);
PMDCFunc PMDEventClose                  (PMDEventHandle *hEvent);
PMDCFunc PMDEventOpenMotion             (PMDEventHandle *hEvent);
PMDCFunc PMDEventOpenDI                 (PMDEventHandle *hEvent, PMDEventNumber number, PMDEventTrigger trigger, PMDEventSignal signal);
PMDCFunc PMDEventOpenTimer              (PMDEventHandle *hEvent, PMDEventNumber number, PMDEventMode mode, PMDparam periodus);
PMDCFunc PMDEventWait                   (PMDEventHandle *hEvent, PMDparam *eventvalue, PMDparam timeoutms);

#if defined(__cplusplus)
}
#endif

#endif

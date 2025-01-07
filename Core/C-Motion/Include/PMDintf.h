#ifndef PMD_Interface
#define	PMD_Interface


typedef enum {
	InterfaceNone		 = 0,
	InterfaceParallel	 = 1,
	InterfacePCI		 = 2,
	InterfaceISA		 = 3,
	InterfaceSerial		 = 4,
	InterfaceCOM 		 = 4,
	InterfaceCAN		 = 5,
	InterfaceTCP		 = 6,
	InterfaceUDP		 = 7,
	InterfaceSPI		 = 8,
	InterfacePRP		 = 9,
	InterfaceRemote		 = 0x10,
} PMDInterfaceType;

// These CAN addresses are in respect to a device that connects to a PMD device.
// A PMD device receives data at address 0x600 and transmits data to address 0x580.
#define CAN_ADDRESS_BASE_RX         0x580   
#define CAN_ADDRESS_BASE_TX         0x600   
#define CAN_ADDRESS_BASE_INT        0x180


#endif //PMD_Interface

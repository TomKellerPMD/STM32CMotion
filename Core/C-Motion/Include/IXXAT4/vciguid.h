/*****************************************************************************
 IXXAT Automation GmbH
******************************************************************************

 File    : VCIGUID.H
 Summary : VCI driver and device GUIDs.

 Date    : 2006-11-07
 Author  : Hartmut Heim

 Compiler: MSVC

******************************************************************************
 all rights reserved
*****************************************************************************/

#ifndef _VCIGUID_H_
#define _VCIGUID_H_

#include <stdtype.h>

/*****************************************************************************
 * Driver and Device Classes from the PC-I04/PCI driver (VCI002Wx.SYS)
 ****************************************************************************
 * DriverClass = {0971DA4A-D77A-42A2-8E36-9E5B3BD7747D}
 * DeviceClass = {E7752D2E-28A3-4318-AF0C-02C756FBC25A}
 ****************************************************************************/
DEFINE_GUID(GUID_PCI04PCI_DRIVER,
       0x0971DA4A,0xD77A,0x42A2,0x8E,0x36,0x9E,0x5B,0x3B,0xD7,0x74,0x7D);

DEFINE_GUID(GUID_PCI04PCI_DEVICE,
       0xE7752D2E,0x28A3,0x4318,0xAF,0x0C,0x02,0xC7,0x56,0xFB,0xC2,0x5A);

/*****************************************************************************
 * Driver and Device Classes from the PC-I04/104 driver (VCI003Wx.SYS)
 ****************************************************************************
 * DriverClass = {5D5B513B-2576-4EB3-8232-17563C1FCFED}
 * DeviceClass = {0D2F015D-984E-4E95-86A0-C9AF51132FA6}
 ****************************************************************************/
DEFINE_GUID(GUID_PCI04104_DRIVER,
       0x5D5B513B,0x2576,0x4EB3,0x82,0x32,0x17,0x56,0x3C,0x1F,0xCF,0xED);

DEFINE_GUID(GUID_PCI04104_DEVICE,
       0x0D2F015D,0x984E,0x4E95,0x86,0xA0,0xC9,0xAF,0x51,0x13,0x2F,0xA6);

/*****************************************************************************
 * Driver and Device Classes from the CANdy lite driver (VCI004Wx.SYS)
 ****************************************************************************
 * DriverClass = {DD33B213-E5D8-47C4-8939-D7282F886AC3}
 * DeviceClass = {A323BB9E-A31B-4FD0-B603-29CCD57801A7}
 ****************************************************************************/
DEFINE_GUID(GUID_CANDYLITE_DRIVER,
       0xDD33B213,0xE5D8,0x47C4,0x89,0x39,0xD7,0x28,0x2F,0x88,0x6A,0xC3);

DEFINE_GUID(GUID_CANDYLITE_DEVICE,
       0xA323BB9E,0xA31B,0x4FD0,0xB6,0x03,0x29,0xCC,0xD5,0x78,0x01,0xA7);




/*****************************************************************************
 * Driver and Device Classes from the iPC-I320/PCI driver (VCI101Wx.SYS)
 ****************************************************************************
 * DriverClass = {E507E5BB-08AD-4BC4-BCB0-ED99C51D1C54}
 * DeviceClass = {78B139E9-503A-47A8-B671-B2FC29444717}
 ****************************************************************************/
DEFINE_GUID(GUID_IPCI320PCI_DRIVER, 
       0xe507e5bb,0x08ad,0x4bc4,0xbc,0xb0,0xed,0x99,0xc5,0x1d,0x1c,0x54);

DEFINE_GUID(GUID_IPCI320PCI_DEVICE, 
       0x78b139e9,0x503a,0x47a8,0xb6,0x71,0xb2,0xfc,0x29,0x44,0x47,0x17);

/*****************************************************************************
 * Driver and Device Classes from the iPC-I320/104 driver (VCI102Wx.SYS)
 ****************************************************************************
 * DriverClass = {198E28C5-C144-499D-B585-0638C4991533}
 * DeviceClass = {9BDA0CEA-B706-44EF-A453-8208667EB436}  
 ****************************************************************************/
DEFINE_GUID(GUID_IPCI320104_DRIVER, 
       0x198e28c5,0xc144,0x499d,0xb5,0x85,0x06,0x38,0xc4,0x99,0x15,0x33);

DEFINE_GUID(GUID_IPCI320104_DEVICE, 
       0x9bda0cea,0xb706,0x44ef,0xa4,0x53,0x82,0x08,0x66,0x7e,0xb4,0x36);
       
/*****************************************************************************
 * Driver and Device Classes from the iPC-I165/ISA driver (VCI103Wx.SYS)
 ****************************************************************************
 * DriverClass = {DAE85408-D658-4ee6-84D8-0731609875BE}
 * DeviceClass = {9D9AFCE2-C3FA-42b1-8325-E8760816F16C}
 ****************************************************************************/
DEFINE_GUID(GUID_IPCI165ISA_DRIVER, 
       0xdae85408, 0xd658, 0x4ee6, 0x84, 0xd8, 0x7, 0x31, 0x60, 0x98, 0x75, 0xbe);
       
DEFINE_GUID(GUID_IPCI165ISA_DEVICE,
       0x9d9afce2, 0xc3fa, 0x42b1, 0x83, 0x25, 0xe8, 0x76, 0x8, 0x16, 0xf1, 0x6c);

/*****************************************************************************
 * Driver and Device Classes from the iPC-I165/PCI driver (VCI104Wx.SYS)
 ****************************************************************************
 * DriverClass = {AFFDD770-A0EB-4418-9313-4AEE685A22C9}
 * DeviceClass = {1E28CE42-10AB-41A8-BF00-4DA611AB05FF}
 ****************************************************************************/
DEFINE_GUID(GUID_IPCI165PCI_DRIVER, 
       0xAFFDD770,0xA0EB,0x4418,0x93,0x13,0x4A,0xEE,0x68,0x5A,0x22,0xC9);

DEFINE_GUID(GUID_IPCI165PCI_DEVICE, 
       0x1E28CE42,0x10AB,0x41A8,0xBF,0x00,0x4D,0xA6,0x11,0xAB,0x05,0xFF);

/*****************************************************************************
 * Driver and Device Classes from the iPC-IXC161/PCI driver (VCI105Wx.SYS)
 ****************************************************************************
 * DriverClass = {63D85058-8DD1-41D2-A3FF-2D25F3049152}
 * DeviceClass = {789CE2B8-814B-48A9-816A-C2E7C1E7D731}
 ****************************************************************************/
DEFINE_GUID(GUID_IPCIXC16PCI_DRIVER, 
       0x63d85058,0x8dd1,0x41d2,0xa3,0xff,0x2d,0x25,0xf3,0x04,0x91,0x52);

DEFINE_GUID(GUID_IPCIXC16PCI_DEVICE, 
       0x789ce2b8,0x814b,0x48a9,0x81,0x6a,0xc2,0xe7,0xc1,0xe7,0xd7,0x31);

/*****************************************************************************
 * Driver and Device Classes from the tinCANV4 driver (VCI106Wx.SYS)
 ****************************************************************************
 * DriverClass = {D852C125-659C-4A17-8651-3D788CA3FB64}
 * DeviceClass = {5A56027D-3D41-4A82-9821-F4E46FA26F0B}
 ****************************************************************************/
DEFINE_GUID(GUID_TINCANV4_DRIVER, 
       0xd852c125,0x659c,0x4a17,0x86,0x51,0x3d,0x78,0x8c,0xa3,0xfb,0x64);

DEFINE_GUID(GUID_TINCANV4_DEVICE, 
       0x5a56027d,0x3d41,0x4a82,0x98,0x21,0xf4,0xe4,0x6f,0xa2,0x6f,0x0b);

/*****************************************************************************
 * Driver and Device Classes from the tinCAN 161 driver (VCI107Wx.SYS)
 ****************************************************************************
 * DriverClass = {E3464ACF-7ED0-4BC6-AC46-AF7F1D61DB9F}
 * DeviceClass = {23E89775-1F7A-4CCE-90B9-E7182952DB35}
 ****************************************************************************/
DEFINE_GUID(GUID_TINCAN161_DRIVER, 
       0xe3464acf,0x7ed0,0x4bc6,0xac,0x46,0xaf,0x7f,0x1d,0x61,0xdb,0x9f);

DEFINE_GUID(GUID_TINCAN161_DEVICE, 
       0x23e89775,0x1f7a,0x4cce,0x90,0xb9,0xe7,0x18,0x29,0x52,0xdb,0x35);

/*****************************************************************************
 * Driver and Device Classes from the CANblue driver (VCI108Wx.SYS)
 ****************************************************************************
 * DriverClass = {B4009307-A1C2-48AA-990A-60C2B424D5C3}
 * DeviceClass = {021AAC76-B2FA-4909-BEB1-67D6C2283359}
 ****************************************************************************/
DEFINE_GUID(GUID_CANBLUE_DRIVER, 
       0xb4009307,0xa1c2,0x48aa,0x99,0x0a,0x60,0xc2,0xb4,0x24,0xd5,0xc3);

DEFINE_GUID(GUID_CANBLUE_DEVICE, 
       0x021aac76,0xb2fa,0x4909,0xbe,0xb1,0x67,0xd6,0xc2,0x28,0x33,0x59);

/*****************************************************************************
 * Driver and Device Classes from the USB-to-CAN compact driver (VCI109Wx.SYS)
 ****************************************************************************
 * DriverClass = {B5B707C4-79A2-4BA4-A9C4-4C75FB9F37F7}
 * DeviceClass = {853857B3-0B08-454C-93FB-2D166B72A5AA}
 ****************************************************************************/
DEFINE_GUID(GUID_USB2CANCOMPACT_DRIVER, 
       0xB5B707C4,0x79A2,0x4ba4,0xa9,0xC4,0x4C,0x75,0xFB,0x9F,0x37,0xF7);

DEFINE_GUID(GUID_USB2CANCOMPACT_DEVICE, 
       0x853857b3,0x0b08,0x454c,0x93,0xfb,0x2d,0x16,0x6b,0x72,0xa5,0xaa);

/*****************************************************************************
 * Driver and Device Classes from the USB-to-CAN II driver (VCI10AWx.SYS)
 ****************************************************************************
 * DriverClass = {BB445600-8721-4198-8090-21B72E917616}
 * DeviceClass = {13942C1E-CD73-4B7A-978B-8C2CD0B48394}
 ****************************************************************************/
DEFINE_GUID(GUID_USB2CANII_DRIVER, 
       0xbb445600,0x8721,0x4198,0x80,0x90,0x21,0xb7,0x2e,0x91,0x76,0x16);
        
DEFINE_GUID(GUID_USB2CANII_DEVICE, 
       0x13942c1e,0xcd73,0x4b7a,0x97,0x8b,0x8c,0x2c,0xd0,0xb4,0x83,0x94);

/*****************************************************************************
 * Driver and Device Classes from the CAN@net2 driver (VCI10CWx.SYS)
 ****************************************************************************
 * DriverClass = {3B3A6402-EC63-4B66-83DB-BF1A29C7DE67}
 * DeviceClass = {97C04C74-FA6D-439D-9E32-359486385607}
 ****************************************************************************/
DEFINE_GUID(GUID_CANATNET2_DRIVER, 
       0x3B3A6402,0xEC63,0x4B66,0x83,0xDB,0xBF,0x1A,0x29,0xC7,0xDE,0x67);

DEFINE_GUID(GUID_CANATNET2_DEVICE, 
       0x97C04C74,0xFA6D,0x439D,0x9E,0x32,0x35,0x94,0x86,0x38,0x56,0x07);

/*****************************************************************************
 * Driver and Device Classes from the EPL-IB-2xx/(c)PCI driver (VCI10DWx.SYS)
 ****************************************************************************
 * DriverClass = {6418B4E9-AC1E-4DCF-939A-2F454B6C9DAB}
 * DeviceClass = {D3DBCF02-C257-4E54-A8F4-AA26C9990568}
 ****************************************************************************/
DEFINE_GUID(GUID_EPLIB2XXPCI_DRIVER, 
       0x6418B4E9,0xAC1E,0x4DCF,0x93,0x9A,0x2F,0x45,0x4B,0x6C,0x9D,0xAB);

DEFINE_GUID(GUID_EPLIB2XXPCI_DEVICE, 
       0xD3DBCF02,0xC257,0x4E54,0xA8,0xF4,0xAA,0x26,0xC9,0x99,0x05,0x68);

/*****************************************************************************
 * Driver and Device Classes from the iPC-IXC161/PCIe driver (VCI10EWx.SYS)
 *   PCI express card
 ****************************************************************************
 * DriverClass = {F5AED1F1-BF55-4913-8EFB-16959FD770C0}
 * DeviceClass = {A196624D-DCC4-41b1-819B-16FFA1125589}
 ****************************************************************************/
DEFINE_GUID(GUID_IPCIXC16PCIE_DRIVER, 
       0xF5AED1F1,0xBF55,0x4913,0x8E,0xFB,0x16,0x95,0x9F,0xD7,0x70,0xC0);
DEFINE_GUID(GUID_IPCIXC16PCIE_DEVICE, 
       0xA196624D,0xDCC4,0x41B1,0x81,0x9B,0x16,0xFF,0xA1,0x12,0x55,0x89);
       
/*****************************************************************************
 * Driver and Device Classes from the PL-IB320/PCI driver (VCI10FWx.SYS)
 ****************************************************************************
 * DriverClass = {8BB19F59-2C0D-42da-B9CC-7EE40D367992}
 * DeviceClass = {32E9654E-9B34-4ea0-940B-1C52B1D38249}
 ****************************************************************************/
DEFINE_GUID(GUID_PLIB320PCI_DRIVER, 
       0x8bb19f59,0x2c0d,0x42da,0xb9,0xcc,0x7e,0xe4,0x0d,0x36,0x79,0x92);
      
DEFINE_GUID(GUID_PLIB320PCI_DEVICE, 
       0x32e9654e,0x9b34,0x4ea0,0x94,0x0b,0x1c,0x52,0xb1,0xd3,0x82,0x49);
       

#endif //_VCIGUID_H_

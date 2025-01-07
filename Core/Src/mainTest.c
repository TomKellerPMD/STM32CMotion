/**
  ******************************************************************************
  * @file    UART/UART_TwoBoards_ComPolling/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code shows how to use UART HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          polling transfer. 
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "c-motion.h"
#include "PMDperiph.h"
#include "PMDsys.h"
#include "PMDdiag.h"
#include "PMDutil.h"
#include "PMDdevice.h"
//#include "examples.h"

/* Private function prototypes -----------------------------------------------*/
static void MPU_Config(void);
static void SystemClock_Config(void);
//static void Error_Handler(void);
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
static void CPU_CACHE_Enable(void);

//#define TEST

#ifdef TEST

/* UART handler declaration */
extern UART_HandleTypeDef UartHandle;
__IO uint32_t UserButtonStatus = 0;  /* set to 1 after User Button interrupt  */

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling **** ";
/* Buffer used for transmission */

uint8_t aTxBufferTest[10];
uint8_t aRxBufferTest[10];
uint8_t aRxBufferTest1[10];
uint8_t aRxBufferTest2[10];

/* Buffer used for reception */

#define TXBUFFTESTSIZE 4
uint8_t aRxBuffer[RXBUFFERSIZE];
#define RXBUFFTESTSIZE 4

#endif

PMDresult result; // result value used by PMD_RESULT macro (made global to simplify the example functions)

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */

PMDresult SetupAxis1(PMDAxisHandle* phAxis);


int main(void)
{

  PMDInterfaceType Interface = InterfaceSerial;
  PMDPeriphHandle hPeriph1,hPeriph2;
  PMDDeviceHandle hDevice;
  PMDDeviceHandle *phDevice = NULL;
  PMDAxisHandle hAxis;
  PMDuint32 vmajor;
  PMDuint32 vminor;
  PMDuint32 version;
  PMDuint16 status;

  PMDint32 position1,position2;

  /* Configure the MPU attributes */
  MPU_Config();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32H7xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 400 MHz */
  SystemClock_Config();
  

#ifndef TEST
      // If this is a host computer open the local peripheral and device handle that is connected to the ION device.


      PMDTaskWait(100);
      // To do: Select one of the following interfaces to connect to the device
      if (Interface == InterfaceSerial)
      {
          // This is just for reference.  main.h needs modifying if UART6 is not used.
    	  result=PMDPeriphOpenCOM( &hPeriph2, NULL, 1, 115200, PMDSerialParityNone, PMDSerialStopBits1);

          // USART6 works as is.
          result=PMDPeriphOpenCOM( &hPeriph1, NULL, USART6, 115200, PMDSerialParityNone, PMDSerialStopBits1);
          //PMD_ABORTONERROR(PMDPeriphOpenMultiDrop(&hPeriph, &hPeriph, nodeID)) enable this line if multidrop mode (RS485)
      }
#if(0)
      else
      if (Interface == InterfaceTCP)
      {
          PMDprintf("Connecting via TCP\r\n");
          PMD_ABORTONERROR(PMDPeriphOpenTCP( &hPeriph, NULL, PMD_IP4_ADDR(192,168,2,2), 40100, 0 ))
      }
      else
      if (Interface == InterfaceCAN)
      {
          PMDprintf("Connecting via CAN\r\n");
          int baud = PMDCANBaud1000000;
  		int nodeID = 0;
  		PMD_ABORTONERROR(PMDDeviceOpenPeriphCANNodeID(&hPeriph, NULL, baud, nodeID))

      }
      // Open a handle to a PRP (PMD Resource Protocol) device. Also referred as CME (C-Motion Engine)
      PMD_ABORTONERROR(PMDRPDeviceOpen(&hDevice, &hPeriph))
      phDevice = &hDevice;
#endif


      // Open a handle to a PRP (PMD Resource Protocol) device. Also referred as CME (C-Motion Engine)
      PMD_ABORTONERROR(PMDRPDeviceOpen(&hDevice, &hPeriph1))
      //result=PMDRPDeviceOpen(&hDevice, &hPeriph);
      phDevice = &hDevice;

      result=PMDDeviceGetVersion(&hDevice, &vmajor, &vminor);
      if(result==0x1002)
      {
    	  result=PMDDeviceGetVersion(&hDevice, &vmajor, &vminor);
      }

      // Motion IC example functions
      PMD_ABORTONERROR(PMDAxisOpen(&hAxis, phDevice, PMDAxis1 ))
      PMD_RESULT(PMDGetVersion32(&hAxis, &version));
      PMDprintf("Magellan version %08X\r\n", version);

      result=SetupAxis1(&hAxis);



      position1=0;
      position2=2000;
      PMDSetVelocity(&hAxis, 10000);
      PMDSetAcceleration(&hAxis, 10000);

      PMDSetActualPosition(&hAxis,position1);  // This forces the current location of the motor to be position1.



      while(1)
      {
    	  PMDSetPosition(&hAxis,position2);
    	  PMDResetEventStatus(&hAxis,0xFFFE);
      	  PMDUpdate(&hAxis);

    	  status=0;
      	  while(!(status&0x0001))
      	  {
      		  PMD_ABORTONERROR(PMDGetEventStatus(&hAxis, &status));
      	  }

      	  PMDSetPosition(&hAxis,position1);
      	  PMDResetEventStatus(&hAxis,0xFFFE);
      	  PMDUpdate(&hAxis);

      	  status=0;
      	  while(!(status&0x0001))
      	  {
      	  		  PMD_ABORTONERROR(PMDGetEventStatus(&hAxis, &status));
      	  }
      }





#else



  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }
  
  
  aTxBufferTest[0]=0x6B;   // This will send GetDeviceVersion
  aTxBufferTest[1]=0x04;
  aTxBufferTest[2]=0x6A;
  aTxBufferTest[3]=0x00;
  aTxBufferTest[4]=0x01;
  aTxBufferTest[5]=0x0;
  txsize=6;
  rxsize=7;

  result=HAL_UART_Receive(&UartHandle, (uint8_t *)aRxBufferTest, 20, 5);  //clears over flow flag

  while(1)
  {
	  result=HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBufferTest, txsize, 5000);
	  if(result)  Error_Handler;
	  result=HAL_UART_Receive(&UartHandle, (uint8_t *)t1, rxsize, 5000);
	  if(result)  Error_Handler;

  }

#endif

}


// N-Series ION DD481S0056/06
PMDresult SetupAxis1(PMDAxisHandle* phAxis)
{
  PMDresult result = PMD_NOERROR;
  PMD_RESULT(PMDSetOperatingMode (phAxis, 1))
  PMD_RESULT(PMDSetSampleTime (phAxis, 51))
  PMD_RESULT(PMDSetMotorType (phAxis, 0))
  PMD_RESULT(PMDResetEventStatus (phAxis, 0x0000))
  PMD_RESULT(PMDSetCommutationParameter (phAxis, PMDCommutationParameterPhaseCounts, 2000))
  PMD_RESULT(PMDSetPositionErrorLimit (phAxis, 65535))
  PMD_RESULT(PMDSetSettleTime (phAxis, 0))
  PMD_RESULT(PMDSetSettleWindow (phAxis, 0))
  PMD_RESULT(PMDSetTrackingWindow (phAxis, 0))
  PMD_RESULT(PMDSetEncoderSource (phAxis, 0))
  PMD_RESULT(PMDSetGearMaster (phAxis, 0, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKp, 100))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKi, 80))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopIlimit, 1000000))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKd, 2000))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopDerivativeTime, 1))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKout, 5655))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKvff, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopKaff, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1Enable, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1B0, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1B1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1B2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1A1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1A2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad1K, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2Enable, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2B0, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2B1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2B2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2A1, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2A2, 0))
  PMD_RESULT(PMDSetPositionLoop (phAxis, PMDPositionLoopBiquad2K, 0))
  PMD_RESULT(PMDSetMotorBias (phAxis, 0))
  PMD_RESULT(PMDSetMotorLimit (phAxis, 32767))
  PMD_RESULT(PMDSetMotorCommand (phAxis, 3277))
  PMD_RESULT(PMDSetMotionCompleteMode (phAxis, 0))
  PMD_RESULT(PMDSetSignalSense (phAxis, 0x0000))
  PMD_RESULT(PMDSetCaptureSource (phAxis, 1))
  PMD_RESULT(PMDSetCommutationParameter (phAxis, PMDCommutationParameterPhaseDenominator, 1))
  PMD_RESULT(PMDSetCommutationMode (phAxis, 0))
  PMD_RESULT(PMDSetPhaseCorrectionMode (phAxis, 0))
  PMD_RESULT(PMDSetPhaseInitializeMode (phAxis, 0))
  PMD_RESULT(PMDSetPhasePrescale (phAxis, 0))
  PMD_RESULT(PMDSetPhaseParameter (phAxis, PMDPhaseParameterRampTime, 9804))
  PMD_RESULT(PMDSetPhaseParameter (phAxis, PMDPhaseParameterPositivePulseTime, 0))
  PMD_RESULT(PMDSetPhaseParameter (phAxis, PMDPhaseParameterNegativePulseTime, 0))
  PMD_RESULT(PMDSetPhaseParameter (phAxis, PMDPhaseParameterPulseMotorCommand, 0))
  PMD_RESULT(PMDSetPhaseParameter (phAxis, PMDPhaseParameterRampCommand, 3277))
  PMD_RESULT(PMDSetBreakpointValue (phAxis, 0x0000, 0x00000000))
  PMD_RESULT(PMDSetBreakpointValue (phAxis, 0x0001, 0x00000000))
  PMD_RESULT(PMDSetBreakpoint (phAxis, 0x0000, 0, 0, 0))
  PMD_RESULT(PMDSetBreakpoint (phAxis, 0x0001, 0, 0, 0))
  PMD_RESULT(PMDSetAuxiliaryEncoderSource (phAxis, 0, 0))
  PMD_RESULT(PMDSetCurrentControlMode (phAxis, 1))
  PMD_RESULT(PMDSetFOC (phAxis, 2, 0, 200))
  PMD_RESULT(PMDSetFOC (phAxis, 2, 1, 150))
  PMD_RESULT(PMDSetFOC (phAxis, 2, 2, 32767))
  PMD_RESULT(PMDSetCurrentLoop (phAxis, 2, 0, 0))
  PMD_RESULT(PMDSetCurrentLoop (phAxis, 2, 1, 0))
  PMD_RESULT(PMDSetCurrentLoop (phAxis, 2, 2, 32767))
  PMD_RESULT(PMDSetAxisOutMask (phAxis, 0, 0, 0x0001, 0x0000))
  PMD_RESULT(PMDSetFaultOutMask (phAxis, 0x0600))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterOvervoltageLimit, 6000))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterUndervoltageLimit, 1000))
  PMD_RESULT(PMDSetCurrentFoldback (phAxis, PMDCurrentFoldbackContinuousCurrentLimit, 10614))
  PMD_RESULT(PMDSetCurrentFoldback (phAxis, PMDCurrentFoldbackI2tEnergyLimit, 425))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventPositiveLimit, 0x0008))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventNegativeLimit, 0x0008))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventMotionError, 0x0005))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventCurrentFoldback, 0x0007))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMLimit, 16384))
  PMD_RESULT(PMDSetDrivePWM (phAxis, PMDDrivePWMFrequency, 5000))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterTemperatureLimit, 19200))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterTemperatureHysteresis, 1280))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterShuntVoltage, 65535))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterShuntPWMDutyCycle, 0))
  PMD_RESULT(PMDSetDriveFaultParameter (phAxis, PMDDriveFaultParameterBusCurrentReturnLimit, 65534))
  PMD_RESULT(PMDSetEventAction (phAxis, PMDEventActionEventBrakeAsserted, 0x000a))
  PMD_RESULT(PMDUpdate (phAxis))
  PMD_RESULT(PMDSetOperatingMode (phAxis, 0x0003))
  PMD_RESULT(PMDSetMotorCommand (phAxis, 3277))
  PMD_RESULT(InitializePhase (phAxis))
  PMD_RESULT(PMDSetMotorCommand (phAxis, 0))
  PMD_RESULT(PMDSetOperatingMode (phAxis, 0x0037))
  return result;
}





/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE BYPASS)
  *            SYSCLK(Hz)                     = 400000000 (CPU Clock)
  *            HCLK(Hz)                       = 200000000 (AXI and AHBs Clock)
  *            AHB Prescaler                  = 2
  *            D1 APB3 Prescaler              = 2 (APB3 Clock  100MHz)
  *            D2 APB1 Prescaler              = 2 (APB1 Clock  100MHz)
  *            D2 APB2 Prescaler              = 2 (APB2 Clock  100MHz)
  *            D3 APB4 Prescaler              = 2 (APB4 Clock  100MHz)
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 4
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 4
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /*!< Supply configuration update enable */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
  
/* Select PLL as system clock source and configure  bus clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_D1PCLK1 | RCC_CLOCKTYPE_PCLK1 | \
                                 RCC_CLOCKTYPE_PCLK2  | RCC_CLOCKTYPE_D3PCLK1);

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;  
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2; 
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2; 
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2; 
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

}
/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  Error_Handler();
}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 // if(GPIO_Pin == BUTTON_USER_PIN)
 // {
 //   UserButtonStatus = 1;
 // }
}



/**
  * @brief  Configure the MPU attributes
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU as Strongly ordered for not defined regions */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x00;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

static void Error_Handler(void)
{
  /* Turn LED_RED on */
 // BSP_LED_On(LED3);

  while(1)
  {
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}

/**
  * @}
  */

/**
  * @}
  */


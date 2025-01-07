#if defined(__cplusplus)
extern "C" {
#endif

PMDresult PMDPCOM_Open(PMDPeriphHandle* hPeriph, PMDparam portnum, PMDparam baud, PMDSerialParity parity, PMDSerialStopBits stopbits);

#if(0)
typedef struct {   uint32_t SR;
				    uint32_t DR;
				    uint32_t BRR;
				    uint32_t CR1;
				   uint32_t CR2;
				    uint32_t CR3;
				    uint32_t GTPR;
} USART_TypeDef;

typedef struct {   uint32_t BaudRate;
				   uint32_t WordLength;
				   uint32_t StopBits;
				   uint32_t Parity;
				   uint32_t Mode;
				   uint32_t HwFlowCtl;
				   uint32_t OverSampling;
} UART_InitTypeDef;




typedef struct __UART_HandleTypeDef {
	USART_TypeDef                 *Instance;
	UART_InitTypeDef              Init;
	const uint8_t                 *pTxBuffPtr;
	uint16_t                      TxXferSize;
	uint16_t                 TxXferCount;
	uint8_t                       *pRxBuffPtr;
	uint16_t                      RxXferSize;
	uint16_t                 RxXferCount;
#if(0)
	HAL_UART_RxTypeTypeDef ReceptionType;
	HAL_UART_RxEventTypeTypeDef RxEventType;
	DMA_HandleTypeDef             *hdmatx;
	DMA_HandleTypeDef             *hdmarx;
	HAL_LockTypeDef               Lock;
	HAL_UART_StateTypeDef    gState;
	HAL_UART_StateTypeDef    RxState;
	uint32_t                 ErrorCode;
	#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)   void (* TxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);
	void (* TxCpltCallback)(struct __UART_HandleTypeDef *huart);
	void (* RxHalfCpltCallback)(struct __UART_HandleTypeDef *huart);
	void (* RxCpltCallback)(struct __UART_HandleTypeDef *huart);
	void (* ErrorCallback)(struct __UART_HandleTypeDef *huart);
	void (* AbortCpltCallback)(struct __UART_HandleTypeDef *huart);
	void (* AbortTransmitCpltCallback)(struct __UART_HandleTypeDef *huart);
	void (* AbortReceiveCpltCallback)(struct __UART_HandleTypeDef *huart);
	void (* WakeupCallback)(struct __UART_HandleTypeDef *huart);
	void (* RxEventCallback)(struct __UART_HandleTypeDef *huart, uint16_t Pos);
	void (* MspInitCallback)(struct __UART_HandleTypeDef *huart);
	void (* MspDeInitCallback)(struct __UART_HandleTypeDef *huart);
	#endif
#endif
	} UART_HandleTypeDef;

#endif



#if defined(__cplusplus)
}
#endif

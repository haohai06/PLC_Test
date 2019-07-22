#ifndef __HAL_UART_H__
#define __HAL_UART_H__
#include "stm32f0xx_hal.h"

#define HAL_USART                          USART2
#define USART1_CLK_ENABLE()             __HAL_RCC_USART1_CLK_ENABLE()
#define USART1_TX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART1_RX_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()

#define USART1_FORCE_RESET()            __HAL_RCC_USART1_FORCE_RESET()
#define USART1_RELEASE_RESET()          __HAL_RCC_USART1_RELEASE_RESET()

#define USART1_TX_PIN                    GPIO_PIN_2
#define USART1_TX_GPIO_PORT              GPIOA
#define USART1_TX_AF                     GPIO_AF1_USART1
#define USART1_RX_PIN                    GPIO_PIN_3
#define USART1_RX_GPIO_PORT              GPIOA
#define USART1_RX_AF                     GPIO_AF1_USART1


#define HAL_USART1_IRQn                      USART1_IRQn

#define CTRL_STATUS_TX	 0
#define CTRL_STATUS_RX	 1

typedef void (*RecvActiveCallback_t)(uint8_t rxtxStatus);

extern uint8_t LOCK_APP_UATR_Init(uint32_t baudrate,uint8_t *rxBuffer,uint16_t rxBufferSize,RecvActiveCallback_t callback);

extern uint8_t LOCK_APP_UART_RX_PIN_Init(void);

extern void HAL_UART2_IRQHandler(void);

extern uint16_t HAL_UART_RxLength(void);
extern uint16_t HAL_UARTRead(uint8_t *pData,uint16_t len);
extern uint16_t HAL_UARTReadPoint(uint8_t *pData,uint16_t len,uint16_t offset);
extern uint16_t HAL_UARTReadClear(uint8_t *pData,uint16_t len);
extern uint16_t HAL_UARTRxClear(uint16_t len);
extern uint16_t HAL_UART_Tx(uint8_t *pTxBuffer,uint16_t txLen);
extern void BSP_UartIRQHandler(void);

void HAL_UARTRxWkUpIntrrupEnable(void);
void HAL_UARTRxWkUpIntrrupDisable(void);

int fputc(int ch,FILE *f);
#endif /*__HAL_UART_H__*/

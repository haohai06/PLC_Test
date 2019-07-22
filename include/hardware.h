#ifndef __HARDWARE_H__
#define __HARDWARE_H__

#include "main.h"
#include "stm32f0xx_hal.h"

#define SHOCK_SENSOR_IO_EVENT  0x1

#define HAL_WKP_SRC_IOEVENT 	 (0x01<<0)
#define HAL_WKP_SRC_UART_EVENT   (0x01<<1)
#define HAL_WKP_SRC_BLE_EVENT    (0x01<<2)
#define HAL_WKP_SRC_RTC_TIMEOUT  (0x01<<3)

void HAL_InitHardware(void);
void HAL_Wakeup_Source_Set(uint8_t event);
uint8_t Hal_CPU_Before_Sleep(void);
uint32_t Hal_CPU_IntoSleep(uint32_t sleepMs);
void Hal_CPU_Before_Wakeup(void);
#endif /*__HARDWARE_H__*/

#ifndef __IO_EVENT_H__
#define  __IO_EVENT_H__

#include "stm32f0xx_hal.h"

#define BUTTON_PORT       GPIOA
#define BUTTON_OPEN_PIN   GPIO_PIN_7
#define BUTTON_CLOSE_PIN  GPIO_PIN_6

#define SHOCK_SENSOR_PORT GPIOB
#define SHOCK_SENSOR_PIN  GPIO_PIN_1

#define UART_RX_LINE_PORT  GPIOA
#define UART_RX_LINE_PIN   GPIO_PIN_3

//#define gpio_pin_interrupt_disable(GPIOx, pin)   HAL_GPIO_DeInit(GPIOx,pin)
void gpio_pin_interrupt_enable(GPIO_TypeDef  *GPIOx, uint32_t pin,uint32_t mode);
void gpio_pin_interrupt_disable(GPIO_TypeDef  *GPIOx, uint32_t pin);

void lock_gpio_init(void);
void io_event_irq_handler(GPIO_TypeDef        *GPIOx,uint16_t GPIO_Pin);
uint32_t lock_io_event_handle(void);
//void shock_sensor_interrupt_disable();
//void shock_sensor_interrupt_enable();

//void mpu6050_gpio_init(void);


#endif /*__IO_EVENT_H__*/


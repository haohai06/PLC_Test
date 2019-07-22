#ifndef __LOCK_MOTOR_H__
#define __LOCK_MOTOR_H__
#include "stdint.h"

#define LOCK_MOTOR_PORT   GPIOA
#define MOTOR_LOCK_PIN    GPIO_PIN_4
#define MOTOR_UNLOCK_PIN  GPIO_PIN_5

#define MOTOR_FEEDBACK_PORT  GPIOA
#define MOTOR_FEEDBACK_PIN   GPIO_PIN_0
void lock_motor_adc_init(void);
void door_lock_motor_init(void);

//uint8_t motor_feedback_pin_adc(uint8_t time,uint32_t *value);

void door_lock_open(void);
void door_lock_close(void);
void door_motor_stop(void);
void door_motor_brake(void);
#endif /*__LOCK_MOTOR_H__*/


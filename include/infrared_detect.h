#ifndef __INFRARED_DETECT_H__
#define __INFRARED_DETECT_H__


#define INFRARED_DETECT_PORT    GPIOB
#define INFRARED_L_IN_PIN       GPIO_PIN_3
#define INFRARED_R_IN_PIN       GPIO_PIN_5
#define INFRARED_L_OUT_PIN      GPIO_PIN_4
#define INFRARED_R_OUT_PIN      GPIO_PIN_6



#define LEFT_SIDE_CLOSE   0
#define RIGHT_SIDE_CLOSE  1

#define IFRARED_IN_PWR_ON_OFF(on)  do{ HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,on ? GPIO_PIN_SET :  GPIO_PIN_RESET);}while(0)

extern uint8_t infrared_detect_task_id;


void infrared_detect_init();

int  infrared_detect_handler();










#endif /*__INFRARED_DETECT_H__*/


#include "stm32f0xx.h"
#include "io_event.h"
//#include "infrared_detect.h"
#include "hardware.h"
#include "slave_lock_task.h"
#include "lock_app_uart.h"
#include "hal_uart.h"

extern uint8_t slave_lock_task_id;

extern uint8_t wakeup_src;

void gpio_irq_event_hander(uint16_t GPIO_Pin);

static uint32_t open_btn_press_cnt  = 0;
static uint32_t close_btn_press_cnt = 0;



void gpio_pin_interrupt_enable(GPIO_TypeDef  *GPIOx, uint32_t pin,uint32_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin  = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void gpio_pin_interrupt_disable(GPIO_TypeDef  *GPIOx, uint32_t pin)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    HAL_GPIO_DeInit(GPIOx,pin);

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin  = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void lock_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pins : PA6 PA7 */
    GPIO_InitStruct.Pin  = BUTTON_OPEN_PIN|BUTTON_CLOSE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);


    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 3, 0); //EXTI2_3_IRQn
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void io_event_irq_handler(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
    HAL_Wakeup_Source_Set(HAL_WKP_SRC_IOEVENT);
    /* EXTI line interrupt detected */
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_Pin) != RESET)
    {
        switch(GPIO_Pin)
        {
        case UART_RX_LINE_PIN:
            uart_interrupt_handler();
            //osal_set_event(slave_lock_task_id,LOCK_OPEN_EVENT);
            break;
        case BUTTON_OPEN_PIN:
        case BUTTON_CLOSE_PIN:
            //uart_interrupt_handler();
            //lock_app_send_req_admin_menu();
            //gpio_pin_interrupt_disable(GPIOA,UART_RX_LINE_PIN);
            //osal_set_event(slave_lock_task_id,LOCK_OPEN_EVENT);
            osal_set_event(slave_lock_task_id,IOSIGNAL_EVENT);
            break;
        }
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
        gpio_pin_interrupt_disable(GPIOx,GPIO_Pin);
    }
}
static uint32_t hal_open_button_handler(void)
{
    //if (nrf_gpio_pin_read(BUTTON_OPEN_PIN) == 0)//
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_OPEN_PIN) == GPIO_PIN_RESET)
    {
        if (++open_btn_press_cnt >= 300)
        {
            open_btn_press_cnt = 0;
            //nrf_drv_gpiote_in_event_enable(BUTTON_OPEN_PIN, true);
            gpio_pin_interrupt_enable(BUTTON_PORT,BUTTON_OPEN_PIN,GPIO_MODE_IT_FALLING);
            lock_app_send_req_admin_menu();
            return 0;
        }
        if (open_btn_press_cnt == 30)
        {
            lock_app_send_req_lock(0);
            osal_set_event(slave_lock_task_id, LOCK_OPEN_EVENT);
        }
        return 1;
    }
    else
    {
        //nrf_drv_gpiote_in_event_enable(BUTTON_OPEN_PIN, true);
        gpio_pin_interrupt_enable(BUTTON_PORT,BUTTON_OPEN_PIN,GPIO_MODE_IT_FALLING);
        open_btn_press_cnt = 0;
    }
    return 0;
}

static uint32_t hal_close_button_handler(void)
{

    //if (nrf_gpio_pin_read(BUTTON_CLOSE_PIN) == 0)
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_CLOSE_PIN) == GPIO_PIN_RESET)
    {
        if (++close_btn_press_cnt >= 500)
        {
            //nrf_drv_gpiote_in_event_enable(BUTTON_CLOSE_PIN, true);
            gpio_pin_interrupt_enable(BUTTON_PORT,BUTTON_CLOSE_PIN,GPIO_MODE_IT_FALLING);
            lock_app_send_reverse_lock();
            close_btn_press_cnt = 0;
            return 0;
        }
        else if (close_btn_press_cnt == 30)
        {
            lock_app_send_req_lock(1);
            osal_set_event(slave_lock_task_id, LOCK_CLOSE_EVENT);
        }
        return 1;
    }
    else
    {
        //nrf_drv_gpiote_in_event_enable(BUTTON_CLOSE_PIN, true);
        gpio_pin_interrupt_enable(BUTTON_PORT,BUTTON_CLOSE_PIN,GPIO_MODE_IT_FALLING);
        close_btn_press_cnt = 0;
    }
    return 0;
}


uint32_t lock_io_event_handle(void)
{
    uint32_t ret = 0;
#if 0
    static uint32_t btn_press_cnt = 0;
    if ((HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_CLOSE_PIN) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_OPEN_PIN) == GPIO_PIN_RESET))
    {
        if (++btn_press_cnt >= 300)
        {
            btn_press_cnt = 0;
            gpio_pin_interrupt_enable(BUTTON_PORT,BUTTON_CLOSE_PIN,GPIO_MODE_IT_FALLING);
            gpio_pin_interrupt_enable(BUTTON_PORT,BUTTON_OPEN_PIN,GPIO_MODE_IT_FALLING);
            //osal_set_event(slave_lock_task_id, DOOR_BELL_PALY_EVENT);
            //start_read_Mag_ref();
            return 1;
        }
        return 0;
    }
    else
#endif
    {
        /* code */
        //btn_press_cnt = 0;
        ret |= hal_open_button_handler();
        ret |= hal_close_button_handler();
        return ret ? 0 : 1;
    }
}




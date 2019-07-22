#include "stm32f0xx.h"
#include "infrared_detect.h"
#include "lock_config.h"
#include "io_event.h"
#include "osal.h"
#define  PERIOD_VALUE       (208 )  /* Period Value  */
#define  PULSE1_VALUE       70         /* Capture Compare 1 Value  */


#define IR_MAX_SEND_NUM   10
#define IR_MAX_RECV_NUM   5

extern uint8_t slave_lock_task_id;

TIM_HandleTypeDef TimHandle;

static uint8_t m_detect_side = RIGHT_SIDE_DETECT;

static uint8_t m_detect_send_time = 0;
static uint8_t m_detect_right_rcv_count  = 0;
static uint8_t m_detect_left_rcv_count  = 0;
static uint8_t m_detect_lpin_last_state = GPIO_PIN_RESET;
static uint8_t m_detect_rpin_last_state = GPIO_PIN_RESET;

static uint8_t m_detect_start  =  FALSE;


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* TIMx Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    /*##-2- Configure the NVIC for TIMx ########################################*/
    /* Set the TIMx priority */
    HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);

    /* Enable the TIMx global Interrupt */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void pwm_tim_init()
{
    TimHandle.Instance = TIM3;

    TimHandle.Init.Period			 = 1;
    TimHandle.Init.Prescaler		 = PERIOD_VALUE / 2 ;
    TimHandle.Init.ClockDivision	 = 0;
    TimHandle.Init.CounterMode		 = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&TimHandle) ;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (m_detect_side & LEFT_SIDE_DETECT)
    {
        GPIOB->ODR ^= INFRARED_L_OUT_PIN;
    }
    if (m_detect_side & RIGHT_SIDE_DETECT)
    {
        GPIOB->ODR ^= INFRARED_R_OUT_PIN;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!m_detect_start)
        return;

    if (GPIO_Pin == INFRARED_L_IN_PIN && m_detect_side & LEFT_SIDE_DETECT)
    {
        m_detect_left_rcv_count++;
        if (m_detect_left_rcv_count > IR_MAX_RECV_NUM )
        {
            m_detect_left_rcv_count  = 0;
            m_detect_start           = FALSE;
            m_detect_send_time       = 0;
            IFRARED_IN_PWR_ON_OFF(0);
            shock_sensor_interrupt_enable();
            HAL_TIM_Base_Stop(&TimHandle);
            osal_set_event(slave_lock_task_id, INFRARED_DETECT_IN_EVENT);
        }
    }
    if (GPIO_Pin == INFRARED_R_IN_PIN && m_detect_side & RIGHT_SIDE_DETECT)
    {
        m_detect_right_rcv_count++;
        if (m_detect_right_rcv_count > IR_MAX_RECV_NUM )
        {
            m_detect_right_rcv_count = 0;
            m_detect_start           = FALSE;
            m_detect_send_time       = 0;
            IFRARED_IN_PWR_ON_OFF(0);
            shock_sensor_interrupt_enable();
            HAL_TIM_Base_Stop(&TimHandle);
            osal_set_event(slave_lock_task_id, INFRARED_DETECT_IN_EVENT);
        }
    }
}

static void infrared_detect_io_pin_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin  = INFRARED_L_IN_PIN | INFRARED_R_IN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    //ir out
    GPIO_InitStruct.Pin  = INFRARED_L_OUT_PIN | INFRARED_R_OUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(INFRARED_DETECT_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(INFRARED_DETECT_PORT, INFRARED_L_OUT_PIN,SET);

    HAL_GPIO_WritePin(INFRARED_DETECT_PORT, INFRARED_R_OUT_PIN,SET);

    //ir pwr
    GPIO_InitStruct.Pin  = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(INFRARED_DETECT_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}



void infrared_detect_init()
{
    m_detect_side = get_infrared_detect_side();
    infrared_detect_io_pin_init();
    pwm_tim_init();
    return;
}

int  infrared_detect_handler()
{
    m_detect_side = get_infrared_detect_side();
    if ( m_detect_side == 0 )
    {
        return 1;
    }
    if (!m_detect_start )
    {
        IFRARED_IN_PWR_ON_OFF(1);
        m_detect_start = TRUE;
        shock_sensor_interrupt_disable();
    }

    if  ( ++m_detect_send_time >=  IR_MAX_SEND_NUM ) {
        m_detect_send_time       = 0;
        m_detect_left_rcv_count  = 0;
        m_detect_right_rcv_count = 0;
        m_detect_start           = FALSE;
        IFRARED_IN_PWR_ON_OFF(0);
        shock_sensor_interrupt_enable();
        HAL_TIM_Base_Stop(&TimHandle);
        return 1;
    }

    HAL_TIM_Base_Start_IT(&TimHandle);
    HAL_Delay(50);
    HAL_TIM_Base_Stop(&TimHandle);
    return 0;
}





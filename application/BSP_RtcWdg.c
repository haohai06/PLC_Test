/**-----------------------------------------�ļ���Ϣ---------------------------------------------
**��          ��: ʱ�ӡ����Ź����ܵ�ʵ���ļ�
**---------------------------------------��ʷ�汾��Ϣ-------------------------------------------
**��    ��    ��:
**��          ��:
**��          ��:
**��          ��:
**----------------------------------------------------------------------------------------------
***********************************************************************************************/

/* Includes-----------------------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

#include "BSP_RtcWdg.h"

/* Private define-----------------------------------------------------------------------------*/
/* Private typedef----------------------------------------------------------------------------*/

/* Private const------------------------------------------------------------------------------*/
/* Private variables--------------------------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

static uint32_t rtc_clock_tick_sec = 0;
static uint32_t system_tick_sec = 0;

static Datetime_t clockDateTime;

//APP_TIMER_DEF(m_tisk_timer_id);

/* Functions --------------------------------------------------------------------------*/

/***********************************************************************************************
SECTION: REAL CLOCK BEGIN
***********************************************************************************************/


void tisk_timer_handler(void * p_context)
{
    rtc_clock_tick_sec ++;
    system_tick_sec++;
}

/*************************************
*��������: BSP_RtcInit
*�������ܣ�ʱ�Ӷ�дǰ�ĳ�ʼ��
*��ڲ�������
*���ڲ�������
*����ֵ�� 0: �ɹ� ��0: �������
*�汾&���ڣ�
*************************************/
uint8_t	BSP_RtcInit(void)
{
    clockDateTime.year  = 0;
    clockDateTime.month = 0;
    clockDateTime.day  = 1;
    clockDateTime.hour = 0;
    clockDateTime.min  = 0;
    clockDateTime.sec  = 0;

    //rtc_clock_tick_sec = CommonCalendarConvertSec(&clockDateTime);
    system_tick_sec = 0;

    //app_timer_create(&m_tisk_timer_id,APP_TIMER_MODE_REPEATED,tisk_timer_handler);
    //app_timer_start(m_tisk_timer_id, APP_TIMER_TICKS(1000), NULL);


    return TRUE;
}

/*************************************
*��������: BSP_RtcRead
*�������ܣ�ʱ�Ӷ�
*��ڲ�������
*���ڲ�����pstTime: ������ʱ��
*����ֵ�� 0: �ɹ� ��0: �������
*�汾&���ڣ�
*************************************/
uint8_t	BSP_RtcRead(void *pstTime)
{
    //CommonSecConvertCalendar(rtc_clock_tick_sec,pstTime);
    return TRUE;
}

/*************************************
*��������: BSP_RtcWrite
*�������ܣ�ʱ��д
*��ڲ�����stTime: д���ʱ��
*���ڲ�������
*����ֵ�� 0: �ɹ� ��0: �������
*�汾&���ڣ�
*************************************/
uint8_t	BSP_RtcWrite (void *pstTime)
{
    //rtc_clock_tick_sec = CommonCalendarConvertSec(pstTime);
    return TRUE;
}

/*************************************
*��������: BSP_RtcReadSecond
*�������ܣ�ʱ�Ӷ���
*��ڲ�������
*���ڲ�������
*����ֵ��
*�汾&���ڣ�
*************************************/
uint32_t BSP_RtcReadSecond(void)
{
    return rtc_clock_tick_sec;
}

/*************************************
*��������: BSP_RtcReadSytemSecond
*�������ܣ�ϵͳ���Ӽ�ʱ�����ϵ翪����ʱ
*��ڲ�������
*���ڲ�������
*����ֵ��
*�汾&���ڣ�
*************************************/
uint32_t BSP_RtcReadSytemSecond(void)
{
    return system_tick_sec;
}

/*************************************
*��������: BSP_RtcReadSubSecond
*�������ܣ�ʱ�Ӷ�΢��
*��ڲ�������
*���ڲ�����΢��
*����ֵ��
*�汾&���ڣ�
*************************************/
uint16_t BSP_RtcReadSubSecond(void)
{
    return 0;
}

/*************************************
*��������: BSP_RtcSetWakeUp
*�������ܣ�����RTC�������ڣ�ʹ��RTC�жϻ���
*��ڲ�������
*���ڲ�����΢��
*����ֵ��
*�汾&���ڣ�
*************************************/
uint8_t BSP_RtcSetWakeUp(uint16_t peroid)
{
    return TRUE;
}

/***********************************************************************************************
SECTION: REAL CLOCK END
***********************************************************************************************/
//nrf_drv_wdt_channel_id m_channel_id;

void wdt_event_handler(void)
{
    // BSP_Log_Print(".........");
}

/*************************************
*��������: BSP_WdgInit
*�������ܣ����Ź��ĳ�ʼ��
*��ڲ�������
*���ڲ�������
*����ֵ�� 0: �ɹ� ��0: �������
*�汾&���ڣ�
*************************************/
uint8_t BSP_WdgInit(void)
{
#ifndef  _SYS_DEBUG_

    uint8_t err_code;

    // Configure WDT.
//    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
//    config.reload_value = 5000; // delay 5s
//    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
//    APP_ERROR_CHECK(err_code);
//    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
//    APP_ERROR_CHECK(err_code);

#endif

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        //Error_Handler();
    }

    return TRUE;
}

/*************************************
*��������: BSP_WdgEnable
*�������ܣ����ÿ��Ź�
*��ڲ�������
*���ڲ�������
*����ֵ�� 0: �ɹ� ��0: �������
*�汾&���ڣ�
*************************************/
uint8_t BSP_WdgEnable(void)
{
#ifndef  _SYS_DEBUG_

    //nrf_drv_wdt_enable();

#endif

    return TRUE;
}

/*************************************
*��������: BSP_WdgFeed
*�������ܣ����Ź�ι��
*��ڲ�������
*���ڲ�������
*����ֵ�� 0: �ɹ� ��0: �������
*�汾&���ڣ�
*************************************/
uint8_t BSP_WdgFeed(void)
{
#ifndef  _SYS_DEBUG_
    // nrf_drv_wdt_channel_feed(m_channel_id);
#endif
    HAL_IWDG_Refresh(&hiwdg);
    return TRUE;
}

/************************************************************************************************/

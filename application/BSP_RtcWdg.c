/**-----------------------------------------文件信息---------------------------------------------
**描          述: 时钟、看门狗功能的实现文件
**---------------------------------------历史版本信息-------------------------------------------
**修    改    人:
**日          期:
**版          本:
**描          述:
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
*函数名称: BSP_RtcInit
*函数功能：时钟读写前的初始化
*入口参数：无
*出口参数：无
*返回值： 0: 成功 非0: 错误代码
*版本&日期：
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
*函数名称: BSP_RtcRead
*函数功能：时钟读
*入口参数：无
*出口参数：pstTime: 读出的时间
*返回值： 0: 成功 非0: 错误代码
*版本&日期：
*************************************/
uint8_t	BSP_RtcRead(void *pstTime)
{
    //CommonSecConvertCalendar(rtc_clock_tick_sec,pstTime);
    return TRUE;
}

/*************************************
*函数名称: BSP_RtcWrite
*函数功能：时钟写
*入口参数：stTime: 写入的时间
*出口参数：无
*返回值： 0: 成功 非0: 错误代码
*版本&日期：
*************************************/
uint8_t	BSP_RtcWrite (void *pstTime)
{
    //rtc_clock_tick_sec = CommonCalendarConvertSec(pstTime);
    return TRUE;
}

/*************************************
*函数名称: BSP_RtcReadSecond
*函数功能：时钟读秒
*入口参数：无
*出口参数：秒
*返回值：
*版本&日期：
*************************************/
uint32_t BSP_RtcReadSecond(void)
{
    return rtc_clock_tick_sec;
}

/*************************************
*函数名称: BSP_RtcReadSytemSecond
*函数功能：系统秒钟计时器，上电开锁计时
*入口参数：无
*出口参数：秒
*返回值：
*版本&日期：
*************************************/
uint32_t BSP_RtcReadSytemSecond(void)
{
    return system_tick_sec;
}

/*************************************
*函数名称: BSP_RtcReadSubSecond
*函数功能：时钟读微秒
*入口参数：无
*出口参数：微秒
*返回值：
*版本&日期：
*************************************/
uint16_t BSP_RtcReadSubSecond(void)
{
    return 0;
}

/*************************************
*函数名称: BSP_RtcSetWakeUp
*函数功能：设置RTC唤醒周期，使能RTC中断唤醒
*入口参数：无
*出口参数：微秒
*返回值：
*版本&日期：
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
*函数名称: BSP_WdgInit
*函数功能：看门狗的初始化
*入口参数：无
*出口参数：无
*返回值： 0: 成功 非0: 错误代码
*版本&日期：
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
*函数名称: BSP_WdgEnable
*函数功能：启用看门狗
*入口参数：无
*出口参数：无
*返回值： 0: 成功 非0: 错误代码
*版本&日期：
*************************************/
uint8_t BSP_WdgEnable(void)
{
#ifndef  _SYS_DEBUG_

    //nrf_drv_wdt_enable();

#endif

    return TRUE;
}

/*************************************
*函数名称: BSP_WdgFeed
*函数功能：看门狗喂狗
*入口参数：无
*出口参数：无
*返回值： 0: 成功 非0: 错误代码
*版本&日期：
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

#include "stm32f0xx.h"
#include "drv_rtc.h"
//#include "osal.h"
#include "slave_lock_task.h"

//extern uint8_t slave_lock_task_id;
RTC_HandleTypeDef RtcHandle;
IWDG_HandleTypeDef hiwdg;
//#define RTC_ASYNCH_PREDIV  0x7F
//#define RTC_SYNCH_PREDIV   0x00FF

//RTC=40k 40000/39+1/99+1 = 10Hz
#define RTC_ASYNCH_PREDIV  39
#define RTC_SYNCH_PREDIV   99

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    // drv_rtc_set_alarm_IT(1);
    //slave_lock_wdg_active();
    wakeup_timeout_handler();
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{

    if(hrtc->Instance==RTC)
    {
        /* USER CODE BEGIN RTC_MspInit 0 */

        /* USER CODE END RTC_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_RTC_ENABLE();
        /* RTC interrupt Init */
        HAL_NVIC_SetPriority(RTC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(RTC_IRQn);
        /* USER CODE BEGIN RTC_MspInit 1 */

        /* USER CODE END RTC_MspInit 1 */
    }

}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{

    if(hrtc->Instance==RTC)
    {
        /* USER CODE BEGIN RTC_MspDeInit 0 */

        /* USER CODE END RTC_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_RTC_DISABLE();

        /* RTC interrupt DeInit */
        HAL_NVIC_DisableIRQ(RTC_IRQn);
        /* USER CODE BEGIN RTC_MspDeInit 1 */

        /* USER CODE END RTC_MspDeInit 1 */
    }

}

void drv_rtc_init(void)
{
    RTC_DateTypeDef  sdatestructure;
    RTC_TimeTypeDef  stimestructure;

    RtcHandle.Instance = RTC;
    RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
    RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;

    HAL_RTC_Init(&RtcHandle);

    sdatestructure.Year = 0x19;
    sdatestructure.Month = RTC_MONTH_FEBRUARY;
    sdatestructure.Date = 0x08;
    sdatestructure.WeekDay = RTC_WEEKDAY_SATURDAY;

    if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        //Error_Handler();
    }
    /*##-2- Configure the Time #################################################*/
    /* Set Time: 02:20:00 */
    stimestructure.Hours = 0x19;
    stimestructure.Minutes = 0x28;
    stimestructure.Seconds = 0x00;
    stimestructure.TimeFormat = RTC_HOURFORMAT_24;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BCD) != HAL_OK)
    {
        /* Initialization Error */
        // Error_Handler();
    }
    //HAL_RTC_MspInit(&RtcHandle);
    //drv_rtc_set_alarm_IT(5);
}

void RTC_read_time(void)
{

    //RTC_TimeTypeDef  stimestructure;
    RTC_DateTypeDef  sdatestructure;

    //HAL_RTC_GetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN);
    //printf("%02d:%02d:%02d",stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds);
}


void drv_rtc_set_alarm_IT(uint32_t sec)
{
    RTC_AlarmTypeDef sAlarm;
    uint32_t counter_alarm = 0;
    uint32_t counter_time = 0;
    RTC_TimeTypeDef stime = {0};

    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY|RTC_ALARMMASK_HOURS;
                       //|RTC_ALARMMASK_MINUTES;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;

    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_A;

    HAL_RTC_GetTime(&RtcHandle,&stime, RTC_FORMAT_BIN);
    counter_time = (uint32_t)(((uint32_t)stime.Hours * 3600) +
                              ((uint32_t)stime.Minutes * 60) + ((uint32_t)stime.Seconds));

    counter_alarm =  counter_time + sec;

    sAlarm.Alarm = RTC_ALARM_A;
    sAlarm.AlarmTime.Hours   = (uint32_t)((counter_alarm / 3600) % 24);
    sAlarm.AlarmTime.Minutes = (uint32_t)((counter_alarm % 3600) / 60);
    sAlarm.AlarmTime.Seconds = (uint32_t)((counter_alarm % 3600) % 60);

    RtcHandle.Instance->ISR = 0x00000000;
    HAL_RTC_SetAlarm_IT(&RtcHandle,&sAlarm,RTC_FORMAT_BIN);
    RTC_read_time();
}


/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
void MX_IWDG_Init(void)
{

    /* USER CODE BEGIN IWDG_Init 0 */

    /* USER CODE END IWDG_Init 0 */

    /* USER CODE BEGIN IWDG_Init 1 */

    /* USER CODE END IWDG_Init 1 */
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_128;  //4096/(40k/128) = 13.1072
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        // Error_Handler();
    }
    /* USER CODE BEGIN IWDG_Init 2 */

    /* USER CODE END IWDG_Init 2 */

}

void BSP_WdgFeed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

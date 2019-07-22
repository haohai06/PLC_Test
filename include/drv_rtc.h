#ifndef __DRV_RTC_H__

#define __DRV_RTC_H__

void drv_rtc_init(void);

void drv_rtc_set_alarm_IT(uint32_t sec);
void RTC_TimeShow(uint8_t* showtime);

void RTC_read_time(void);
void MX_IWDG_Init(void);
void BSP_WdgFeed(void);
#endif /*__DRV_RTC_H__*/



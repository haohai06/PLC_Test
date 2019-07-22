
#ifndef HAL_BOARD_H
#define HAL_BOARD_H

#include "hardware.h"
#include "hal_type.h"


#define INT_HEAP_LEN  2048
#define MAXMEMHEAP	  INT_HEAP_LEN

#define st(x)      do { x } while (__LINE__ == -1)

#define HAL_ENTER_CRITICAL_SECTION(x)	__disable_irq()
#define HAL_EXIT_CRITICAL_SECTION(x)    __enable_irq()

//#define HAL_ENTER_CRITICAL_SECTION(x)	//do{ asm("cpsid i "); }while(0)
//#define HAL_EXIT_CRITICAL_SECTION(x)   // do{ asm("cpsie i "); }while(0)

#define HAL_DISABLE_INTERRUPTS()  //do{ asm("cpsid i "); }while(0)
#define HAL_ENABLE_INTERRUPTS()   //do{ asm("cpsie i "); }while(0)

#define HAL_ASSERT(expr)       do{if(!expr) while(1); }while(0)

#define OSAL_TIM_TICK               1 // 1ms

#define OSAL_TIMERS_MAX_TIMEOUT 	10800000  // 3*3600*1000 MS
#define OSAL_MAX_SLEEP_TIME			3600000	  // 36000000 // 1 hour ms
#define OSAL_MIN_SLEEP_TIME			100 		// 100 ms 

#define OSAL_SET_CPU_BEFORE_SLEEP()	    Hal_CPU_Before_Sleep()
#define OSAL_SET_CPU_BEFORE_WAKEUP()	Hal_CPU_Before_Wakeup()

#define OSAL_SET_CPU_INTO_SLEEP(sleepTime)	Hal_CPU_IntoSleep(sleepTime)

void halTimerInit(void);

void halTimerTickUpdate(void);

void  halTimerTickIntDisable(void);

void  halTimerTickIntEnable(void);

uint32_t macMcuPrecisionCount(void);

uint32_t halTimerTickGet(void);

void halTimerDelay(uint32_t delay);

void halDelayus(uint32_t us);



#endif


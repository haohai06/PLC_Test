
#include "hal_type.h"
#include "hal_board.h"
#include "main.h"

static uint32_t sysCounttick = 0;

void halTimerInit(void)
{
}

void halTimerTickUpdate(void)
{
    sysCounttick++;
}

void  halTimerTickIntDisable(void)
{

}

void  halTimerTickIntEnable(void)
{

}

uint32_t macMcuPrecisionCount(void)
{
  
    sysCounttick = HAL_GetTick();
    
	return sysCounttick;
}

uint32_t halTimerTickGet(void)
{
	return sysCounttick;
}

void halTimerDelay(uint32_t delay)
{
  //halDelayus(delay*1000 );
#if 1
	uint32_t tickstart = 0U;
	
	//tickstart = halTimerTickGet();
	tickstart = macMcuPrecisionCount();

	while ( ( macMcuPrecisionCount() - tickstart) < delay )
	{

	}
#endif
}
//__asm void nop(void)
//{
//    NOP
//	//asm("nop");
//}
void halDelayus(uint32_t us)
{
	while(us--)
	{	
//	asm("nop");
//	asm("nop");
//	asm("nop");
//	asm("nop");
//	asm("nop");
//	asm("nop");
//	asm("nop");
//nop();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();		
		
	}
}





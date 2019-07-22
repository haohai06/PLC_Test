/**************************************************************************************************
Filename:       OSAL_pwrmgr.c
Revised:        $Date: 2014-11-21 16:17:37 -0800 (Fri, 21 Nov 2014) $
Revision:       $Revision: 41218 $

Description:    This file contains the OSAL Power Management API.


Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
* INCLUDES
*/

#include "hal_type.h"
#include "hal_board.h"
#include "OSAL.h"
#include "OSAL_Task.h"
#include "OSAL_Timers.h"
#include "OSAL_PwrMgr.h"

/*********************************************************************
* MACROS
*/

/*********************************************************************
* CONSTANTS
*/

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

/* This global variable stores the power management attributes.
*/
pwrmgr_attribute_t pwrmgr_attribute;
/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/

/*********************************************************************
* LOCAL FUNCTION PROTOTYPES
*/

/*********************************************************************
* FUNCTIONS
*********************************************************************/

/*********************************************************************
* @fn      osal_pwrmgr_init
*
* @brief   Initialize the power management system.
*
* @param   none.
*
* @return  none.
*/
void osal_pwrmgr_init(void)
{
	pwrmgr_attribute.pwrmgr_device  = PWRMGR_BATTERY; // Default to no power conservation.
	pwrmgr_attribute.pwrmgr_task_state = 0;            // Cleared.  All set to conserve
}

/*********************************************************************
* @fn      osal_pwrmgr_device
*
* @brief   Sets the device power characteristic.
*
* @param   pwrmgr_device - type of power devices. With PWRMGR_ALWAYS_ON
*          selection, there is no power savings and the device is most
*          likely on mains power. The PWRMGR_BATTERY selection allows the
*          HAL sleep manager to enter sleep.
*
* @return  none
*/
void osal_pwrmgr_device(uint8_t pwrmgr_device)
{
	pwrmgr_attribute.pwrmgr_device = pwrmgr_device;
}

/*********************************************************************
* @fn      osal_pwrmgr_task_state
*
* @brief   This function is called by each task to state whether or
*          not this task wants to conserve power.
*
* @param   task_id - calling task ID.
*          state - whether the calling task wants to
*          conserve power or not.
*
* @return  SUCCESS if task complete
*/
uint8_t osal_pwrmgr_task_state(uint8_t task_id, uint8_t state)
{
	halIntState_t intState;

	if (task_id >= tasksCnt)
		return (INVALID_TASK);

	HAL_ENTER_CRITICAL_SECTION(intState);

	if (state == PWRMGR_CONSERVE)
	{
		// Clear the task state flag
		pwrmgr_attribute.pwrmgr_task_state &= ~(1 << task_id);
	}
	else
	{
		// Set the task state flag
		pwrmgr_attribute.pwrmgr_task_state |= (1 << task_id);
	}

	HAL_EXIT_CRITICAL_SECTION(intState);

	return (SUCCESS);
}

/*********************************************************************
* @fn      osal_pwrmgr_powerconserve
*
* @brief   This function is called from the main OSAL loop when there are
*          no events scheduled and shouldn't be called from anywhere else.
*
* @param   none.
*
* @return  none.
*/
void osal_pwrmgr_powerconserve(void)
{
    uint32_t  next = 0;
    uint32_t  sleepms = 0;
    RTC_TimeTypeDef stimestructureget;////test
	halIntState_t intState;
	extern RTC_HandleTypeDef RtcHandle;
	// Should we even look into power conservation
	if (pwrmgr_attribute.pwrmgr_device != PWRMGR_ALWAYS_ON)
	{
      // Are all tasks in agreement to conserve
      if (pwrmgr_attribute.pwrmgr_task_state == 0 )
      {
        // Hold off interrupts.
        HAL_ENTER_CRITICAL_SECTION(intState);

        // Get next time-out
        next = osal_next_timeout();

        // Re-enable interrupts.
        HAL_EXIT_CRITICAL_SECTION(intState);

        if( next == 0 )
          	next = OSAL_TIMERS_MAX_TIMEOUT;
          
        if( next < OSAL_MIN_SLEEP_TIME )
          return;
		
 		//printf("next=%d",next);
        if( OSAL_SET_CPU_BEFORE_SLEEP())
        {

          // Put the processor into sleep mode
			sleepms = OSAL_SET_CPU_INTO_SLEEP(next);

            OSAL_SET_CPU_BEFORE_WAKEUP();

          //HAL_DEBUG_PRINT_STR("%u,%u\r\n",next,sleepms);

			osal_adjust_timers(sleepms);
			//printf("a2=%d",next);
        }
      }		
	}
}
/*********************************************************************
*********************************************************************/

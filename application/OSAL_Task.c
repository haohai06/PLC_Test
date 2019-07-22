/**************************************************************************************************
  Filename:       OSAL_Task.c
  Revised:        $Date: 2012-02-12 17:37:02 -0800 (Sun, 12 Feb 2012) $
  Revision:       $Revision: 29221 $

  Description:    This file contains the OSAL Task create/manipulate functions.


  Copyright 2010-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hal_type.h"
#include "hal_board.h"
#include "OSAL.h"
#include "OSAL_Task.h"
#include "OSAL_Memory.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_Clock.h"

#include "slave_lock_task.h"
#include "lock_app_uart.h"

/*********************************************************************
* GLOBAL VARIABLES
*/

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] = {
    slave_lock_process_event,
    uart_task_process_event,
};

const uint8_t tasksCnt = sizeof(tasksArr) / sizeof(tasksArr[0]);
uint16_t *tasksEvents;

/*********************************************************************
* FUNCTIONS
*********************************************************************/

/*********************************************************************
* @fn      osalInitTasks
*
* @brief   This function invokes the initialization function for each task.
*
* @param   void
*
* @return  none
*/
void osalInitTasks(void)
{
    uint8_t taskID = 0;

    tasksEvents = (uint16_t *)osal_mem_alloc(sizeof(uint16_t)* tasksCnt);

    osal_memset(tasksEvents, 0, (sizeof(uint16_t)* tasksCnt));
	 	
    slave_lock_task_init(taskID++);	

	uart_task_event_init(taskID++);
}


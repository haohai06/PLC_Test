/**************************************************************************************************
Filename:       type.h
Revised:        $Date:
Revision:       $Revision: 23160 $

Description:    Type definitions and macros.

Copyright 2004-2008 Texas Instruments Incorporated. All rights reserved.
**************************************************************************************************/

#ifndef _TYPE_H
#define _TYPE_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
* INCLUDES
*/

#include <stdio.h>
#include <stdint.h>
  
/*********************************************************************
* Lint Keywords
*/
#ifndef VOID
#define VOID (void)
#endif

/*********************************************************************
* CONSTANTS
*/

#ifndef false
#define false 0
#endif

#ifndef true
#define true 1
#endif

#ifndef CONST
#define CONST const
#endif

#ifndef GENERIC
#define GENERIC
#endif

#ifndef NULL
#define NULL 0
#endif
/*********************************************************************
* TYPEDEFS
*/

//typedef unsigned char bool;

//typedef unsigned char uint8_t;
//typedef unsigned short uint16_t;
//typedef unsigned int uint32_t;

//typedef char int8_t;
//typedef short int16_t;
//typedef int int32_t;

typedef int32_t uint24_t;

// Generic Status return
typedef uint8_t Status_t;

typedef uint8_t halIntState_t;

typedef uint32_t halDataAlign_t;

/*********************************************************************
* MACROS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

/*********************************************************************
* FUNCTIONS
*/

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* COMDEF_H */

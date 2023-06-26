// Copyright (c) 2023 Ziga Miklosic
// All Rights Reserved
////////////////////////////////////////////////////////////////////////////////
/**
*@file      can.h
*@brief     CAN LL drivers based on STM32 HAL library
*@author    Ziga Miklosic
*@email		ziga.miklosic@gmail.si
*@date      26.06.2023
*@version   V0.1.0
*/
////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup CAN_API
* @{ <!-- BEGIN GROUP -->
*
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef __CAN_H
#define __CAN_H

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 *  Module version
 */
#define CAN_VER_MAJOR          ( 0 )
#define CAN_VER_MINOR          ( 1 )
#define CAN_VER_DEVELOP        ( 0 )

/**
 *  CAN status
 */
typedef enum
{
    eCAN_OK        = 0x00U,    /**<Normal operation */
    eCAN_ERROR     = 0x01U,    /**<General error code */
} can_status_t;

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
can_status_t can_init     (void);

#endif // __CAN_H

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

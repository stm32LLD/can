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

#include "../../can_cfg.h"

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
    eCAN_OK         = 0x00U,    /**<Normal operation */
    eCAN_ERROR      = 0x01U,    /**<General error code */
    eCAN_WAR_EMPTY  = 0x02U,    /**<Buffer empty warning */
    eCAN_WAR_FULL   = 0x04U,    /**<Buffer full warning */
} can_status_t;

/**
 *  CAN DLC - Data Lenght Code
 */
typedef enum
{
    eCAN_DLC_0  = 0,    /**< 0 bytes data field */
    eCAN_DLC_1,         /**< 1 bytes data field */
    eCAN_DLC_2,         /**< 2 bytes data field */
    eCAN_DLC_3,         /**< 3 bytes data field */
    eCAN_DLC_4,         /**< 4 bytes data field */
    eCAN_DLC_5,         /**< 5 bytes data field */
    eCAN_DLC_6,         /**< 6 bytes data field */
    eCAN_DLC_7,         /**< 7 bytes data field */
    eCAN_DLC_8,         /**< 8 bytes data field */
    eCAN_DLC_12,        /**< 12 bytes data field */
    eCAN_DLC_16,        /**< 16 bytes data field */
    eCAN_DLC_20,        /**< 20 bytes data field */
    eCAN_DLC_24,        /**< 24 bytes data field */
    eCAN_DLC_32,        /**< 32 bytes data field */
    eCAN_DLC_48,        /**< 48 bytes data field */
    eCAN_DLC_64,        /**< 64 bytes data field */
} can_dlc_opt_t;

/**
 *  CAN Message
 *
 *  sizeof(can_msg_t) = 72 bytes
 */
typedef struct
{
    uint32_t        id;         /**<CAN message ID */
    uint8_t         data[64];   /**<Data field */
    can_dlc_opt_t   dlc;        /**<Data Length Code */
    bool            fd;         /**<FD message */
} can_msg_t;

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
can_status_t can_init       (const can_ch_t can_ch);
can_status_t can_deinit     (const can_ch_t can_ch);
can_status_t can_is_init    (const can_ch_t can_ch, bool * const p_is_init);
can_status_t can_transmit   (const can_ch_t can_ch, const can_msg_t * const p_msg);
can_status_t can_receive    (const can_ch_t can_ch, can_msg_t * const p_msg);

#endif // __CAN_H

////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

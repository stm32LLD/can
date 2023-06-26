// Copyright (c) 2023 Ziga Miklosic
// All Rights Reserved
////////////////////////////////////////////////////////////////////////////////
/**
*@file      can.h
*@brief     CAN LL drivers based on STM32 HAL library
*@author    Ziga Miklosic
*@email     ziga.miklosic@gmail.si
*@date      26.06.2023
*@version   V0.1.0
*/
////////////////////////////////////////////////////////////////////////////////
/*!
* @addtogroup IWDT
* @{ <!-- BEGIN GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Includes
////////////////////////////////////////////////////////////////////////////////
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "can.h"
#include "../../can_cfg.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Function prototypes
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/**
*@addtogroup CAN_API
* @{ <!-- BEGIN GROUP -->
*
* 	Following function are part of CAN LL driver API.
*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        CAN initialization
*
* @return       status - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////



can_status_t can_init(const can_ch_t can_ch)
{
    can_status_t status = eCAN_OK;

    (void) can_ch;

    return status;
}


/*
can_status_t can_deinit(const can_ch_t can_ch)
{
    can_status_t status = eCAN_OK;


    return status;
}


can_status_t can_is_init(const can_ch_t can_ch, bool * const p_is_init)
{
    can_status_t status = eCAN_OK;


    return status;
}


can_status_t can_transmit(const can_ch_t can_ch, const uint8_t * const p_data, const uint32_t size)
{
    can_status_t status = eCAN_OK;


    return status;
}


can_status_t can_receive(const can_ch_t can_ch, uint8_t * const p_data)
{
    can_status_t status = eCAN_OK;


    return status;
}
*/





////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////
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
#include <assert.h>

#include "can.h"
#include "../../can_cfg.h"

// Fifo
#include "middleware/ring_buffer/src/ring_buffer.h"

// Gpio
#include "drivers/peripheral/gpio/gpio/src/gpio.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/**
 *  Compatibility check with RING_BUFFER
 *
 *  Support version V2.0.x
 */
_Static_assert( 2 == RING_BUFFER_VER_MAJOR );
_Static_assert( 0 >= RING_BUFFER_VER_MINOR );

/**
 *  CAN control
 */
typedef struct
{
    FDCAN_HandleTypeDef handle;         /**<FDCAN handler */
    p_ring_buffer_t     tx_buf;         /**<Transmission buffer */
    p_ring_buffer_t     rx_buf;         /**<Reception buffer */
    bool                is_init;        /**<Initialization flag */
} can_ctrl_t;

/**
 *  FIFO buffer attributes
 */
static const ring_buffer_attr_t g_buf_attr =
{
   .item_size   = sizeof( uint8_t ),    // Byte size
   .override    = false,                // Do not lost data
   .p_mem       = NULL,                 // Dynamically allocate
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

/**
 *  CAN control block
 */
static can_ctrl_t g_can[eCAN_CH_NUM_OF] = {0};


////////////////////////////////////////////////////////////////////////////////
// Function prototypes
////////////////////////////////////////////////////////////////////////////////
static can_status_t can_init_fifo       (const can_ch_t can_ch, const uint32_t tx_size, const uint32_t rx_size);
static void         can_enable_clock    (const FDCAN_GlobalTypeDef * p_inst);
static void         can_disable_clock   (const FDCAN_GlobalTypeDef * p_inst);
static void         can_init_gpio       (const can_pin_cfg_t * const p_pin_cfg);
static void         can_deinit_gpio     (const can_pin_cfg_t * const p_pin_cfg);

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Initialize CAN FIFO buffers
*
* @param[in]    can_ch      - CAN communication channel
* @param[in]    tx_size     - Size of TX FIFO
* @param[in]    rx_size     - Size of RX FIFO
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static can_status_t can_init_fifo(const can_ch_t can_ch, const uint32_t tx_size, const uint32_t rx_size)
{
    can_status_t            status      = eCAN_OK;
    ring_buffer_status_t    fifo_status = eRING_BUFFER_OK;

    // Init FIFO
    fifo_status |= ring_buffer_init( &g_can[ can_ch ].rx_buf, rx_size, &g_buf_attr );
    fifo_status |= ring_buffer_init( &g_can[ can_ch ].tx_buf, tx_size, &g_buf_attr );

    // Init Rx FIFO
    if ( eRING_BUFFER_OK != fifo_status )
    {
        status = eCAN_ERROR;

        CAN_ASSERT( 0 );
    }

    return status;
}


////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Enable CAN clock
*
* @param[in]    p_inst  - CAN peripheral instance
* @return       status  - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void can_enable_clock(const FDCAN_GlobalTypeDef * p_inst)
{
#if defined(FDCAN1)
    if ( FDCAN1 == p_inst )
    {
        __HAL_RCC_FDCAN_CLK_ENABLE();
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Disable CAN clock
*
* @param[in]    p_inst  - CAN peripheral instance
* @return       status  - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void can_disable_clock(const FDCAN_GlobalTypeDef * p_inst)
{
#if defined(FDCAN1)
    if ( FDCAN1 == p_inst )
    {
        __HAL_RCC_FDCAN_CLK_DISABLE();
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Init CAN pin
*
* @param[in]    p_pin_cfg   - Pointer to pin configuration
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void can_init_gpio(const can_pin_cfg_t * const p_pin_cfg)
{
    GPIO_InitTypeDef gpio_init = {0};

    // Enable clock
    gpio_enable_port_clock( p_pin_cfg->p_port );

    // Prepare gpio init structure
    gpio_init.Pin        = p_pin_cfg->pin;
    gpio_init.Mode       = GPIO_MODE_AF_PP;
    gpio_init.Speed      = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Pull       = p_pin_cfg->pull;
    gpio_init.Alternate  = p_pin_cfg->af;

    // Init gpio
    HAL_GPIO_Init( p_pin_cfg->p_port, &gpio_init );
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        De-Init CAN pin
*
* @param[in]    p_pin_cfg   - Pointer to pin configuration
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static void can_deinit_gpio(const can_pin_cfg_t * const p_pin_cfg)
{
    // Init gpio
    HAL_GPIO_DeInit( p_pin_cfg->p_port, p_pin_cfg->pin );
}




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
* @brief        Initialize CAN
*
* @param[in]    can_ch  - CAN communication channel
* @return       status  - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
can_status_t can_init(const can_ch_t can_ch)
{
    can_status_t status = eCAN_OK;

    CAN_ASSERT( can_ch < eCAN_CH_NUM_OF );

    if ( can_ch < eCAN_CH_NUM_OF )
    {
        if  ( false == g_can[can_ch].is_init )
        {
            // Get CAN configurations
            const can_cfg_t * p_can_cfg = can_cfg_get_config( can_ch );

            // Init FIFO
            status |= can_init_fifo( can_ch, p_can_cfg->tx_buf_size, p_can_cfg->rx_buf_size );

            // Init GPIOs
            can_init_gpio( &( p_can_cfg->tx_pin ));
            can_init_gpio( &( p_can_cfg->rx_pin ));

            // Enable clock
            can_enable_clock( p_can_cfg->p_instance );

            // Prepare HAL init structure
            g_can[can_ch].handle.Instance                   = p_can_cfg->p_instance;
            g_can[can_ch].handle.Init.ClockDivider          = FDCAN_CLOCK_DIV1;
            g_can[can_ch].handle.Init.FrameFormat           = FDCAN_FRAME_FD_BRS;
            g_can[can_ch].handle.Init.Mode                  = FDCAN_MODE_NORMAL;
            g_can[can_ch].handle.Init.AutoRetransmission    = DISABLE;
            g_can[can_ch].handle.Init.TransmitPause         = DISABLE;
            g_can[can_ch].handle.Init.ProtocolException     = DISABLE;

            // Nominal baudrate settgins
            g_can[can_ch].handle.Init.NominalPrescaler      = p_can_cfg->baud_nor.prescaler;
            g_can[can_ch].handle.Init.NominalSyncJumpWidth  = p_can_cfg->baud_nor.sjw;
            g_can[can_ch].handle.Init.NominalTimeSeg1       = p_can_cfg->baud_nor.seg1;
            g_can[can_ch].handle.Init.NominalTimeSeg2       = p_can_cfg->baud_nor.seg2;

            // Data baudrate settigns - applicable only in FD mode
            g_can[can_ch].handle.Init.DataPrescaler         = p_can_cfg->baud_data.prescaler;
            g_can[can_ch].handle.Init.DataSyncJumpWidth     = p_can_cfg->baud_data.sjw;
            g_can[can_ch].handle.Init.DataTimeSeg1          = p_can_cfg->baud_data.seg1;
            g_can[can_ch].handle.Init.DataTimeSeg2          = p_can_cfg->baud_data.seg2;

            g_can[can_ch].handle.Init.StdFiltersNbr         = 0;
            g_can[can_ch].handle.Init.ExtFiltersNbr         = 0;
            g_can[can_ch].handle.Init.TxFifoQueueMode       = FDCAN_TX_FIFO_OPERATION;

            // Init CAN
            if ( HAL_OK != HAL_FDCAN_Init( &g_can[can_ch].handle ))
            {
                status = eCAN_ERROR;
            }

            // Init success
            if ( eCAN_OK == status )
            {
                // Enable reception buffer not empty interrupt
                __HAL_FDCAN_ENABLE_IT( &g_can[can_ch].handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE );

                // Error interrupt (error passive, error active & bus-off)
                // TODO: CHeck for that!!
                //__HAL_FDCAN_ENABLE_IT( &g_can[can_ch].handle, ( FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF ));

                // Setup UART interrupt priority and enable it
                NVIC_SetPriority( p_can_cfg->irq_num, p_can_cfg->irq_prio );
                NVIC_EnableIRQ( p_can_cfg->irq_num );

                // Init success
                g_can[can_ch].is_init = true;
            }
        }
    }
    else
    {
        status = eCAN_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        De-Initialize CAN
*
* @param[in]    can_ch      - CAN communication channel
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
can_status_t can_deinit(const can_ch_t can_ch)
{
    can_status_t status = eCAN_OK;

    CAN_ASSERT( can_ch < eCAN_CH_NUM_OF );

    if ( can_ch < eCAN_CH_NUM_OF )
    {
        if  ( true == g_can[can_ch].is_init )
        {
            // Get CAN configurations
            const can_cfg_t * p_can_cfg = can_cfg_get_config( can_ch );

            // De-init gpios
            can_deinit_gpio( &( p_can_cfg->tx_pin ));
            can_deinit_gpio( &( p_can_cfg->rx_pin ));

            // Disable clock
            can_disable_clock( p_can_cfg->p_instance );

            // De-Init success
            if ( eCAN_OK == status )
            {
                g_can[can_ch].is_init = false;
            }
        }
    }
    else
    {
        status = eCAN_ERROR;
    }

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Get CAN initialization flag
*
* @param[in]    can_ch      - CAN communication channel
* @param[out]   p_is_init   - Pointer to init flag
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
can_status_t can_is_init(const can_ch_t can_ch, bool * const p_is_init)
{
    can_status_t status = eCAN_OK;

    CAN_ASSERT( can_ch < eCAN_CH_NUM_OF );
    CAN_ASSERT( NULL != p_is_init );

    if (    ( can_ch < eCAN_CH_NUM_OF )
        &&  ( NULL != p_is_init ))
    {
        *p_is_init = g_can[can_ch].is_init;
    }
    else
    {
        status = eCAN_ERROR;
    }

    return status;
}


/*


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

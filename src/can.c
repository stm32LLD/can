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
   .item_size   = sizeof( can_msg_t ),  // CAN Message size
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
static can_status_t     can_init_fifo       (const can_ch_t can_ch, const uint32_t tx_size, const uint32_t rx_size);
static void             can_enable_clock    (const FDCAN_GlobalTypeDef * p_inst);
static void             can_disable_clock   (const FDCAN_GlobalTypeDef * p_inst);
static void             can_init_gpio       (const can_pin_cfg_t * const p_pin_cfg);
static void             can_deinit_gpio     (const can_pin_cfg_t * const p_pin_cfg);
static inline bool      can_find_channel    (const FDCAN_GlobalTypeDef * p_inst, can_ch_t * const p_ch);
static inline void      can_process_isr     (const FDCAN_GlobalTypeDef * p_inst);
static can_dlc_opt_t    can_dlc_to_real     (const uint32_t dlc_raw);

//static uint32_t         can_dlc_to_raw      (const can_dlc_opt_t dlt_opt);


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
/*!
* @brief        Find CAN channel by hardware instance
*
* @param[in]    p_inst      - CAN periphery (FDCAN1, FDCAN2,...)
* @param[out]   p_opt       - CAN enumeration option
* @return       status      - Status of operation
*/
////////////////////////////////////////////////////////////////////////////////
static inline bool can_find_channel(const FDCAN_GlobalTypeDef * p_inst, can_ch_t * const p_ch)
{
    bool found = false;

    for ( uint8_t ch = 0U; ch < eCAN_CH_NUM_OF; ch++ )
    {
        // Get CAN configurations
        const can_cfg_t * p_can_cfg = can_cfg_get_config( ch );

        if ( p_inst == p_can_cfg->p_instance )
        {
            found = true;
            *p_ch = ch;
            break;
        }
    }

    return found;
}

////////////////////////////////////////////////////////////////////////////////
/*!
* @brief        Process CAN ISR
*
* @param[in]    p_inst     - CAN periphery (FDCAN1, FDCAN2,...)
* @return       void
*/
////////////////////////////////////////////////////////////////////////////////
static inline void can_process_isr(const FDCAN_GlobalTypeDef * p_inst)
{
    can_ch_t                can_ch  = 0;
    FDCAN_RxHeaderTypeDef   header  = {0};
    can_msg_t               can_msg = {0};

    // Find CAN channel by hardware instance
    if ( true == can_find_channel( p_inst, &can_ch ))
    {
        // New message in FIFO0
        if ( __HAL_FDCAN_GET_FLAG( &g_can[can_ch].handle, FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE ))
        {
            // Clear flag
            __HAL_FDCAN_CLEAR_FLAG( &g_can[can_ch].handle, FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE );

            // Get Rx CAN message
            HAL_FDCAN_GetRxMessage( &g_can[can_ch].handle, FDCAN_RX_FIFO0, &header, (uint8_t*) &can_msg.data );

            // Assemble CAN message
            can_msg.id  = header.Identifier;
            can_msg.dlc = can_dlc_to_real( header.DataLength );
            can_msg.fd  = ( FDCAN_CLASSIC_CAN == header.FDFormat ) ? false : true;

            // Put to Rx fifo
            (void) ring_buffer_add( g_can[can_ch].rx_buf, (can_msg_t*) &can_msg );
        }
    }
}


static uint32_t gu32_dlc_map[eCAN_DLC_NUM_OF] =
{
        [eCAN_DLC_0]    = FDCAN_DLC_BYTES_0,
        [eCAN_DLC_1]    = FDCAN_DLC_BYTES_1,
        [eCAN_DLC_2]    = FDCAN_DLC_BYTES_2,
        [eCAN_DLC_3]    = FDCAN_DLC_BYTES_3,
        [eCAN_DLC_4]    = FDCAN_DLC_BYTES_4,
        [eCAN_DLC_5]    = FDCAN_DLC_BYTES_5,
        [eCAN_DLC_6]    = FDCAN_DLC_BYTES_6,
        [eCAN_DLC_7]    = FDCAN_DLC_BYTES_7,
        [eCAN_DLC_8]    = FDCAN_DLC_BYTES_8,
        [eCAN_DLC_12]   = FDCAN_DLC_BYTES_12,
        [eCAN_DLC_16]   = FDCAN_DLC_BYTES_16,
        [eCAN_DLC_20]   = FDCAN_DLC_BYTES_20,
        [eCAN_DLC_24]   = FDCAN_DLC_BYTES_24,
        [eCAN_DLC_32]   = FDCAN_DLC_BYTES_32,
        [eCAN_DLC_48]   = FDCAN_DLC_BYTES_48,
        [eCAN_DLC_64]   = FDCAN_DLC_BYTES_64,
};



static can_dlc_opt_t can_dlc_to_real(const uint32_t dlc_raw)
{
    can_dlc_opt_t dcl = eCAN_DLC_0;

    for ( can_dlc_opt_t opt = eCAN_DLC_0; opt < eCAN_DLC_NUM_OF; opt++ )
    {
        if ( dlc_raw == gu32_dlc_map[opt] )
        {
            dcl = opt;
            break;
        }
    }

    return dcl;
}

/*
static uint32_t can_dlc_to_raw(const can_dlc_opt_t dlt_opt)
{
    return gu32_dlc_map[dlt_opt];
}
*/


#if defined(FDCAN1)
    ////////////////////////////////////////////////////////////////////////////////
    /*!
    * @brief        FDCAN 1 ISR
    *
    * @return       void
    */
    ////////////////////////////////////////////////////////////////////////////////
    void FDCAN1_IT0_IRQHandler(void)
    {
        can_process_isr( FDCAN1 );
    }
#endif

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
            g_can[can_ch].handle.Init.Mode                  = FDCAN_MODE_INTERNAL_LOOPBACK;     // TODO: Only testing! Make back to: FDCAN_MODE_NORMAL;
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
                //__HAL_FDCAN_ENABLE_IT( &g_can[can_ch].handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE );

                HAL_FDCAN_ActivateNotification( &g_can[can_ch].handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0 );

                // Error interrupt (error passive, error active & bus-off)
                // TODO: CHeck for that!!
                //__HAL_FDCAN_ENABLE_IT( &g_can[can_ch].handle, ( FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF ));

                // Setup UART interrupt priority and enable it
                NVIC_SetPriority( p_can_cfg->irq_num, p_can_cfg->irq_prio );
                NVIC_EnableIRQ( p_can_cfg->irq_num );

                // Init success
                g_can[can_ch].is_init = true;

                // Start
                HAL_FDCAN_Start( &g_can[can_ch].handle );
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



can_status_t can_transmit(const can_ch_t can_ch, const can_msg_t * const p_msg)
{
    can_status_t    status  = eCAN_OK;
    can_msg_t       can_msg = {0};

    CAN_ASSERT( can_ch < eCAN_CH_NUM_OF );
    CAN_ASSERT( true == g_can[can_ch].is_init );
    CAN_ASSERT( NULL != p_msg );

    if ( can_ch < eCAN_CH_NUM_OF )
    {
        if (    ( true == g_can[can_ch].is_init )
            &&  ( NULL != p_msg ))
        {
            if ( p_msg-> dlc < eCAN_DLC_NUM_OF )
            {
                // Copy can message
                memcpy( &can_msg, p_msg, sizeof( can_msg_t ));

                // Enter critical
                __disable_irq();

                // FIFO full
                if ( eRING_BUFFER_OK != ring_buffer_add( g_can[can_ch].tx_buf, (can_msg_t*) &can_msg ))
                {
                    status = eCAN_WAR_FULL;
                }
                else
                {
                    // Raise TX empty IRQ
                    // NOTE: Later in irq message is being transmitted
                    __HAL_FDCAN_ENABLE_IT( &g_can[can_ch].handle, FDCAN_IT_TX_FIFO_EMPTY );
                }

                // Exit critical
                __enable_irq();
            }
            else
            {
                status = eCAN_ERROR;
            }
        }
        else
        {
            status = eCAN_ERROR;
        }
    }
    else
    {
        status = eCAN_ERROR;
    }

    return status;
}


can_status_t can_receive(const can_ch_t can_ch, can_msg_t * const p_msg)
{
    can_status_t    status  = eCAN_OK;

    CAN_ASSERT( can_ch < eCAN_CH_NUM_OF );
    CAN_ASSERT( true == g_can[can_ch].is_init );
    CAN_ASSERT( NULL != p_msg );

    if ( can_ch < eCAN_CH_NUM_OF )
    {
        if  (   ( true == g_can[can_ch].is_init )
            &&  ( NULL != p_msg ))
        {
            // Get data from RX FIFO
            if ( eRING_BUFFER_OK != ring_buffer_get( g_can[can_ch].rx_buf, (can_msg_t*) p_msg ))
            {
                status = eCAN_WAR_EMPTY;
            }
        }
        else
        {
            status = eCAN_ERROR;
        }
    }
    else
    {
        status = eCAN_ERROR;
    }


    return status;
}


#if 0

can_status_t can_transmit(const can_ch_t can_ch, const uint8_t * const p_data, const uint32_t size)
{
    can_status_t    status          = eCAN_OK;

/*    (void) size;
    (void) p_data;

    static uint8_t data[8] = { 0, 1, 2, 3, 4, 5, 6, 0 };

    FDCAN_TxHeaderTypeDef msg_header =
    {
        .Identifier             = 0x222,
        .IdType                 = FDCAN_STANDARD_ID,
        .TxFrameType            = FDCAN_DATA_FRAME,
        .DataLength             = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator    = FDCAN_ESI_ACTIVE,
        .BitRateSwitch          = FDCAN_BRS_OFF,
        .FDFormat               = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl     = FDCAN_NO_TX_EVENTS,
        .MessageMarker          = 0,
    };

    data[0]++;
    data[7]++;

    HAL_FDCAN_AddMessageToTxFifoQ( &g_can[can_ch].handle, (FDCAN_TxHeaderTypeDef*) &msg_header, (uint8_t*) &data );

    msg_header.Identifier = 0x333;
    HAL_FDCAN_AddMessageToTxFifoQ( &g_can[can_ch].handle, (FDCAN_TxHeaderTypeDef*) &msg_header, (uint8_t*) &data );

    msg_header.Identifier = 0x444;
    HAL_FDCAN_AddMessageToTxFifoQ( &g_can[can_ch].handle, (FDCAN_TxHeaderTypeDef*) &msg_header, (uint8_t*) &data );

    msg_header.Identifier = 0x555;
    HAL_FDCAN_AddMessageToTxFifoQ( &g_can[can_ch].handle, (FDCAN_TxHeaderTypeDef*) &msg_header, (uint8_t*) &data );*/


    uint32_t        buf_free_space  = 0U;

    CAN_ASSERT( uart_ch < eCAN_CH_NUM_OF );
    CAN_ASSERT( true == g_can[can_ch].is_init );
    CAN_ASSERT( NULL != p_data );

    if ( can_ch < eCAN_CH_NUM_OF )
    {
        if  (   ( true == g_can[can_ch].is_init )
            &&  ( NULL != p_data ))
        {
            // Enter critical
            __disable_irq();

            // Check if there is space in Tx FIFO
            (void) ring_buffer_get_free( g_can[can_ch].tx_buf, &buf_free_space );

            // There is space in Tx FIFO for complete message
            if ( size <= buf_free_space )
            {
                // Put all data to Tx FIFO
                for ( uint32_t byte_idx = 0; byte_idx < size; byte_idx++ )
                {
                    (void) ring_buffer_add( g_can[can_ch].tx_buf, (uint8_t*) &p_data[byte_idx] );
                }

                // Raise TX empty IRQ
                // NOTE: Later in irq message is being transmitted
                __HAL_FDCAN_ENABLE_IT( &g_can[can_ch].handle, FDCAN_IT_TX_FIFO_EMPTY );
            }

            // No space in Tx FIFO
            else
            {
                status = eUART_WAR_FULL;
            }

            // Exit critical
            __enable_irq();
        }
        else
        {
            status = eUART_ERROR;
        }
    }
    else
    {
        status = eUART_ERROR;
    }

    return status;
}


can_status_t can_receive(const can_ch_t can_ch, uint8_t * const p_data)
{
    can_status_t status = eCAN_OK;

    (void) can_ch;
    (void) p_data;

    return status;
}

#endif




////////////////////////////////////////////////////////////////////////////////
/**
* @} <!-- END GROUP -->
*/
////////////////////////////////////////////////////////////////////////////////

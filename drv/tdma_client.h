#ifndef __TDMA_H
#define __TDMA_H

/**
 * @defgroup    drv_tdma      TDMA radio driver
 * @ingroup     drv
 * @brief       Driver for Time-Division-Multiple-Access fot the DotBot radio
 *
 * @{
 * @file
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @copyright Inria, 2024
 * @}
 */

#include <stdint.h>
#include <stdlib.h>
#include <nrf.h>
#include "gpio.h"
#include "radio.h"

//=========================== defines ==========================================

///< TDMA internal registrarion state
typedef enum {
    DB_TDMA_UNREGISTERED,       ///< the DotBot is not registered with the gateway
    DB_TDMA_REGISTERED,         ///< the DotBot registered with the gateway
} db_tdma_registration_state_t;

///< TDMA internal RX state
typedef enum {
    DB_TDMA_RX_WAIT,       ///< the DotBot receiver is OFF
    DB_TDMA_RX_ON,         ///< the DotBot is receiving radio packets
} db_tdma_rx_state_t;


//=========================== variables ========================================

/// Data type to store the TDMA table
typedef struct __attribute__((packed)){
    uint32_t frame;        ///< Duration of the entire TDMA frame [microseconds]
    uint32_t rx_start;     ///< Time between the start of the frame and when the gateway starts transmitting
    uint16_t rx_duration;  ///< Duration the gateway will transmit messages
    uint32_t tx_start;     ///< Time between the start of the frame and the start of the DotBot's alloted frame
    uint16_t tx_duration;  ///< Duration of the DotBot's alloted frame.
} tdma_client_table_t;

typedef void (*tdma_client_cb_t)(uint8_t *packet, uint8_t length);  ///< Function pointer to the callback function called on packet receive

//=========================== prototypes ==========================================

/**
 * @brief Initializes the TDMA scheme
 *
 * Starts advertising registration packets, set a default tdma table
 * and innits the radio
 *
 * @param[in] callback      pointer to a function that will be called each time a packet is received.
 * @param[in] radio_mode    BLE mode used by the radio (1MBit, 2MBit, LR125KBit, LR500Kbit)
 * @param[in] freq          Frequency of the radio [0, 100]
 * @param[in] buffer_size   Number of messages that can be queued for sending via radio.
 *
 */
void db_tdma_client_init(tdma_client_cb_t callback, db_radio_ble_mode_t radio_mode, uint8_t radio_freq, uint8_t buffer_size);

/**
 * @brief Updates the RX and TX timings for the TDMA table
 *
 * @param[in] table       New table of TDMA timings
 */
void db_tdma_client_update_table(tdma_client_table_t *table);

/**
 * @brief Queues a single packet to send through the Radio
 *
 * @param[in] packet pointer to the array of data to send over the radio (max size = 32)
 * @param[in] length Number of bytes to send (max size = 32)
 *
 */
void db_tdma_client_tx(const uint8_t *packet, uint8_t length);

/**
 * @brief Ignore TDMA table and send all pending packets inmediatly
 *
 */
void db_tdma_client_flush(void);

/**
 * @brief Earese all pending packets in the TDMA queue
 *
 */
void db_tdma_client_empty(void);

#endif
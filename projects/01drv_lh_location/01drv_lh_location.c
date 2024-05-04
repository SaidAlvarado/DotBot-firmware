/**
 * @file
 * @ingroup samples_drv
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @brief This is a short example of how to calculate the position of the DotBot using the 1LH_2D algorithm.
 *

 *
 * @date 2024
 *
 * @copyright Inria, 2024
 *
 */
#include <nrf.h>
#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "board_config.h"
#include "lh2.h"
#include "radio.h"
#include "protocol.h"
#include "lh_location.h"


// TODO:
// - Ignore all messages except the calibration one.
// - Continuosly send Processed LH2 data until you receive the calibration packet.
//    - Maybe make a getter that retrieves the last received LH2 packet (per basestation). and a status code whether you already got it, or no.q
//    - Make a getter that returns an enum UNCALIBRATED error when you try to fetch your current location from the lh_loc driver
// - Then start sending XYZ position

//=========================== defines ==========================================

#define DB_BUFFER_MAX_BYTES       (255U)   ///< Max bytes in UART receive buffer

typedef struct {
    db_lh2_t                 lh2;                                ///< LH2 device descriptor
    uint8_t                  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    protocol_lh2_location_t  last_location;                      ///< Last computed LH2 location received
    uint64_t                 device_id;                          ///< Device ID of the DotBot
} lh_loc_vars_t;

//=========================== variables ========================================

static lh_loc_vars_t _lh_loc_vars;

//=========================== prototype ========================================

static void radio_callback(uint8_t *pkt, uint8_t len);

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    // Initialize the board core features (voltage regulator)
    db_board_init();

    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_rx();              // Start receiving packets.


    // Initialize the LH2
    db_lh2_init(&_lh_loc_vars.lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_start();

    while (1) {
        // wait until something happens e.g. an SPI interrupt
        __WFE();

        // the location function has to be running all the time
        db_lh2_process_location(&_lh2);
    }

    // one last instruction, doesn't do anything, it's just to have a place to put a breakpoint.
    __NOP();
}

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;


    uint8_t           *ptk_ptr           = pkt;
    protocol_header_t *header            = (protocol_header_t *)ptk_ptr;
    // Check destination address matches
    if (header->dst != DB_BROADCAST_ADDRESS && header->dst != _dotbot_vars.device_id) {
        return;
    }

    // Check version is supported
    if (header->version != DB_FIRMWARE_VERSION) {
        return;
    }

    // Check application is compatible
    if (header->application != DotBot) {
        return;
    }

    uint8_t *cmd_ptr = ptk_ptr + sizeof(protocol_header_t);
    // parse received packet and update the motors' speeds
    switch (header->type) {
        case DB_PROTOCOL_CMD_MOVE_RAW:
        {
            protocol_move_raw_command_t *command = (protocol_move_raw_command_t *)cmd_ptr;
            int16_t                      left    = (int16_t)(100 * ((float)command->left_y / INT8_MAX));
            int16_t                      right   = (int16_t)(100 * ((float)command->right_y / INT8_MAX));
            db_motors_set_speed(left, right);
        } break;
        case DB_PROTOCOL_CMD_RGB_LED:
        {
            protocol_rgbled_command_t *command = (protocol_rgbled_command_t *)cmd_ptr;
            db_rgbled_pwm_set_color(command->r, command->g, command->b);
        } break;
        case DB_PROTOCOL_LH2_LOCATION:
        {
            const protocol_lh2_location_t *location = (const protocol_lh2_location_t *)cmd_ptr;
            int16_t                        angle    = -1000;
            _compute_angle(location, &_dotbot_vars.last_location, &angle);
            if (angle != DB_DIRECTION_INVALID) {
                _dotbot_vars.last_location.x = location->x;
                _dotbot_vars.last_location.y = location->y;
                _dotbot_vars.last_location.z = location->z;
                _dotbot_vars.direction       = angle;
            }
            _dotbot_vars.update_control_loop = (_dotbot_vars.control_mode == ControlAuto);
        } break;
        case DB_PROTOCOL_CONTROL_MODE:
            db_motors_set_speed(0, 0);
            break;
        case DB_PROTOCOL_LH2_WAYPOINTS:
        {
            db_motors_set_speed(0, 0);
            _dotbot_vars.control_mode        = ControlManual;
            _dotbot_vars.waypoints.length    = (uint8_t)*cmd_ptr++;
            _dotbot_vars.waypoints_threshold = (uint32_t)((uint8_t)*cmd_ptr++ * 1000);
            memcpy(&_dotbot_vars.waypoints.points, cmd_ptr, _dotbot_vars.waypoints.length * sizeof(protocol_lh2_location_t));
            _dotbot_vars.next_waypoint_idx = 0;
            if (_dotbot_vars.waypoints.length > 0) {
                _dotbot_vars.control_mode = ControlAuto;
            }
        } break;
        default:
            break;
    }
}
/**
 * @file
 * @defgroup project_dotbot    DotBot application
 * @ingroup projects
 * @brief This is the radio-controlled DotBot app
 *
 * The remote control can be either a keyboard, a joystick or buttons on the gateway
 * itself
 *
 * @author Said Alvarado-Marin <said-alexander.alvarado-marin@inria.fr>
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 */

#include <nrf.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
// Include BSP headers
#include "board.h"
#include "board_config.h"
#include "device.h"
#include "lh2.h"
#include "protocol.h"
#include "motors.h"
#include "radio.h"
#include "rgbled_pwm.h"
#include "timer.h"
#include "log_flash.h"
#include "qdec.h"
// Include drivers for the motors
#include "pid.h"

//=========================== defines ==========================================

#define DB_LH2_UPDATE_DELAY_MS    (100U)   ///< 100ms delay between each LH2 data refresh
#define DB_ADVERTIZEMENT_DELAY_MS (500U)   ///< 500ms delay between each advertizement packet sending
#define DB_TIMEOUT_CHECK_DELAY_MS (200U)   ///< 200ms delay between each timeout delay check
#define TIMEOUT_CHECK_DELAY_TICKS (17000)  ///< ~500 ms delay between packet received timeout checks
#define DB_LH2_COUNTER_MASK       (0x07)   ///< Maximum number of lh2 iterations without value received
#define DB_BUFFER_MAX_BYTES       (255U)   ///< Max bytes in UART receive buffer
#define DB_DIRECTION_THRESHOLD    (0.01)   ///< Threshold to update the direction
#define DB_DIRECTION_INVALID      (-1000)  ///< Invalid angle e.g out of [0, 360] range
#define DB_MAX_SPEED              (60)     ///< Max speed in autonomous control mode
#if defined(BOARD_DOTBOT_V2)
#define DB_REDUCE_SPEED_FACTOR  (0.7)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE   (25)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SPEED_FACTOR (35)   ///< Constant applied to the normalized angle to target error
#define DB_ANGULAR_SIDE_FACTOR  (-1)   ///< Angular side factor
#else                                  // BOARD_DOTBOT_V1
#define DB_REDUCE_SPEED_FACTOR  (0.9)  ///< Reduction factor applied to speed when close to target or error angle is too large
#define DB_REDUCE_SPEED_ANGLE   (20)   ///< Max angle amplitude where speed reduction factor is applied
#define DB_ANGULAR_SPEED_FACTOR (30)   ///< Constant applied to the normalized angle to target error
#define DB_ANGULAR_SIDE_FACTOR  (1)    ///< Angular side factor
#endif

// QDEC defnintions
#define QDEC_LEFT  0
#define QDEC_RIGHT 1
// PID definitions
#define PID_SAMPLE_TIME_MS  10    ///< time between PID calculations
#define DB_L_CM             10.0  ///< distance between wheels (cm)
#define DB_WHEEL_D_CM       6.0   ///< Diameter of the wheels
#define DB_WHEEL_CPR        12.0  ///< counts per resolution of the quadrature encoder
#define DB_MOTOR_GEAR_RATIO 50.0  ///< Gear ratio of the DC motor

typedef struct {
    bool left_overflow;
    bool right_overflow;
} qdec_vars_t;

// Local variables
typedef struct {
    uint32_t                 ts_last_packet_received;            ///< Last timestamp in microseconds a control packet was received
    db_lh2_t                 lh2;                                ///< LH2 device descriptor
    uint8_t                  radio_buffer[DB_BUFFER_MAX_BYTES];  ///< Internal buffer that contains the command to send (from buttons)
    protocol_lh2_location_t  last_location;                      ///< Last computed LH2 location received
    int16_t                  direction;                          ///< Current direction of the DotBot (angle in °)
    protocol_control_mode_t  control_mode;                       ///< Remote control mode
    protocol_lh2_waypoints_t waypoints;                          ///< List of waypoints
    uint32_t                 waypoints_threshold;                ///< Distance to target waypoint threshold
    uint8_t                  next_waypoint_idx;                  ///< Index of next waypoint to reach
    bool                     update_control_loop;                ///< Whether the control loop need an update
    bool                     advertize;                          ///< Whether an advertize packet should be sent
    bool                     update_lh2;                         ///< Whether LH2 data must be processed
    uint8_t                  lh2_update_counter;                 ///< Counter used to track when lh2 data were received and to determine if an advertizement packet is needed
    uint64_t                 device_id;                          ///< Device ID of the DotBot
    db_log_dotbot_data_t     log_data;
    uint32_t                 update_pid;
} dotbot_vars_t;

//=========================== variables ========================================

static dotbot_vars_t _dotbot_vars;

#ifdef DB_RGB_LED_PWM_RED_PORT  // Only available on DotBot v2
static const db_rgbled_pwm_conf_t rgbled_pwm_conf = {
    .pwm  = 1,
    .pins = {
        { .port = DB_RGB_LED_PWM_RED_PORT, .pin = DB_RGB_LED_PWM_RED_PIN },
        { .port = DB_RGB_LED_PWM_GREEN_PORT, .pin = DB_RGB_LED_PWM_GREEN_PIN },
        { .port = DB_RGB_LED_PWM_BLUE_PORT, .pin = DB_RGB_LED_PWM_BLUE_PIN },
    }
};
#endif

static const qdec_conf_t qdec_left = {
    .pin_a = &db_qdec_left_a_pin,
    .pin_b = &db_qdec_left_b_pin,
};

static const qdec_conf_t qdec_right = {
    .pin_a = &db_qdec_right_a_pin,
    .pin_b = &db_qdec_right_b_pin,
};

static qdec_vars_t _qdec_vars = {
    .left_overflow  = false,
    .right_overflow = false,
};

static pid_t             _pid_left   = { 0 };
static pid_t             _pid_right  = { 0 };
static const pid_gains_t _pid_params = {
    .kp = 3,
    .ki = 3.5,
    .kd = 0,
};

//=========================== prototypes =======================================

// static void _timeout_check(void);
static void _advertise(void);
static void _compute_angle(const protocol_lh2_location_t *next, const protocol_lh2_location_t *origin, int16_t *angle);
static void _update_control_loop(void);
static void _update_lh2(void);
static void _update_pid(void);

//=========================== callbacks ========================================

static void radio_callback(uint8_t *pkt, uint8_t len) {
    (void)len;

    _dotbot_vars.ts_last_packet_received = db_timer_ticks();
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
            // Convert to forward speed and angular speed (or Y axis and X axis)
            float joy_y = (left + right) / 2;
            float joy_x = (left - right) / 2;
            // Add dead zones and convert the range to cm/s

            // Convert Y-axis between the ranges [5 - 40]cm/s
            if (joy_y >= 15.) {
                joy_y = 10 +  (joy_y - 10) * (70 - 10) / (100 - 10);
            } else if (joy_y <= -15) {
                joy_y = -10 +  (joy_y + 10) * (-70 + 10) / (-100 + 30);
            } else {
                joy_y = 0.0;
            }
            // Convert X-axis between the ranges [PI/4 - 2*PI]rad/s
            if (joy_x >= 10.) {
                joy_x = M_PI_4 + 7 * M_PI * (joy_x + 20) / 240;
            } else if (joy_x <= -10) {
                joy_x = -M_PI_4 + 7 * M_PI * (joy_x - 20) / 240;
            } else {
                joy_x = 0.0;
            }



            // Convert  angular speed to cm/s
            joy_x *= DB_L_CM / 2;

            // Convert back to left and right speeds
            // db_motors_set_speed(left, right);
            _pid_left.target  = joy_y + joy_x;
            _pid_right.target = joy_y - joy_x;
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
            // db_motors_set_speed(0, 0);
            _pid_left.target  = 0.0;
            _pid_right.target = 0.0;
            break;
        case DB_PROTOCOL_LH2_WAYPOINTS:
        {
            // db_motors_set_speed(0, 0);
            _pid_left.target  = 0.0;
            _pid_right.target = 0.0;

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

static void qdec_callback(void *ctx) {
    *(bool *)ctx = true;
}

//=========================== main =============================================

/**
 *  @brief The program starts executing here.
 */
int main(void) {
    db_board_init();
#ifdef ENABLE_DOTBOT_LOG_DATA
    db_log_flash_init(LOG_DATA_DOTBOT);
#endif
    db_protocol_init();
#ifdef DB_RGB_LED_PWM_RED_PORT
    db_rgbled_pwm_init(&rgbled_pwm_conf);
#endif

    // Setup the QDEC + PID
    db_qdec_init(QDEC_LEFT, &qdec_left, qdec_callback, (void *)&_qdec_vars.left_overflow);
    db_qdec_init(QDEC_RIGHT, &qdec_right, qdec_callback, (void *)&_qdec_vars.right_overflow);
    db_pid_init(&_pid_left, 0.0, 0.0,
                _pid_params.kp, _pid_params.ki, _pid_params.kd,
                -100.0, 100.0, PID_SAMPLE_TIME_MS, DB_PID_MODE_AUTO, DB_PID_DIRECTION_DIRECT);
    db_pid_init(&_pid_right, 0.0, 0.0,
                _pid_params.kp, _pid_params.ki, _pid_params.kd,
                -100.0, 100.0, PID_SAMPLE_TIME_MS, DB_PID_MODE_AUTO, DB_PID_DIRECTION_DIRECT);

    db_motors_init();
    db_radio_init(&radio_callback, DB_RADIO_BLE_1MBit);
    db_radio_set_frequency(8);  // Set the RX frequency to 2408 MHz.
    db_radio_rx();              // Start receiving packets.

    // Set an invalid heading since the value is unknown on startup.
    // Control loop is stopped and advertize packets are sent
    _dotbot_vars.direction           = DB_DIRECTION_INVALID;
    _dotbot_vars.update_control_loop = false;
    _dotbot_vars.advertize           = false;
    _dotbot_vars.update_lh2          = false;
    _dotbot_vars.lh2_update_counter  = 0;

    // Retrieve the device id once at startup
    _dotbot_vars.device_id = db_device_id();

    db_timer_init();
    // db_timer_set_periodic_ms(0, DB_TIMEOUT_CHECK_DELAY_MS, &_timeout_check);
    db_timer_set_periodic_ms(1, DB_ADVERTIZEMENT_DELAY_MS, &_advertise);
    db_timer_set_periodic_ms(2, DB_LH2_UPDATE_DELAY_MS, &_update_lh2);
    db_timer_set_periodic_ms(0, PID_SAMPLE_TIME_MS, &_update_pid);
    db_lh2_init(&_dotbot_vars.lh2, &db_lh2_d, &db_lh2_e);
    db_lh2_start();

    while (1) {
        __WFE();

        bool need_advertize = false;
        // Process available lighthouse data
        db_lh2_process_location(&_dotbot_vars.lh2);

        if (_dotbot_vars.update_lh2) {
            // Check if data is ready to send
            if (_dotbot_vars.lh2.data_ready[0][0] == DB_LH2_PROCESSED_DATA_AVAILABLE && _dotbot_vars.lh2.data_ready[1][0] == DB_LH2_PROCESSED_DATA_AVAILABLE) {

                db_lh2_stop();
                // Prepare the radio buffer
                db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_DOTBOT_DATA);
                memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t), &_dotbot_vars.direction, sizeof(int16_t));
                // Add the LH2 sweep
                for (uint8_t lh2_sweep_index = 0; lh2_sweep_index < LH2_SWEEP_COUNT; lh2_sweep_index++) {
                    memcpy(_dotbot_vars.radio_buffer + sizeof(protocol_header_t) + sizeof(int16_t) + lh2_sweep_index * sizeof(db_lh2_raw_data_t), &_dotbot_vars.lh2.raw_data[lh2_sweep_index][0], sizeof(db_lh2_raw_data_t));
                    // Mark the data as already sent
                    _dotbot_vars.lh2.data_ready[lh2_sweep_index][0] = DB_LH2_NO_NEW_DATA;
                }
                size_t length = sizeof(protocol_header_t) + sizeof(int16_t) + sizeof(db_lh2_raw_data_t) * LH2_SWEEP_COUNT;

                // Send the radio packet
                db_radio_disable();
                db_radio_tx(_dotbot_vars.radio_buffer, length);

                db_lh2_start();
            } else {
                _dotbot_vars.lh2_update_counter = (_dotbot_vars.lh2_update_counter + 1) & DB_LH2_COUNTER_MASK;
                need_advertize                  = (_dotbot_vars.lh2_update_counter == DB_LH2_COUNTER_MASK);
            }
            _dotbot_vars.update_lh2 = false;
        }

        if (_dotbot_vars.update_control_loop) {
            _update_control_loop();
            _dotbot_vars.update_control_loop = false;
        }

        if (_dotbot_vars.advertize && need_advertize) {
            db_protocol_header_to_buffer(_dotbot_vars.radio_buffer, DB_BROADCAST_ADDRESS, DotBot, DB_PROTOCOL_ADVERTISEMENT);
            size_t length = sizeof(protocol_header_t);
            db_radio_disable();
            db_radio_tx(_dotbot_vars.radio_buffer, length);
            _dotbot_vars.advertize = false;

            printf("left: %f, right: %f\n", _pid_left.target, _pid_right.target);
        }

        if (_dotbot_vars.update_pid) {
            // turn off flag
            _dotbot_vars.update_pid = false;
            // Update current input
            _pid_left.input  = (db_qdec_read_and_clear(QDEC_LEFT) / (DB_WHEEL_CPR * DB_MOTOR_GEAR_RATIO) * M_PI * DB_WHEEL_D_CM) / (PID_SAMPLE_TIME_MS * 0.001);   // current left  speed of the motor in cm/s
            _pid_right.input = (db_qdec_read_and_clear(QDEC_RIGHT) / (DB_WHEEL_CPR * DB_MOTOR_GEAR_RATIO) * M_PI * DB_WHEEL_D_CM) / (PID_SAMPLE_TIME_MS * 0.001);  // current right speed of the motor in cm/s
            // Run the PID
            db_pid_update(&_pid_left);
            db_pid_update(&_pid_right);

            // Get the result into the motors
            if (abs((int)_pid_left.target) < 5 && abs((int)_pid_right.target) < 5) {
                db_motors_set_speed(0, 0);

            } else {
                db_motors_set_speed((int16_t)_pid_left.output, (int16_t)_pid_right.output);
            }
        }
    }
}

//=========================== private functions ================================

static void _update_control_loop(void) {
    if (_dotbot_vars.next_waypoint_idx >= _dotbot_vars.waypoints.length) {
        db_motors_set_speed(0, 0);
        return;
    }
    float dx               = ((float)_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx].x - (float)_dotbot_vars.last_location.x) / 1e6;
    float dy               = ((float)_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx].y - (float)_dotbot_vars.last_location.y) / 1e6;
    float distanceToTarget = sqrtf(powf(dx, 2) + powf(dy, 2));

    float speedReductionFactor = 1.0;  // No reduction by default

    if ((uint32_t)(distanceToTarget * 1e6) < _dotbot_vars.waypoints_threshold * 2) {
        speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
    }

    int16_t left_speed      = 0;
    int16_t right_speed     = 0;
    int16_t angular_speed   = 0;
    int16_t angle_to_target = 0;
    int16_t error_angle     = 0;
    if ((uint32_t)(distanceToTarget * 1e6) < _dotbot_vars.waypoints_threshold) {
        // Target waypoint is reached
        _dotbot_vars.next_waypoint_idx++;
    } else if (_dotbot_vars.direction == DB_DIRECTION_INVALID) {
        // Unknown direction, just move forward a bit
        left_speed  = (int16_t)DB_MAX_SPEED * speedReductionFactor;
        right_speed = (int16_t)DB_MAX_SPEED * speedReductionFactor;
    } else {
        // compute angle to target waypoint
        _compute_angle(&_dotbot_vars.waypoints.points[_dotbot_vars.next_waypoint_idx], &_dotbot_vars.last_location, &angle_to_target);
        error_angle = angle_to_target - _dotbot_vars.direction;
        if (error_angle < -180) {
            error_angle += 360;
        } else if (error_angle > 180) {
            error_angle -= 360;
        }
        if (error_angle > DB_REDUCE_SPEED_ANGLE || error_angle < -DB_REDUCE_SPEED_ANGLE) {
            speedReductionFactor = DB_REDUCE_SPEED_FACTOR;
        }
        angular_speed = (int16_t)(((float)error_angle / 180) * DB_ANGULAR_SPEED_FACTOR);
        left_speed    = (int16_t)(((DB_MAX_SPEED * speedReductionFactor) - (angular_speed * DB_ANGULAR_SIDE_FACTOR)));
        right_speed   = (int16_t)(((DB_MAX_SPEED * speedReductionFactor) + (angular_speed * DB_ANGULAR_SIDE_FACTOR)));
        if (left_speed > DB_MAX_SPEED) {
            left_speed = DB_MAX_SPEED;
        }
        if (right_speed > DB_MAX_SPEED) {
            right_speed = DB_MAX_SPEED;
        }
    }

    db_motors_set_speed(left_speed, right_speed);

#ifdef ENABLE_DOTBOT_LOG_DATA
    // Log control loop internal data and output on flash
    _dotbot_vars.log_data.direction          = (int32_t)_dotbot_vars.direction;
    _dotbot_vars.log_data.pos_x              = _dotbot_vars.last_location.x;
    _dotbot_vars.log_data.pos_y              = _dotbot_vars.last_location.y;
    _dotbot_vars.log_data.next_waypoint_idx  = (uint16_t)_dotbot_vars.next_waypoint_idx;
    _dotbot_vars.log_data.distance_to_target = (uint32_t)(distanceToTarget * 1e6);
    _dotbot_vars.log_data.angle_to_target    = angle_to_target;
    _dotbot_vars.log_data.error_angle        = error_angle;
    _dotbot_vars.log_data.angular_speed      = angular_speed;
    _dotbot_vars.log_data.left_speed         = left_speed;
    _dotbot_vars.log_data.right_speed        = right_speed;
    db_log_flash_write(&_dotbot_vars.log_data, sizeof(db_log_dotbot_data_t));
#endif
}

static void _compute_angle(const protocol_lh2_location_t *next, const protocol_lh2_location_t *origin, int16_t *angle) {
    float dx       = ((float)next->x - (float)origin->x) / 1e6;
    float dy       = ((float)next->y - (float)origin->y) / 1e6;
    float distance = sqrtf(powf(dx, 2) + powf(dy, 2));

    if (distance < DB_DIRECTION_THRESHOLD) {
        return;
    }

    int8_t sideFactor = (dx > 0) ? -1 : 1;
    *angle            = (int16_t)(acosf(dy / distance) * 180 / M_PI) * sideFactor;
    if (*angle < 0) {
        *angle = 360 + *angle;
    }
}

// static void _timeout_check(void) {
//     uint32_t ticks = db_timer_ticks();
//     if (ticks > _dotbot_vars.ts_last_packet_received + TIMEOUT_CHECK_DELAY_TICKS) {
//         db_motors_set_speed(0, 0);
//     }
// }

static void _advertise(void) {
    _dotbot_vars.advertize = true;
}

static void _update_lh2(void) {
    _dotbot_vars.update_lh2 = true;
}

static void _update_pid(void) {
    _dotbot_vars.update_pid = true;
}

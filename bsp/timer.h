#ifndef __TIMER_H
#define __TIMER_H

/**
 * @file timer.h
 * @addtogroup BSP
 *
 * @brief  Cross-platform declaration "timer" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */

#include <stdint.h>

//=========================== defines ==========================================

typedef void(*timer_cb_t)(void);

//=========================== prototypes =======================================

void db_timer_init(void);
void db_timer_set_periodic_ms(uint8_t channel, uint32_t ms, timer_cb_t cb);
void db_timer_set_oneshot_ticks(uint8_t channel, uint32_t ticks, timer_cb_t cb);
void db_timer_set_oneshot_ms(uint8_t channel, uint32_t ms, timer_cb_t cb);
void db_timer_set_oneshot_s(uint8_t channel, uint32_t s, timer_cb_t cb);
void db_timer_delay_ticks(uint32_t ticks);
void db_timer_delay_ms(uint32_t ms);
void db_timer_delay_s(uint32_t s);

#endif
/**
 * @file rpm.c
 * @addtogroup BSP
 *
 * @brief  nRF52833-specific definition of the "rpm" bsp module.
 *
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 *
 * @copyright Inria, 2022
 */
#include <stdio.h>
#include <stdlib.h>
#include <nrf.h>

#include "rpm.h"

//=========================== defines =========================================

#ifndef RPM_LEFT_PIN
#define RPM_LEFT_PIN            (17)
#endif

#ifndef RPM_LEFT_TIMER
#define RPM_LEFT_TIMER          (NRF_TIMER0)
#endif

#ifndef RPM_LEFT_PPI_CHAN
#define RPM_LEFT_PPI_CHAN       (0)
#endif

#ifndef RPM_LEFT_GPIOTE_CHAN
#define RPM_LEFT_GPIOTE_CHAN    (0)
#endif

#ifndef RPM_RIGHT_PIN
#define RPM_RIGHT_PIN           (15)
#endif

#ifndef RPM_RIGHT_TIMER
#define RPM_RIGHT_TIMER         (NRF_TIMER1)
#endif

#ifndef RPM_RIGHT_PPI_CHAN
#define RPM_RIGHT_PPI_CHAN      (1)
#endif

#ifndef RPM_RIGHT_GPIOTE_CHAN
#define RPM_RIGHT_GPIOTE_CHAN   (1)
#endif

#ifndef RPM_RTC
#define RPM_RTC                 (NRF_RTC0)
#endif

#ifndef RPM_RTC_IRQ
#define RPM_RTC_IRQ             (RTC0_IRQn)
#endif

#ifndef RPM_RTC_ISR
#define RPM_RTC_ISR             (RTC0_IRQHandler)
#endif

/**
 * Helper macro to compute speed in cm/s
 *
 * number of cycles captured within last 10ms (one rotation is 3.77mm distance of the wheel)
 * so speed = 3.7 * cycles cm/s
 */
#define RPM_CYCLES_TO_SPEED(cycles)     (uint32_t)(37.7 * cycles / 8)

/**
 * Helper macro to compute rotation per minute
 *
 * 1 cycle corresponds to one rotation, so convert to the number of minutes, given the RTC frequency of 125ms
 */
#define RPM_CYCLES_TO_RPM(cycles)       (60 * 8 * cycles)

/**
 * Helper macro to compute rotation per second
 *
 * 1 cycle corresponds to one rotation, so convert to the number of seconds, given the RTC frequency of 125ms
 */
#define RPM_CYCLES_TO_RPS(cycles)       (8 * cycles)

//=========================== variables =========================================

/*
 * Global variables used to store cycle counts at the beginning and at the end of
 * an RTC timeframe, for each side
 */
static uint32_t last_left_counts = 0;
static uint32_t previous_left_counts = 0;
static uint32_t last_right_counts = 0;
static uint32_t previous_right_counts = 0;

//=========================== public ==========================================

/**
 * @brief Initalize the revolution counter driver
 *
 * 2 GPIOTE input pins with one timer each. Each time a magnet comes in front of the magnetic encoder,
 * GPIOTE event is triggered which clears the timer ticks counter, for each side (left and right).
 * The speed/rpm are computed by the user code on demand by capturing the timer current count, reading
 * the timer CC register and clearing the timer count. Computations are done with the `ME_TICK_TO_*`
 * constants.
 */
void db_rpm_init(void) {
    // Configure pin connected to left magnetic encoder sensor, input pullup
    NRF_P0->PIN_CNF[RPM_LEFT_PIN]               |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[RPM_LEFT_PIN]               &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_GPIOTE->CONFIG[RPM_LEFT_GPIOTE_CHAN]     = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) | \
                                                    (RPM_LEFT_PIN << GPIOTE_CONFIG_PSEL_Pos) | \
                                                    (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    // Configure pin connected to right magnetic encoder sensor, input pullup
    NRF_P0->PIN_CNF[RPM_RIGHT_PIN]              |= (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    NRF_P0->PIN_CNF[RPM_RIGHT_PIN]              &= ~(1UL << GPIO_PIN_CNF_INPUT_Pos);
    NRF_GPIOTE->CONFIG[RPM_RIGHT_GPIOTE_CHAN]    = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) | \
                                                    (RPM_RIGHT_PIN << GPIOTE_CONFIG_PSEL_Pos) | \
                                                    (GPIOTE_CONFIG_POLARITY_LoToHi  << GPIOTE_CONFIG_POLARITY_Pos);

    // Configure and clear timers
    // Timers are configured in counter mode with 32bit length
    RPM_LEFT_TIMER->MODE            = TIMER_MODE_MODE_LowPowerCounter;
    RPM_LEFT_TIMER->BITMODE         = TIMER_BITMODE_BITMODE_32Bit;
    RPM_LEFT_TIMER->INTENCLR        = 1;
    RPM_LEFT_TIMER->TASKS_CLEAR     = 1;

    RPM_RIGHT_TIMER->MODE           = TIMER_MODE_MODE_LowPowerCounter;
    RPM_RIGHT_TIMER->BITMODE        = TIMER_BITMODE_BITMODE_32Bit;
    RPM_RIGHT_TIMER->INTENCLR       = 1;
    RPM_RIGHT_TIMER->TASKS_CLEAR    = 1;

    // Configure gpiote/timer count PPI
    //   - PO.17 (left magnetic encoder) event connected to PPI channel 0 which fires Timer 0 capture 0
    //   - PO.15 (right magnetic encoder) event connected to PPI channel 1 which fires Timer 0 capture 1
    NRF_PPI->CH[RPM_LEFT_PPI_CHAN].EEP  = (uint32_t)&NRF_GPIOTE->EVENTS_IN[RPM_LEFT_GPIOTE_CHAN];
    NRF_PPI->CH[RPM_LEFT_PPI_CHAN].TEP  = (uint32_t)&RPM_LEFT_TIMER->TASKS_COUNT;
    NRF_PPI->CH[RPM_RIGHT_PPI_CHAN].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[RPM_RIGHT_GPIOTE_CHAN];
    NRF_PPI->CH[RPM_RIGHT_PPI_CHAN].TEP = (uint32_t)&RPM_RIGHT_TIMER->TASKS_COUNT;

    //Enable PPI channels
    NRF_PPI->CHENSET = (1 << RPM_RIGHT_PPI_CHAN) | (1 << RPM_LEFT_PPI_CHAN);

    // Configure RTC with 125ms fire event delay
    NRF_CLOCK->LFCLKSRC             = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->TASKS_LFCLKSTART     = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_LFCLKSTARTED  = 0;

    RPM_RTC->TASKS_STOP             = 1;
    RPM_RTC->TASKS_CLEAR            = 1;
    RPM_RTC->PRESCALER              = (1 << 12) - 1;
    RPM_RTC->CC[0]                  = 1;
    RPM_RTC->EVTENSET               = (RTC_EVTENSET_COMPARE0_Enabled << RTC_EVTENSET_COMPARE0_Pos);
    RPM_RTC->INTENSET               = (RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos);
    NVIC_EnableIRQ(RPM_RTC_IRQ);
    // Start RTC
    RPM_RTC->TASKS_START = 1;
}

static void update_counters(void) {
    previous_left_counts = last_left_counts;
    previous_right_counts = last_right_counts;
    RPM_LEFT_TIMER->TASKS_CAPTURE[0] = 1;
    RPM_RIGHT_TIMER->TASKS_CAPTURE[0] = 1;
    last_left_counts = RPM_LEFT_TIMER->CC[0];
    last_right_counts = RPM_RIGHT_TIMER->CC[0];
}

void RPM_RTC_ISR(void) {
    if (RPM_RTC->EVENTS_COMPARE[0] == 1) {
        NVIC_ClearPendingIRQ(RPM_RTC_IRQ);
        RPM_RTC->EVENTS_COMPARE[0] = 0;
        update_counters();
        RPM_RTC->TASKS_CLEAR = 1;
    }
}

static inline uint32_t _db_rpm_left_cycles(void) {
    if (last_left_counts < previous_left_counts) {
        return UINT32_MAX - previous_left_counts + last_left_counts;
    }
    return last_left_counts - previous_left_counts;
}

static inline uint32_t _db_rpm_right_cycles(void) {
    if (last_right_counts < previous_right_counts) {
        return UINT32_MAX - previous_right_counts + last_right_counts;
    }
    return last_right_counts - previous_right_counts;
}

void db_rpm_encoder_timers_start(void) {
    RPM_LEFT_TIMER->TASKS_START = 1;
    RPM_RIGHT_TIMER->TASKS_START = 1;
}

void db_rpm_encoder_timers_stop(void) {
    RPM_LEFT_TIMER->TASKS_STOP = 1;
    RPM_RIGHT_TIMER->TASKS_STOP = 1;
}

uint32_t db_rpm_get_left_rpm(void) {
    return RPM_CYCLES_TO_RPM(_db_rpm_left_cycles());
}

uint32_t db_rpm_get_right_rpm(void) {
    return RPM_CYCLES_TO_RPM(_db_rpm_right_cycles());
}

uint32_t db_rpm_get_left_rps(void) {
    return RPM_CYCLES_TO_RPS(_db_rpm_left_cycles());
}

uint32_t db_rpm_get_right_rps(void) {
    return RPM_CYCLES_TO_RPS(_db_rpm_right_cycles());
}

uint32_t db_rpm_get_left_speed(void) {
    return RPM_CYCLES_TO_SPEED(_db_rpm_left_cycles());   // cm/s
}

uint32_t db_rpm_get_right_speed(void) {
    return RPM_CYCLES_TO_SPEED(_db_rpm_right_cycles());  // cm/s
}

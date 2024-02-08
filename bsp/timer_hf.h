#ifndef __TIMER_HF_H
#define __TIMER_HF_H

/**
 * @defgroup    bsp_timer_hf    High Frequency Timer
 * @ingroup     bsp
 * @brief       High level timing functions on top of the TIMER peripheral
 *
 * @{
 * @file
 * @author Alexandre Abadie <alexandre.abadie@inria.fr>
 * @copyright Inria, 2022
 * @}
 */

#include <stdint.h>

//=========================== defines ==========================================

#if defined(NRF5340_XXAA)
#if defined(NRF_APPLICATION)
#define TIMER_HF (NRF_TIMER2_S)  ///< Backend TIMER peripheral used by the timer
#elif defined(NRF_NETWORK)
#define TIMER_HF (NRF_TIMER2_NS)  ///< Backend TIMER peripheral used by the timer
#endif
#define TIMER_HF_IRQ      (TIMER2_IRQn)        ///< IRQ corresponding to the TIMER used
#define TIMER_HF_ISR      (TIMER2_IRQHandler)  ///< ISR function handler corresponding to the TIMER used
#define TIMER_HF_CB_CHANS (TIMER2_CC_NUM - 1)  ///< Number of channels that can be used for periodic callbacks
#else
#define TIMER_HF          (NRF_TIMER4)         ///< Backend TIMER peripheral used by the timer
#define TIMER_HF_IRQ      (TIMER4_IRQn)        ///< IRQ corresponding to the TIMER used
#define TIMER_HF_ISR      (TIMER4_IRQHandler)  ///< ISR function handler corresponding to the TIMER used
#define TIMER_HF_CB_CHANS (TIMER4_CC_NUM - 1)  ///< Number of channels that can be used for periodic callbacks
#endif

typedef void (*timer_hf_cb_t)(void);  ///< Callback function prototype, it is called when the timer fires an event

//=========================== prototypes =======================================

/**
 * @brief Configure a high frequency timer with microsecond precision
 */
void db_timer_hf_init(void);

/**
 * @brief Return the current timer time in microseconds
 */
uint32_t db_timer_hf_now(void);

/**
 * @brief Set a callback to be called periodically using the high frequency timer
 *
 * @param[in] channel   TIMER channel used
 * @param[in] us        periodicity in microseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_periodic_us(uint8_t channel, uint32_t us, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of microseconds
 *
 * @param[in] channel   TIMER channel used
 * @param[in] us        delay in microseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_us(uint8_t channel, uint32_t us, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called once after an amount of milliseconds
 *
 * @param[in] channel   TIMER channel used
 * @param[in] ms        delay in milliseconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_ms(uint8_t channel, uint32_t ms, timer_hf_cb_t cb);

/**
 * @brief Set a callback to be called after an amount of seconds
 *
 * @param[in] channel   TIMER channel used
 * @param[in] s         delay in seconds
 * @param[in] cb        callback function
 */
void db_timer_hf_set_oneshot_s(uint8_t channel, uint32_t s, timer_hf_cb_t cb);

/**
 * @brief Add a delay in us using the high frequency timer
 *
 * @param[in] us    delay in us
 */
void db_timer_hf_delay_us(uint32_t us);

/**
 * @brief Add a delay in milliseconds using the high frequency timer
 *
 * @param[in] ms    delay in milliseconds
 */
void db_timer_hf_delay_ms(uint32_t ms);

/**
 * @brief Add a delay in seconds using the high frequency timer
 *
 * @param[in] s delay in seconds
 */
void db_timer_hf_delay_s(uint32_t s);

#endif

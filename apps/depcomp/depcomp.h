/*
 * Copyright (c) 2017, ETH Zurich.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Authors: Roman Lim
 *          Reto Da Forno
 */

#ifndef GLOSSY_TEST_H_
#define GLOSSY_TEST_H_

#include "depcomp_config.h"
#include "glossy.h"
#include "node-id.h"
#include "flocklab_gpio.h"
#include "linreg.h"

/**
 * \defgroup glossy-test-settings Application settings
 * @{
 */

/**
 * \brief disable debug prints, LEDs and all other unnecessary resources to minimize energy consumption
 * \remark does not affect PRINT_METRICS
 */
#define MIN_ENERGY_MODE         1

#if MIN_ENERGY_MODE
  #define ENERGEST_CONF_ON      0     /* disable energest */
#endif /* MIN_ENERGY_MODE */

/**
 * \brief use light sensor? if disabled, the INT1 pin state will be monitored instead
 */
#ifndef USE_LIGHT_SENSOR
#define USE_LIGHT_SENSOR        0
#endif /* USE_LIGHT_SENSOR */

/**
 * \brief sleep between events: after an event was detected, the nodes can skip some rounds without compromizing the latency.
 * SLEEP_BTW_EVENTS_START rounds after an event occurred, the nodes will enters sleep mode for SLEEP_BTW_EVENTS_ROUNDS rounds.
 */
#if GLOSSY_PERIOD_MS <= 50
#define SLEEP_BTW_EVENTS_ROUNDS 31
#else
#define SLEEP_BTW_EVENTS_ROUNDS 15     // sleep for x rounds (max. value is 31)
#endif /* GLOSSY_PERIOD_MS <= 50 */
#define SLEEP_BTW_EVENTS_LONG   0    // long sleep phase (max value is 31); set to 0 to disable this feature
#define SLEEP_BTW_EVENTS_START  4     // go to sleep x rounds after an event was detected (max value is 7)

/**
 * \brief Application-specific header.
 *        Default value: 0x0
 */
#define APPLICATION_HEADER      0

/**
 * \brief Maximum number of transmissions N.
 *        Default value: 5.
 */
#define N_TX_RECEIVER           N_TX
#define N_TX_INITIATOR          N_TX

/**
 * \brief Period with which a Glossy phase is scheduled.
 *        Default value: 250 ms.
 */
#define GLOSSY_PERIOD           (((uint32_t)RTIMER_SECOND * (uint32_t)GLOSSY_PERIOD_MS) / 1000)

/**
 * \brief Duration of each Glossy phase.
 *        Default value: 20 ms.
 */
#define GLOSSY_DURATION         (RTIMER_SECOND / 80)     //  12.5 ms

/**
 * \brief Guard-time at receivers.
 *        Default value: 526 us.
 */
#if COOJA
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1000)
#else
// #define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 1900)   // 526 us
#define GLOSSY_GUARD_TIME       (RTIMER_SECOND / 312)   // 3.2 ms
#endif /* COOJA */

/**
 * \brief Number of consecutive Glossy phases with successful computation of reference time required to exit from bootstrapping.
 *        Default value: 3.
 */
// 2 out of 8
#define GLOSSY_BOOTSTRAP_PERIODS 2
#define GLOSSY_BOOTSTRAP_MASK 0xff

/**
 * \brief Number of missed rounds that lead to suspension of the channel hopping mechanism.
 *        Default value: 1.
 */
#define GLOSSY_MISSES_FOR_NO_HOPPING 8

/**
 * \brief Number of missed rounds to reset all state back to bootstrap mode.
 *        Default value: 5.
 */
#define GLOSSY_MISSES_FOR_BOOTRSTRAP 10

/**
 * \brief Period during bootstrapping at receivers.
 *        It should not be an exact fraction of \link GLOSSY_INIT_PAUSE \endlink.
 *        Default value: 69.474 ms.
 */
#define GLOSSY_INIT_PAUSE       (RTIMER_SECOND / 100)   // 10ms

/**
 * \brief Duration during bootstrapping at receivers.
 *        Default value: 59.474 ms.
 */
#define GLOSSY_INIT_DURATION    (RTIMER_SECOND / 20)    //  50 ms

/**
 * \brief Guard-time during bootstrapping at receivers.
 *        Default value: 50 ms.
 */
#define GLOSSY_INIT_GUARD_TIME  (RTIMER_SECOND / 200)   //  5 ms

#define GLOSSY_LINREG_MAX_ERROR 128
#define GLOSSY_LINREG_MAX_BAD_TIMESTAMPS 3

#define FAST_LINREG_CALCULATION

/**
 * \brief Data structure used to represent flooding data.
 */
typedef struct {
	uint16_t seq_no;         /**< Sequence number, incremented by the initiator at each Glossy phase. */
	uint8_t light_state;     /**< current light state (LSB) & passed rounds since event occurred (7 bit) */
	uint8_t sleep_btw_ev;    /**< sleep duration (upper 5 bits) & countdown/#rounds until sleep starts (lower 3 bits) */
} glossy_data_struct;

#define LIGHT_STATE(g)              (((g).light_state) & 0x1)
#define PASSED_ROUNDS(g)            (((g).light_state) >> 1)
#define SLEEP_DURATION(g)           (((g).sleep_btw_ev) >> 3)
#define SET_SLEEP_CNT_DUR(g, c, d)  ((g).sleep_btw_ev = (((c) & 0x07) | ((d) << 3)))
#define IS_TIME_TO_SLEEP(g)         ((SLEEP_DURATION(g) > 0) && ((((g).sleep_btw_ev) & 0x07) == 0))
//#define INCR_PASSED_ROUNDS(g)       { if ((g).light_state < 254) { (g).light_state += 2; } }
#define DELAY_ERROR(g)              (((int8_t)(g).light_state) >> 3)
#define SET_DELAY_ERROR(g, e)       {(g).light_state = ((g).light_state & 7) + ((e) << 3);}

#define SET_EVT_COUNT(g, c)         ((g).light_state = ((g).light_state & 0xf9) | ((c << 1) & 0x06))
#define EVT_COUNT(g)                (((g).light_state & 0x06) >> 1)

#if SLEEP_BTW_EVENTS
 #define DECR_SLEEP_COUNTDOWN(g)    { if (((g).sleep_btw_ev & 0x07) > 0) { (g).sleep_btw_ev--; } }
#else
 #define DECR_SLEEP_COUNTDOWN(g)
#endif /* SLEEP_BTW_EVENTS */

// error checks
#if SLEEP_BTW_EVENTS_LONG > 31 || SLEEP_BTW_EVENTS_ROUNDS > 31
#error "SLEEP_BTW_EVENTS_LONG and SLEEP_BTW_EVENTS_ROUNDS must be < 32"
#endif

#define MAX_DELAY_ERROR 8
#define MIN_DELAY_ERROR -8

#define DELAY_ERROR_WINDOW  64

#define SKEW_MAX            10

/** @} */

/**
 * \defgroup glossy-test-defines Application internal defines
 * @{
 */

/**
 * \brief Length of data structure.
 */
#define DATA_LEN                    sizeof(glossy_data_struct)

/**
 * \brief Check if the nodeId matches the one of the initiator.
 */
#define IS_INITIATOR()              (node_id == INITIATOR_NODE_ID)

/**
 * \brief Check if Glossy is still bootstrapping.
 * \sa \link GLOSSY_BOOTSTRAP_PERIODS \endlink.
 */
#define GLOSSY_IS_BOOTSTRAPPING()   (popcount(bootstrap_sync_history & GLOSSY_BOOTSTRAP_MASK) < GLOSSY_BOOTSTRAP_PERIODS)

/**
 * \brief Check if Glossy is synchronized.
 *
 * The application assumes that a node is synchronized if it updated the reference time
 * during the last Glossy phase.
 * \sa \link is_t_ref_l_updated \endlink
 */
#define GLOSSY_IS_SYNCED()          (is_t_ref_l_updated())

/**
 * \brief Get Glossy reference time.
 * \sa \link get_t_ref_l \endlink
 */
#define GLOSSY_REFERENCE_TIME       (get_t_ref_l())

#define GLOSSY_USE_CHANNEL_HOPPING() (!((GLOSSY_IS_BOOTSTRAPPING()) || (sync_missed >= GLOSSY_MISSES_FOR_NO_HOPPING) || (skipped_rounds_since_last_sync && !get_init_phase_done())))

/** @} */

/** @} */

#endif /* GLOSSY_TEST_H_ */

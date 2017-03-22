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
 *
 */

#include "depcomp.h"
#include "lib/random.h"
#include "dev/light-sensor.h"

/**
 * \defgroup glossy-test-variables Application variables
 * @{
 */

/**
 * \defgroup glossy-test-variables-sched-sync Scheduling and synchronization variables
 * @{
 */

static glossy_data_struct glossy_data;     /**< \brief Flooding data. */
static struct rtimer rt;                   /**< \brief Rtimer used to schedule Glossy. */
static struct pt pt;                       /**< \brief Protothread used to schedule Glossy. */
static rtimer_clock_t t_ref_l_old = 0;     /**< \brief Reference time computed from the Glossy
                                                phase before the last one. \sa get_t_ref_l */
static uint16_t t_ref_l_old_round = 0;
static rtimer_clock_t t_start = 0;         /**< \brief Starting time (low-frequency clock)
                                                of the last Glossy phase. */
static int period_skew = 0;                /**< \brief Current estimation of clock skew over a period
                                                of length \link GLOSSY_PERIOD \endlink. */
static uint16_t evt_counter = 0;

/** @} */

/** @} */
/** @} */

/**
 * \defgroup glossy-test-processes Application processes and functions
 * @{
 */

/**
 * \defgroup glossy-test-print-stats Print statistics information
 * @{
 */

uint16_t rtimer_ext = 0;
uint16_t last_rtimer_low = 0;
uint8_t bad_timestamp_count = 0;
uint8_t halftick = 0;
static vht_clock_t t_first_slot;
static uint8_t rounds_to_skip = 0;
static uint8_t skipped_rounds_since_last_sync = 0;
static uint16_t tx_times_round;
static int16_t delay_error_sum;
static int16_t delay_error = 0;
static uint8_t delay_error_cnt;
static uint8_t has_bootstrapped = 0;
static uint8_t bootstrap_sync_history = 0;
static uint16_t bad_skew_cnt = 0;

uint8_t popcount(uint8_t i) {
  uint8_t count = 0;
  while (i > 0) {
    count+=(i & 0x1);
    i=i>>1;
  }
  return count;
}

void print_metrics(void)
{
  static uint16_t pkts_rcvd = 0, pkts_missed = 0, bootstrap_cnt = 0;
  
  uint8_t last_rx_ch = 255, relay_cnt = 255;
  if (get_rx_cnt()) {
    pkts_rcvd++;
    last_rx_ch = get_last_rx_channel();
  } else {
    pkts_missed++;
  }
  if (has_bootstrapped && GLOSSY_IS_BOOTSTRAPPING()) {
    bootstrap_cnt++;
  }
  if (has_bootstrapped) {
    relay_cnt = get_relay_cnt();
  }
  // most important metrics: hop count on rx (relay count), reliability (% rcvd pkts), delay (passed rounds since event occurred)
  uint16_t reliability = (uint16_t)(((uint32_t)pkts_rcvd * 10000) / (uint32_t)(pkts_rcvd + pkts_missed));
  printf("METRICS round=%u light=%u hop=%u rel=%u ch=%u boot=%u\n", glossy_data.seq_no, LIGHT_STATE(glossy_data), relay_cnt, reliability, last_rx_ch, bootstrap_cnt);
}


void update_time_measurements(void)
{
  vht_clock_t t_global;
  vht_clock_t t_local;
  PIN_CLR(FLOCKLAB_INT1);
  PIN_SET(FLOCKLAB_INT1);
  rtimer_clock_t now = RTIMER_NOW();
  vht_clock_t now_ext = 0;
  if (now > last_rtimer_low)
    now_ext = now + (((vht_clock_t)rtimer_ext) << 16);
  else // overflow occurred
    now_ext = now + (((vht_clock_t)rtimer_ext + 1) << 16);
  // vht is 32 kHz value << 7 + DCO value since last 32 kHz change (7 bits)
  t_local = ((now_ext - (now - get_t_ref_local_l())) << 7) + get_t_ref_local_h(); // local tx timestamp
  t_global = ((glossy_data.seq_no * GLOSSY_PERIOD) << 7) + get_relay_cnt() * get_tx_slot_len() + CC2420_MESSAGE_DELAY + delay_error / 2; // global tx timestamp
  delay_error = DELAY_ERROR(glossy_data);
  // subtract "half a tick" from the global time to emulate half a clock cycle for the message delay
  if (delay_error & 0x1) {
    if (delay_error > 0)
      t_global += halftick;
    else
      t_global -= halftick;
  }
  halftick = (halftick + 1) % 2;
  // sanity check
  vht_clock_t estlocal = 0;
  if (get_init_phase_done()) {
    if (tx_times_round == glossy_data.seq_no)
      estlocal = tx_times[get_relay_cnt()] + CC2420_MESSAGE_DELAY; // reuse computed timestamp
    else
      estlocal = map_to_global(t_global); // compute new timestamp
    vht_clock_t diff = estlocal > t_local?estlocal-t_local:t_local-estlocal;
    if (diff <= GLOSSY_LINREG_MAX_ERROR) {
      update_tables(t_global, t_local); // NOTE: regression in other direction
      bad_timestamp_count = 0;
    } else {
      bad_timestamp_count++;
      if (bad_timestamp_count >= GLOSSY_LINREG_MAX_BAD_TIMESTAMPS) {
        init_reg_tables();
        bad_timestamp_count = 0;
      }
    }
  } else {
    update_tables(t_global, t_local); // NOTE: regression in other direction
  }

  PIN_CLR(FLOCKLAB_INT1);
#if !MIN_ENERGY_MODE
  estlocal = map_to_global(t_global);
  printf("local %lu, est local %lu, global %lu, prev est local %lu, relaycount %u, t_h_local %u, t_l_local %u, done %u, bad %u, delay %d\n",
          t_local, estlocal, t_global, tx_times[get_relay_cnt()] + CC2420_MESSAGE_DELAY, get_relay_cnt(), get_t_ref_local_h(), get_t_ref_local_l(), get_init_phase_done(), bad_timestamp_count,delay_error);
#endif /* MIN_ENERGY_MODE */
  
}


void calc_linreg(void)
{
  if (time_mapping_available()) { // receiver time mapping for next round
#ifndef FAST_LINREG_CALCULATION
    uint16_t i;
    vht_clock_t t_global = ((glossy_data.seq_no + 1) * GLOSSY_PERIOD) << 7;
    for (i=0;i<N_MAX_TX_SLOTS;i++) {
      PIN_SET(FLOCKLAB_INT1);
      tx_times[i] = map_to_global(t_global);
  #if !MIN_ENERGY_MODE
      if (i==0) {
        printf("tx %u, %lu, %lu, %lu, %lu\n", i, t_global, tx_times[i],sum_t_ref_squared,sum_numerator);
      }
  #endif /* MIN_ENERGY_MODE */
      t_global+=get_tx_slot_len();
      PIN_CLR(FLOCKLAB_INT1);
    }
#else  /* FAST_LINREG_CALCULATION */
    PIN_SET(FLOCKLAB_INT1);
    map_n_to_global(((glossy_data.seq_no + 1) * GLOSSY_PERIOD) << 7, get_tx_slot_len(), N_MAX_TX_SLOTS, tx_times);
    PIN_CLR(FLOCKLAB_INT1);
#endif /* FAST_LINREG_CALCULATION */

    tx_times_round = glossy_data.seq_no + 1;
    set_tx_times_updated(get_init_phase_done());
    if (get_init_phase_done()) {
      t_first_slot = tx_times[0] + CC2420_MESSAGE_DELAY;
    }
  }
}


void initiator_update_delay_error(void)
{
  vht_clock_t t_local, t_global;
  t_local = (uint16_t)((get_t_ref_local_l() << 7) + get_t_ref_local_h()); // local tx timestamp
  t_global = (uint16_t)(tx_times[0] + get_relay_cnt() * get_tx_slot_len() + CC2420_MESSAGE_DELAY); // global tx 
  int16_t diff = t_local - t_global; //negative means packets are sent too early on receivers
  // t_local should be the same as t_global
  delay_error_sum += diff;
  delay_error_cnt++;
  if (delay_error_cnt == DELAY_ERROR_WINDOW) {
    if (delay_error_sum>0)
      delay_error = (delay_error_sum + delay_error_cnt/4)/ (delay_error_cnt/2);
    else
      delay_error = (delay_error_sum - delay_error_cnt/4)/ (delay_error_cnt/2);
    delay_error_cnt /= 2;
    delay_error_sum /= 2;
    // sanity check
    if (delay_error > MAX_DELAY_ERROR || delay_error < MIN_DELAY_ERROR) {
      // reset measurements
      delay_error_cnt = 0;
      delay_error_sum = 0;
      delay_error = 0;
    }
    SET_DELAY_ERROR(glossy_data, delay_error);
  }
}

void print_stats(void)
{
  static unsigned long packets_received = 0; /**< \brief Current number of received packets. */
  static unsigned long packets_missed = 0;   /**< \brief Current number of missed packets. */
  static unsigned long latency = 0;          /**< \brief Latency of last Glossy phase, in us. */
  static unsigned long sum_latency = 0;      /**< \brief Current sum of latencies, in ticks of low-frequency clock (used to compute average). */
  
  if (get_rx_cnt()) {  // Packet received at least once.
    // Increment number of successfully received packets.
    packets_received++;
    // Compute latency during last Glossy phase.
    rtimer_clock_t lat = get_t_first_rx_l() - get_t_ref_l();
    // Add last latency to sum of latencies.
    sum_latency += lat;
    // Convert latency to microseconds.
    latency = (unsigned long)(lat) * 1e6 / RTIMER_SECOND;
    // Print information about last packet and related latency.
    printf("Glossy received %u time%s: seq_no %u, latency %lu.%03lu ms\n",
        get_rx_cnt(), (get_rx_cnt() > 1) ? "s" : "", glossy_data.seq_no,
            latency / 1000, latency % 1000);
  } else {  // Packet not received.
    // Increment number of missed packets.
    packets_missed++;
    // Print failed reception.
    printf("Glossy NOT received\n");
  }
  printf("skew %d\n", period_skew);
#if GLOSSY_DEBUG
  //printf("skew %ld ppm\n", (long)(period_skew * 1e6) / GLOSSY_PERIOD);
  printf("high_T_irq %u, rx_timeout %u, bad_length %u, bad_header %u, bad_crc %u\n",
         high_T_irq, rx_timeout, bad_length, bad_header, bad_crc);
#endif /* GLOSSY_DEBUG */
  // Compute current average reliability.
  unsigned long avg_rel = packets_received * 1e5 / (packets_received + packets_missed);
  // Print information about average reliability.
  printf("average reliability %3lu.%03lu %% ",
      avg_rel / 1000, avg_rel % 1000);
  printf("(missed %lu out of %lu packets)\n",
      packets_missed, packets_received + packets_missed);
#if ENERGEST_CONF_ON
  // Compute average radio-on time, in microseconds.
  unsigned long avg_radio_on = (unsigned long)GLOSSY_PERIOD * 1e6 / RTIMER_SECOND *
      (energest_type_time(ENERGEST_TYPE_LISTEN) + energest_type_time(ENERGEST_TYPE_TRANSMIT)) /
      (energest_type_time(ENERGEST_TYPE_CPU) + energest_type_time(ENERGEST_TYPE_LPM));
  // Print information about average radio-on time.
  printf("average radio-on time %lu.%03lu ms\n",
      avg_radio_on / 1000, avg_radio_on % 1000);
#endif /* ENERGEST_CONF_ON */
  // Compute average latency, in microseconds.
  unsigned long avg_latency = sum_latency * 1e6 / (RTIMER_SECOND * packets_received);
  // Print information about average latency.
  printf("average latency %lu.%03lu ms\n", avg_latency / 1000, avg_latency % 1000);
}


PROCESS(glossy_print_stats_process, "Glossy print stats");
PROCESS_THREAD(glossy_print_stats_process, ev, data)
{
  PROCESS_BEGIN();
  
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    
    PIN_CLR(FLOCKLAB_INT1);
    PIN_CLR(FLOCKLAB_LED1);
    PIN_CLR(FLOCKLAB_LED2);
    PIN_SET(FLOCKLAB_INT1);
    PIN_SET(FLOCKLAB_LED1);
    PIN_SET(FLOCKLAB_LED2);

  #if PRINT_METRICS   // regardless of MIN_ENERGY_MODE
    print_metrics();
  #endif /* PRINT_METRICS */
    
    if (!IS_INITIATOR()) {
      if (GLOSSY_IS_SYNCED()) {   // receiver with new time information
        update_time_measurements();  
        PROCESS_PAUSE();
      }
    #if SLEEP_BTW_EVENTS
      // increment round counter (seq_no)
      if (rounds_to_skip) {
        glossy_data.seq_no += rounds_to_skip;
      }
    #endif /* SLEEP_BTW_EVENTS */
      if (!GLOSSY_IS_BOOTSTRAPPING()) { 
        calc_linreg();
      }
      
      /* report max. 2 missed event */
      if (evt_counter != EVT_COUNT(glossy_data) &&
          ((evt_counter + 1) % 4) != EVT_COUNT(glossy_data))
      {
        PIN_XOR(SINK_REPORT_PIN);
        __delay_cycles(F_CPU/1000); // wait 1ms
        PIN_XOR(SINK_REPORT_PIN);
      }
      evt_counter = EVT_COUNT(glossy_data);
    }
  
  #if !MIN_ENERGY_MODE
    if (!GLOSSY_IS_BOOTSTRAPPING()) {
      if (DBG_overflow) {
        printf("DBG %lu,%u,%u,%u,%u,%u,%u\n", DBG_tx_try_time, DBG_tx_try_time_idx, DBG_sfd_cap_l_1, DBG_sfd_cap_h_1, DBG_target_dco,DBG_timed_now,DBG_t_irq_timed);
        DBG_overflow = 0;
      }
      printf("dco %u,%u,%u,%u, %u,%u,%u,%u,%u, %u,%d\n", dco_int_h, dco_int_l, dco_int_rx_h, dco_int_rx_l, dco_delay[0],dco_delay[1],dco_delay[2],dco_delay[3],dco_delay[4],dco_comp_threshold,dco_comp);
    }
  #endif /* MIN_ENERGY_MODE */
        
    if (IS_INITIATOR() && get_rx_cnt()) {
      initiator_update_delay_error();
    }
        
  #if !MIN_ENERGY_MODE
    // Print statistics only if Glossy is not still bootstrapping.
    if (!GLOSSY_IS_BOOTSTRAPPING()) {
      print_stats();
    }
  #endif  /* MIN_ENERGY_MODE */

    PIN_CLR(FLOCKLAB_INT1);
    PIN_CLR(FLOCKLAB_LED1);
    PIN_CLR(FLOCKLAB_LED2);
  }

  PROCESS_END();
}

/** @} */

/**
 * \defgroup glossy-test-skew Clock skew estimation
 * @{
 */

static inline void estimate_period_skew(void)
{
  // Estimate clock skew over a period only if the reference time has been updated.
  if (GLOSSY_IS_SYNCED()) {
    // Estimate clock skew based on previous reference time and the Glossy period.
    uint16_t rounds_diff = glossy_data.seq_no - t_ref_l_old_round;
    if (rounds_diff < ((65535 / GLOSSY_PERIOD) - 1)) {
      period_skew = (get_t_ref_l() - (t_ref_l_old + (rtimer_clock_t)GLOSSY_PERIOD * rounds_diff)) / rounds_diff;
    }
    // Update old reference time with the newer one.
    t_ref_l_old = get_t_ref_l();
    t_ref_l_old_round = glossy_data.seq_no;
    if (period_skew > SKEW_MAX || period_skew < -SKEW_MAX) {
      bad_skew_cnt++;
      period_skew = 0;
    }
    // If Glossy is still bootstrapping, count the number of consecutive updates of the reference time.
    if (GLOSSY_IS_BOOTSTRAPPING()) {
      // Increment number of consecutive updates of the reference time.
      bootstrap_sync_history = (bootstrap_sync_history << rounds_diff) | 0x1;
      // Check if Glossy has exited from bootstrapping.
      if (!GLOSSY_IS_BOOTSTRAPPING()) {
        // Glossy has exited from bootstrapping.
        leds_off(LEDS_RED);
        // Initialize Energest values.
        energest_init();
#if GLOSSY_DEBUG
        high_T_irq = 0;
        bad_crc = 0;
        bad_length = 0;
        bad_header = 0;
#endif /* GLOSSY_DEBUG */
      }
    }
  }
}

/** @} */

/**
 * \defgroup glossy-test-scheduler Periodic scheduling
 * @{
 */

static uint16_t light_state;
#if USE_LIGHT_SENSOR
static uint16_t light_threshold;
#endif /* USE_LIGHT_SENSOR */
#if SLEEP_BTW_EVENTS && SLEEP_BTW_EVENTS_LONG
static uint16_t ev_det_low_latency = 0,
                ev_det_high_latency = 0;
#endif /* SLEEP_BTW_EVENTS_LONG */
static uint8_t        sync_missed = 0;   // Current number of consecutive phases without synchronization (reference time not computed)
static rtimer_clock_t t_guard = 0;

char comm_task(struct rtimer *t, void *ptr)
{
  PT_BEGIN(&pt);

  if (IS_INITIATOR()) {
    
    // Glossy initiator ##############################
    while (1) {
      PIN_SET(FLOCKLAB_INT1);      
      // Increment sequence number.
      glossy_data.seq_no++;

  #if USE_LIGHT_SENSOR
      light_state = //(light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC) > light_threshold) ||
                    (light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR) > light_threshold);
      // note: off-state values are just over 80 whereas the on-values are 122 (LIGHT_SENSOR_PHOTOSYNTHETIC) and 629 (LIGHT_SENSOR_TOTAL_SOLAR)
  #else /* USE_LIGHT_SENSOR */
      light_state = PIN_GET(FLOCKLAB_SIG1);
  #endif /* USE_LIGHT_SENSOR */
      if (LIGHT_STATE(glossy_data) != light_state) {
        // light state has changed, reset round count
        glossy_data.light_state  = light_state;
        evt_counter++;
        SET_EVT_COUNT(glossy_data, evt_counter);
        glossy_data.sleep_btw_ev = 0;           // reset
  #if SLEEP_BTW_EVENTS
        uint16_t sleep_duration  = SLEEP_BTW_EVENTS_ROUNDS;
      #if SLEEP_BTW_EVENTS_LONG
        // depending on the current average detection latency (estimate), choose the long or short sleep period
        if (ev_det_low_latency > (ev_det_high_latency * 4)) {   // 80% threshold
          sleep_duration = SLEEP_BTW_EVENTS_LONG;
        }
      #endif /* SLEEP_BTW_EVENTS_LONG */
        if (rounds_to_skip) {
          // event happened during the sleep phase
          // is the remaining time after wakeup sufficient to enter sleep mode?
          if (sleep_duration > (rounds_to_skip + SLEEP_BTW_EVENTS_START)) {
            SET_SLEEP_CNT_DUR(glossy_data, SLEEP_BTW_EVENTS_START, sleep_duration - rounds_to_skip);
          } // else: do not enter sleep mode next time
      #if SLEEP_BTW_EVENTS_LONG
          ev_det_high_latency++;
      #endif /* SLEEP_BTW_EVENTS_LONG */
        } else {
          SET_SLEEP_CNT_DUR(glossy_data, SLEEP_BTW_EVENTS_START, sleep_duration);
      #if SLEEP_BTW_EVENTS_LONG
          ev_det_low_latency++;
      #endif /* SLEEP_BTW_EVENTS_LONG */
        }
      }
      // only start glossy if not in the sleep phase
      if (rounds_to_skip) {
        rounds_to_skip--;
      } else {
        if (IS_TIME_TO_SLEEP(glossy_data)) {
          rounds_to_skip = SLEEP_DURATION(glossy_data);
        }
  #else /* SLEEP_BTW_EVENTS */
      }
      {
  #endif /* SLEEP_BTW_EVENTS */
        
        rtimer_clock_t t_stop = RTIMER_TIME(t) + GLOSSY_DURATION;
        glossy_start((uint8_t *)&glossy_data, DATA_LEN, GLOSSY_INITIATOR, GLOSSY_SYNC, N_TX_INITIATOR,
                     APPLICATION_HEADER, t_stop, (rtimer_callback_t)comm_task, t, ptr, 0, glossy_data.seq_no);
        // Store time at which Glossy has started.
        // Yield the protothread. It will be resumed when Glossy terminates.
        PT_YIELD(&pt);
        glossy_stop();

        if (rounds_to_skip) {
          glossy_data.sleep_btw_ev = 0;       // reset
        }
        DECR_SLEEP_COUNTDOWN(glossy_data);
      }
      t_start = RTIMER_TIME(t);
      // Schedule begin of next Glossy phase based on GLOSSY_PERIOD.
      rtimer_set_long(t, t_start, GLOSSY_PERIOD, (rtimer_callback_t)comm_task, ptr);
      // Poll the process that prints statistics (will be activated later by Contiki).
      process_poll(&glossy_print_stats_process);
      // Yield the protothread.
      PT_YIELD(&pt);
    }
  } else {

    // Glossy receiver ##############################
    while (1) {      
      PIN_SET(FLOCKLAB_INT1);      
      // Glossy phase.
      rtimer_clock_t t_stop = RTIMER_TIME(t) + GLOSSY_DURATION + t_guard;
      PIN_CLR(FLOCKLAB_INT1);
      glossy_start((uint8_t *)&glossy_data, DATA_LEN, (GLOSSY_USE_CHANNEL_HOPPING())<<1, GLOSSY_SYNC, N_TX_RECEIVER,
                   APPLICATION_HEADER, t_stop, (rtimer_callback_t)comm_task, t, ptr, t_first_slot, glossy_data.seq_no+1);
      // Yield the protothread. It will be resumed when Glossy terminates.
      PT_YIELD(&pt);
      glossy_stop();
      
      set_tx_times_updated(0);
      
      // adjust t_start
      if (GLOSSY_IS_SYNCED()) {
        t_start = GLOSSY_REFERENCE_TIME;
      } else {
        // The reference time was not updated
        t_start = RTIMER_TIME(t) + t_guard;
        DECR_SLEEP_COUNTDOWN(glossy_data);    // manually decrement the counter
      }
      if (!GLOSSY_IS_BOOTSTRAPPING()) {
        // Glossy has already successfully bootstrapped.
        has_bootstrapped = 1;
        if (!GLOSSY_IS_SYNCED()) {
          // The reference time was not updated:
          // Increment sync_missed.
          sync_missed++;
          // increment sequence number
          glossy_data.seq_no++;
          if (sync_missed == GLOSSY_MISSES_FOR_BOOTRSTRAP) {
            // reset states, back to bootstrap mode
            bootstrap_sync_history = 0;
            sync_missed = 0;
            period_skew = 0;
            init_reg_tables();
          }
        } else {    
          sync_missed = 0;      // The reference time was updated: reset sync_missed to zero.
        }
      }
      // Estimate the clock skew over the last period.
      estimate_period_skew();
      
  #if SLEEP_BTW_EVENTS
      if (IS_TIME_TO_SLEEP(glossy_data))
      {
        rounds_to_skip = SLEEP_DURATION(glossy_data);
        glossy_data.sleep_btw_ev = 0;       // reset
        skipped_rounds_since_last_sync = 1;
      } else {
        rounds_to_skip = 0;
        if (GLOSSY_IS_SYNCED())
          skipped_rounds_since_last_sync = 0;
      }
  #endif /* SLEEP_BTW_EVENTS */
        
      if (GLOSSY_IS_BOOTSTRAPPING()) {
        // Glossy is still bootstrapping.
        if (!GLOSSY_IS_SYNCED()) {
          // The reference time was not updated:
          rtimer_set(t, RTIMER_NOW() + GLOSSY_INIT_PAUSE, 1, (rtimer_callback_t)comm_task, ptr);
          t_guard = GLOSSY_INIT_DURATION - GLOSSY_DURATION;
        } else {
          // The reference time was updated:
          rtimer_set_long(t, t_start, GLOSSY_PERIOD * (1 + rounds_to_skip) - GLOSSY_INIT_GUARD_TIME, (rtimer_callback_t)comm_task, ptr);
          t_guard = GLOSSY_INIT_GUARD_TIME;
        }
      } else {
        // Glossy has already successfully bootstrapped:
        // Schedule begin of next Glossy phase based on reference time and GLOSSY_PERIOD.
        unsigned long period = (GLOSSY_PERIOD + period_skew) * (rounds_to_skip + 1);
        rtimer_set_long(t, t_start, period - GLOSSY_GUARD_TIME, (rtimer_callback_t)comm_task, ptr);
        t_guard = GLOSSY_GUARD_TIME;
        t_first_slot = (t_start + period) << 7; // convert to vht time unit
      }
      if (last_rtimer_low > RTIMER_NOW()) {
        rtimer_ext++;
      }
      last_rtimer_low = RTIMER_NOW();
      // Poll the process that prints statistics (will be activated later by Contiki).
      process_poll(&glossy_print_stats_process);
      // Yield the protothread.
      PT_YIELD(&pt);
    }
  }

  PT_END(&pt);
}

/** @} */

/**
 * \defgroup glossy-test-init Initialization
 * @{
 */

PROCESS(glossy_test, "Glossy test");
AUTOSTART_PROCESSES(&glossy_test);
PROCESS_THREAD(glossy_test, ev, data)
{
  PROCESS_BEGIN();
    
#if !COMPETITION_MODE
  PIN_CFG_OUT(FLOCKLAB_LED1);   // used to indicate RF activity in glossy.c and jammer.c
  PIN_CFG_OUT(FLOCKLAB_LED2);   // used in glossy.c to indicate RX of a packet
  PIN_CFG_OUT(FLOCKLAB_LED3);   // used in glossy.c to measure execution time
  PIN_CFG_OUT(FLOCKLAB_INT1);   // used in glossy.c to measure execution time
  PIN_CFG_OUT(FLOCKLAB_INT2);   // used to indicate an event trigger and detection on source and sink
  PIN_CFG_IN(FLOCKLAB_SIG1);    // to simulate the lamp state (on/off)
  PIN_CFG_IN(FLOCKLAB_SIG2);
#endif /* COMPETITION_MODE */
  leds_on(LEDS_RED);

  PIN_CFG_OUT(SINK_REPORT_PIN);
  PIN_CLR(SINK_REPORT_PIN);
  
  // put external flash memory into deep power-down mode
  SPI_FLASH_ENABLE();       // set chip select low (P4.4)
  FASTSPI_TX(0xb9);         // send deep power-down command
  SPI_WAITFORTx_ENDED();    // wait for transmission to end
  SPI_FLASH_DISABLE();      // set chip select high  
    
  // Initialize Glossy data.
  glossy_data.seq_no       = 0;
  glossy_data.light_state  = 0;
  glossy_data.sleep_btw_ev = 0;
  if (!IS_INITIATOR()) {
    init_reg_tables();
  }
#if USE_LIGHT_SENSOR 
  else {
    SENSORS_ACTIVATE(light_sensor);
    __delay_cycles(F_CPU / 50);   // wait 20ms
  #if LIGHT_SENSOR_THRES
    // use a fixed predefined threshold
    light_threshold = LIGHT_SENSOR_THRES;
  #else
    // read the off-state value of the light sensor
    light_threshold = (light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR) + light_sensor.value(LIGHT_SENSOR_TOTAL_SOLAR)) / 2 + 50;
  #endif /* LIGHT_SENSOR_THRES */
    printf("light sensor threshold: %u\n", light_threshold);
  }
#endif /* USE_LIGHT_SENSOR  */

  // print the most important parameters
  printf("round duration/period: %u/%u, n_tx: %u, n_ch: %u, sleep: %u\n", (uint16_t)((uint32_t)GLOSSY_DURATION * 1000 / RTIMER_SECOND), GLOSSY_PERIOD_MS, N_TX, N_CH, SLEEP_BTW_EVENTS);
  
  printf("Message delay is %u\n", (uint16_t)CC2420_MESSAGE_DELAY);
  process_start(&glossy_print_stats_process, NULL);
  process_start(&glossy_process, NULL);
    
  // start the communication task in 1 second
  rtimer_set(&rt, RTIMER_NOW() + RTIMER_SECOND, 1, (rtimer_callback_t)comm_task, NULL);
  last_rtimer_low = RTIMER_NOW();
  (void)PT_YIELD_FLAG;
  
  PROCESS_END();
}

/** @} */
/** @} */
/** @} */

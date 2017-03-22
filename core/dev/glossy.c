/*
 * Copyright (c) 2011-2017, ETH Zurich.
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
 *          Federico Ferrari
 *
 */

/**
 * \file
 *         Glossy core, source file.
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#include "glossy.h"

#define CM_POS              CM_1
#define CM_NEG              CM_2
#define CM_BOTH             CM_3

static uint8_t initiator, sync, rx_cnt, tx_cnt, tx_max, rx_try_cnt;
static uint8_t *data, *packet;
static uint8_t data_len, packet_len, packet_len_tmp, header;
static uint8_t bytes_read, tx_relay_cnt_last;
static int8_t n_timeouts;
static volatile uint8_t state;
static rtimer_clock_t t_rx_start, t_rx_stop, t_tx_start, t_tx_stop, t_start;
static rtimer_clock_t t_rx_timeout;
static rtimer_clock_t T_irq;
static rtimer_clock_t t_stop;
static rtimer_callback_t cb;
static struct rtimer *rtimer;
static void *ptr;
static unsigned short ie1, ie2, p1ie, p2ie, tbiv;
static int16_t s[30];

static rtimer_clock_t T_slot_h, t_ref_l, T_offset_h, t_first_rx_l, T_tx_h, T_32_h;
#if GLOSSY_SYNC_WINDOW
static unsigned long T_slot_h_sum;
static uint8_t win_cnt;
#endif /* GLOSSY_SYNC_WINDOW */
static uint8_t relay_cnt, t_ref_l_updated;
rtimer_clock_t glossy_tx_tx_wait, glossy_rx_tx_delay;

static uint8_t packet_len_cached;
static rtimer_clock_t tx_slot_len_cached;
static rtimer_clock_t calib_cap_h1, calib_cap_l1;
static int16_t measured_slack_time;

static const uint16_t channelvalues[] = {
 357 + 5 * (26 - 11) + 0x4000,
 352 + 3 + 0x4000,                      // 2403
 357 + 5 * (25 - 11) + 0x4000,
#if N_CH > 3
 357 + 5 * (20 - 11) + 0x4000,
#endif
#if N_CH > 4
 357 + 5 * (14 - 11) + 0x4000,
#endif
#if N_CH > 5
 357 + 5 * (15 - 11) + 2 + 0x4000,
#endif
#if N_CH > 6
 357 + 5 * (18 - 11) + 3 + 0x4000,
#endif
#if N_CH > 7
 357 + 5 * (13 - 11) + 0x4000,
#endif
#if N_CH > 8
 357 + 5 * (16 - 11) + 0x4000,
#endif
#if N_CH > 9
 357 + 5 * (24 - 11) + 0x4000,
#endif
#if N_CH > 10
 357 + 5 * (23 - 11) + 0x4000,
#endif
#if N_CH > 11
 357 + 5 * (15 - 11) + 0x4000,
#endif
#if N_CH > 12
 357 + 5 * (12 - 11) + 0x4000,
#endif
#if N_CH > 13
 357 + 5 * (11 - 11) + 0x4000,
#endif
#if N_CH > 14
 357 + 5 * (22 - 11) + 0x4000,
#endif
#if N_CH > 15
 357 + 5 * (17 - 11) + 0x4000,
#endif
}; // channels 11 to 26
static uint8_t channel_history[GLOSSY_CHANNEL_HISTORY_SIZE];
static uint8_t channel_history_count = 0;
static uint8_t channel_history_pos = 0;

static uint8_t glossy_channel_offset;
static uint8_t dohopping;
static uint8_t next_channel;
static uint8_t channel_offset_adjusted;
inline void glossy_set_channel(uint8_t channelidx) __attribute__((always_inline));
inline void glossy_set_channel_off_on(uint8_t channelidx) __attribute__((always_inline));
inline void glossy_set_channel_off(uint8_t channelidx) __attribute__((always_inline));
void glossy_schedule_channel_timeout();
void glossy_schedule_channel_timeout_time();
void glossy_stop_channel_timeout();
void add_channel_to_history(uint8_t channelidx);
uint8_t get_best_channel_from_history();
uint8_t get_good_random_channel();

#define NUM_CHANNELVALUES (sizeof(channelvalues)/sizeof(channelvalues[0]))

#define CALC_NEXT_CHANNEL(slot) do{\
  next_channel = (slot + glossy_channel_offset) % NUM_CHANNELVALUES;\
}while(0)

#define CALC_DCO_COMPENSATION(interval_h, interval_l, out_compensate, out_threshold) do{\
  if ((interval_h) > 128 * (interval_l))\
    out_compensate = 1; /* dco is fast, do more to compensate */\
  else if ((interval_h) < 128 * (interval_l))\
    out_compensate = -1; /* dco is slow, do less to compensate */\
  else out_compensate = 0;\
  if (out_compensate != 0)\
    out_threshold = (128 * (interval_l)) / (2*out_compensate*((interval_h) - 128 * (interval_l)));\
  else\
    out_threshold = 0xffff;\
}while(0)

// for accurate vht timestamp
rtimer_clock_t sfd_cap_l, sfd_cap_h;
static rtimer_clock_t t_ref_local_l, t_ref_local_h;
static uint8_t tx_times_updated;
static uint8_t next_tx_time_idx;
static uint8_t tx_try_idx;

/* --------------------------- Radio functions ---------------------- */
static inline void radio_flush_tx(void) {
  FASTSPI_STROBE(CC2420_SFLUSHTX);
}

static inline uint8_t radio_status(void) {
  uint8_t status;
  FASTSPI_UPD_STATUS(status);
  return status;
}

static inline void radio_on(void) {
  FASTSPI_STROBE(CC2420_SRXON);
  while(!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
}

static inline void radio_off(void) {
#if ENERGEST_CONF_ON
  if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  }
  if (energest_current_mode[ENERGEST_TYPE_LISTEN]) {
    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  }
#endif /* ENERGEST_CONF_ON */
  FASTSPI_STROBE(CC2420_SRFOFF);
}

static inline void radio_flush_rx(void) {
  uint8_t dummy;
  FASTSPI_READ_FIFO_BYTE(dummy);
  FASTSPI_STROBE(CC2420_SFLUSHRX);
  FASTSPI_STROBE(CC2420_SFLUSHRX);
  (void)dummy;
}

static inline void radio_abort_rx(void) {
  state = GLOSSY_STATE_ABORTED;
  radio_flush_rx();
  PIN_CLR(FLOCKLAB_LED2);
}

static inline void radio_abort_tx(void) {
  FASTSPI_STROBE(CC2420_SRXON);
#if ENERGEST_CONF_ON
  if (energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  }
#endif /* ENERGEST_CONF_ON */
  radio_flush_rx();
  PIN_CLR(FLOCKLAB_LED1);
  if (dohopping) {
    n_timeouts ++;
    glossy_schedule_channel_timeout();
  }
}

static inline void radio_start_tx(void) {
  FASTSPI_STROBE(CC2420_STXON);
  PIN_SET(FLOCKLAB_LED1);
#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
}

static inline void radio_write_tx(void) {
  FASTSPI_WRITE_FIFO(packet, packet_len_tmp - 1);
}

uint16_t t_overflow=0;
uint16_t t_noverflow=0;
/* --------------------------- SFD interrupt ------------------------ */
// RX->TX
uint8_t start_tx_delayed(uint16_t delaycycles) {
  // wait until processing_cycles cycles occurred since the last SFD event
  TBCCR4 = TBCCR1 + delaycycles;
  TBCCTL4 &= ~(CCIE | CCIFG);
//   PIN_SET(FLOCKLAB_LED3);
  do {
#if COOJA
  } while (TBCCR4 - RTIMER_NOW_DCO() < 0x8000);
#else
  } while (!(TBCCTL4 & CCIFG));
#endif
  TBCCTL4 = 0;
  // the next instruction is executed at least delaycycles+12 cycles since the last SFD event
  T_irq = ((RTIMER_NOW_DCO() - TBCCR1) - (delaycycles + 10)) << 1;
  // one iteration of while loop takes 9 cycles
  // T_irq in [0,...,34]
  if (T_irq <= 34) {
    // NOPs (variable number) to compensate for the interrupt service and the busy waiting delay
    asm volatile("add %[d], r0" : : [d] "m" (T_irq));
    asm volatile("nop");            // irq_delay = 0
    asm volatile("nop");            // irq_delay = 2
    asm volatile("nop");            // irq_delay = 4
    asm volatile("nop");            // irq_delay = 6
    asm volatile("nop");            // irq_delay = 8
    asm volatile("nop");            // irq_delay = 10
    asm volatile("nop");            // irq_delay = 12
    asm volatile("nop");            // irq_delay = 14
    asm volatile("nop");            // irq_delay = 16
    asm volatile("nop");            // irq_delay = 18
    asm volatile("nop");            // irq_delay = 20
    asm volatile("nop");            // irq_delay = 22
    asm volatile("nop");            // irq_delay = 24
    asm volatile("nop");            // irq_delay = 26
    asm volatile("nop");            // irq_delay = 28
    asm volatile("nop");            // irq_delay = 30
    asm volatile("nop");            // irq_delay = 32
    asm volatile("nop");            // irq_delay = 34
    // relay the packet
    radio_start_tx();
    t_noverflow = T_irq;
    return 1;
  }
  PIN_SET(FLOCKLAB_LED3);
  t_overflow = T_irq;
  PIN_CLR(FLOCKLAB_LED3);
  return 0;
}

// TX->TX
uint8_t start_tx_timed() {
  // use precomputed tx times
  rtimer_clock_t t_cap_h, t_cap_l, target_dco, t_irq_timed, timed_now;
  READ_CAPTURE_NEXT_CLOCK_TICK_NOWAIT(t_cap_h, t_cap_l);
  target_dco = ((uint16_t)tx_times[next_tx_time_idx]) - ((uint16_t)(t_cap_l<<7));
  if (target_dco > dco_comp_threshold)
    target_dco+=dco_comp;
  target_dco+= t_cap_h - 17;
  timed_now = RTIMER_NOW_DCO();
  // TODO: replace 36 with actual minimal number of cycles needed
  if (!RTIMER_CLOCK_LT(timed_now, (target_dco - 36))) {
    // we missed the target dco
    PIN_CLR(FLOCKLAB_LED3);
    PIN_SET(FLOCKLAB_LED3);
    PIN_CLR(FLOCKLAB_LED3);
    tx_cnt = 0xff;
    // TODO: schedule tx timeout
    DBG_tx_try_time = tx_times[next_tx_time_idx];
    DBG_tx_try_time_idx = next_tx_time_idx;
    DBG_sfd_cap_h_1 = t_cap_h;
    DBG_sfd_cap_l_1 = t_cap_l;
    DBG_target_dco = target_dco;
    DBG_timed_now = timed_now;
    DBG_overflow = 1;
    return 0;
  }
  PIN_CLR(FLOCKLAB_LED3);
  TBCCR4 = target_dco;
  TBCCTL4 &= ~(CCIE | CCIFG);
  do {
#if COOJA
  } while (target_dco - RTIMER_NOW_DCO() < 0x8000);
#else
  } while (!(TBCCTL4 & CCIFG));
#endif
  TBCCTL4 = 0;
  t_irq_timed = (RTIMER_NOW_DCO() - (target_dco + 17)+6) << 1;
  PIN_SET(FLOCKLAB_LED3);
  if (t_irq_timed <= 14) {
    // NOPs (variable number) to compensate for the interrupt service and the busy waiting delay
    asm volatile("add %[d], r0" : : [d] "m" (t_irq_timed));
    asm volatile("nop");            // irq_delay = 0
    asm volatile("nop");            // irq_delay = 2
    asm volatile("nop");            // irq_delay = 4
    asm volatile("nop");            // irq_delay = 6
    asm volatile("nop");            // irq_delay = 8
    asm volatile("nop");            // irq_delay = 10
    asm volatile("nop");            // irq_delay = 12
    asm volatile("nop");            // irq_delay = 14
    // start the transmission
    radio_start_tx();
    PIN_CLR(FLOCKLAB_LED3);
    dco_delay[next_tx_time_idx] = target_dco - t_cap_h;
    return 1;
  }
  else {
    PIN_CLR(FLOCKLAB_LED3);
    PIN_SET(FLOCKLAB_LED3);
    PIN_CLR(FLOCKLAB_LED3);
    tx_cnt = 0xff;
    dco_delay[next_tx_time_idx] = target_dco - t_cap_h;
    return 0;
  }
}


interrupt(TIMERB1_VECTOR)/* __attribute__ ((section(".glossy")))*/
timerb1_interrupt(void)
{
  // NOTE: if you modify the code if this function
  // you may need to change the constant part of the interrupt delay (currently 21 DCO ticks),
  // due to possible different compiler optimizations
  rtimer_clock_t t_listen_timeout;
  PIN_SET(FLOCKLAB_INT1);
  if (state == GLOSSY_STATE_RECEIVING && !SFD_IS_1) {
    // packet reception has finished
    ARM_CAPTURE_NEXT_CLOCK_TICK();
    glossy_set_channel_off(next_channel);
    uint8_t start_tx_success = 0;
    if (!initiator) {
#if GLOSSY_TX_FIRST_TIMED
      if (tx_times_updated) {
        PIN_SET(FLOCKLAB_LED3);
        start_tx_success = start_tx_timed();
      }
      else {
#endif
        DISABLE_CAPTURE_NEXT_CLOCK_TICK();
        start_tx_success = start_tx_delayed(glossy_rx_tx_delay);
#if GLOSSY_TX_FIRST_TIMED
      }
#endif
    }
    if (initiator || start_tx_success) {
      glossy_stop_channel_timeout();
      glossy_end_rx();
    } else {
      // interrupt service delay is too high: do not relay the packet
      radio_flush_rx();
      FASTSPI_STROBE(CC2420_SRXON);
      state = GLOSSY_STATE_WAITING;
    }
    // read TBIV to clear IFG
    tbiv = TBIV;
  } else {
    if (state == GLOSSY_STATE_TRANSMITTING && !SFD_IS_1) {
      // packet transmission has finished
      PIN_CLR(FLOCKLAB_LED1);
      if (tx_cnt < tx_max -1) {
        ARM_CAPTURE_NEXT_CLOCK_TICK();
        glossy_set_channel_off(next_channel);
        if (tx_times_updated) {
          PIN_SET(FLOCKLAB_LED3);
          start_tx_timed();
        }
        else {
          DISABLE_CAPTURE_NEXT_CLOCK_TICK();
          start_tx_delayed(glossy_tx_tx_wait);
        }
      }
      state = GLOSSY_STATE_RECEIVED;
      // read TBIV to clear IFG
      tbiv = TBIV;
      glossy_end_tx();
    } else {
      // read TBIV to clear IFG
      tbiv = TBIV;
      if (state == GLOSSY_STATE_WAITING && SFD_IS_1) {
        // packet reception has started
        ARM_CAPTURE_NEXT_CLOCK_TICK();
        glossy_stop_channel_timeout();
        glossy_begin_rx();
      } else {
        if (state == GLOSSY_STATE_RECEIVED && SFD_IS_1) {
          // packet transmission has started
          glossy_begin_tx();
        } else {
          if (state == GLOSSY_STATE_ABORTED) {
            // packet reception has been aborted
            if (dohopping) {
              PIN_SET(FLOCKLAB_LED3);
              // switch to next channel
              CALC_NEXT_CHANNEL(n_timeouts+1);
              glossy_set_channel_off_on(next_channel);
              // schedule timeout based on elapsed time
              glossy_schedule_channel_timeout_time();
              PIN_CLR(FLOCKLAB_LED3);
            }
            state = GLOSSY_STATE_WAITING;
          } else {
            if ((state == GLOSSY_STATE_WAITING) && (tbiv == TBIV_TBCCR4)) {
              // channel timeout
              if (initiator) {
                glossy_stop_channel_timeout();
                radio_off();
                state = GLOSSY_STATE_OFF;
              }
              else {
                n_timeouts++;
                if (rx_cnt == 0) {
                  PIN_SET(FLOCKLAB_LED3);
                  CALC_NEXT_CHANNEL(n_timeouts);
                  glossy_set_channel_off_on(next_channel);
                  PIN_CLR(FLOCKLAB_LED3);
                  // schedule the timeout again
                  t_listen_timeout = TBCCR4 + (CC2420_TXTOPAYLOADBYTES * F_CPU) / 31250 + 1328;
                  glossy_schedule_channel_timeout();
                  // busy wait for SFD IRQ, turn off radio otherwise
                  while (!SFD_IS_1) {
                    if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_listen_timeout)) {
                      FASTSPI_STROBE(CC2420_SRFOFF);
                      PIN_SET(FLOCKLAB_LED3);
                      PIN_CLR(FLOCKLAB_LED3);
                      break;
                    }
                  }
                  PIN_SET(FLOCKLAB_LED3);
                  PIN_CLR(FLOCKLAB_LED3);
                } else {
                  // at least one packet has been received: just stop the timeout
                  glossy_stop_channel_timeout();
                }
              }
            } else {
              if (tbiv == TBIV_TBCCR5) {
                // rx timeout
                if (state == GLOSSY_STATE_RECEIVING) {
                  // we are still trying to receive a packet: abort the reception
                  radio_abort_rx();
#if GLOSSY_DEBUG
                  rx_timeout++;
#endif /* GLOSSY_DEBUG */
                }
                // stop the timeout
                glossy_stop_rx_timeout();
              } else {
                if (state != GLOSSY_STATE_OFF) {
                  // something strange is going on: go back to the waiting state
                  radio_flush_rx();
                  state = GLOSSY_STATE_WAITING;
                }
              }
            }
          }
        }
      }
    }
  }
PIN_CLR(FLOCKLAB_INT1);
}

/* --------------------------- Glossy process ----------------------- */
PROCESS(glossy_process, "Glossy busy-waiting process");
PROCESS_THREAD(glossy_process, ev, data) {
  PROCESS_BEGIN();

  do {
    packet = (uint8_t *) malloc(128);
  } while (packet == NULL);

  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
    // prevent the Contiki main cycle to enter the LPM mode or
    // any other process to run while Glossy is running
    while (GLOSSY_IS_ON() && RTIMER_CLOCK_LT(RTIMER_NOW(), t_stop));
#if COOJA
    while (state == GLOSSY_STATE_TRANSMITTING);
#endif /* COOJA */
    // Glossy finished: execute the callback function
    dint();
    cb(rtimer, ptr);
    eint();
  }

  PROCESS_END();
}

static inline void glossy_disable_other_interrupts(void) {
    int s = splhigh();
  ie1 = IE1;
  ie2 = IE2;
  p1ie = P1IE;
  p2ie = P2IE;
  IE1 = 0;
  IE2 = 0;
  P1IE = 0;
  P2IE = 0;
  CACTL1 &= ~CAIE;
  DMA0CTL &= ~DMAIE;
  DMA1CTL &= ~DMAIE;
  DMA2CTL &= ~DMAIE;
  // disable etimer interrupts
  TACCTL1 &= ~CCIE;
  TBCCTL0 = 0;
  DISABLE_FIFOP_INT();
  CLEAR_FIFOP_INT();
  SFD_CAP_INIT(CM_BOTH);
  ENABLE_SFD_INT();
  // stop Timer B
  TBCTL = 0;
  // Timer B sourced by the DCO
  TBCTL = TBSSEL1;
  // start Timer B
  TBCTL |= MC1;
    splx(s);
    watchdog_stop();
}

static inline void glossy_enable_other_interrupts(void) {
  int s = splhigh();
  IE1 = ie1;
  IE2 = ie2;
  P1IE = p1ie;
  P2IE = p2ie;
  // enable etimer interrupts
  TACCTL1 |= CCIE;
#if COOJA
  if (TACCTL1 & CCIFG) {
    etimer_interrupt();
  }
#endif
  DISABLE_SFD_INT();
  CLEAR_SFD_INT();
  FIFOP_INT_INIT();
  ENABLE_FIFOP_INT();
  // stop Timer B
  TBCTL = 0;
  // Timer B sourced by the 32 kHz
  TBCTL = TBSSEL0;
  // start Timer B
  TBCTL |= MC1;
    splx(s);
    watchdog_start();
}

inline void calculate_next_tx_time_idx(void) {
  if (GLOSSY_RELAY_CNT_FIELD < N_MAX_TX_SLOTS)
    next_tx_time_idx = GLOSSY_RELAY_CNT_FIELD + 1;
  while(RTIMER_CLOCK_LT((rtimer_clock_t)(tx_times[next_tx_time_idx] >> 7),RTIMER_NOW())) {
    if (next_tx_time_idx < N_MAX_TX_SLOTS)
      next_tx_time_idx++;
    else
      break;
  }
}

/* --------------------------- Main interface ----------------------- */
void glossy_start(uint8_t *data_, uint8_t data_len_, uint8_t initiator_,
    uint8_t sync_, uint8_t tx_max_, uint8_t header_,
    rtimer_clock_t t_stop_, rtimer_callback_t cb_,
    struct rtimer *rtimer_, void *ptr_, vht_clock_t t_start_vht, uint8_t glossy_channel_offset_) {
  uint8_t i;
  rtimer_clock_t calib_cap_h2, calib_cap_l2;
  // copy function arguments to the respective Glossy variables
  data = data_;
  data_len = data_len_;
  initiator = initiator_ & 0x1;
#if GLOSSY_DO_HOPPING
  dohopping = (initiator_ & 0x2)>0;
#endif /* GLOSSY_DO_HOPPING */
  sync = sync_;
  tx_max = tx_max_;
  header = header_;
  t_stop = t_stop_;
  cb = cb_;
  rtimer = rtimer_;
  ptr = ptr_;
  // disable all interrupts that may interfere with Glossy
  glossy_disable_other_interrupts();
  FASTSPI_STROBE(CC2420_SXOSCON);
  // initialize Glossy variables
  tx_cnt = 0;
  rx_cnt = 0;
  rx_try_cnt = 0; // for debugging
  glossy_channel_offset = glossy_channel_offset_;
    tx_try_idx = 0;
  CAPTURE_NEXT_CLOCK_TICK(calib_cap_h1, calib_cap_l1);
  // set Glossy packet length, with or without relay counter depending on the sync flag value
  if (data_len) {
    packet_len_tmp = (sync) ?
        data_len + FOOTER_LEN + GLOSSY_RELAY_CNT_LEN + GLOSSY_HEADER_LEN :
        data_len + FOOTER_LEN + GLOSSY_HEADER_LEN;
    packet_len = packet_len_tmp;
    // set the packet length field to the appropriate value
    GLOSSY_LEN_FIELD = packet_len_tmp;
    // set the header field
    GLOSSY_HEADER_FIELD = GLOSSY_HEADER | (header & ~GLOSSY_HEADER_MASK);
  } else {
    // packet length not known yet (only for receivers)
    packet_len = 0;
  }
  if (initiator) {
    // initiator: copy the application data to the data field
    memcpy(&GLOSSY_DATA_FIELD, data, data_len);
    // set Glossy state
    state = GLOSSY_STATE_RECEIVED;
    // set tx_times
    PIN_SET(FLOCKLAB_LED3);
    vht_clock_t t_global = ((vht_clock_t)(RTIMER_TIME(rtimer) + GLOSSY_INITIATOR_DELAY)) << 7;
    for (i=0;i<=tx_max;i++) {
        tx_times[i] = t_global;
        t_global+=get_tx_slot_len();
    }
    t_start_vht = tx_times[0] + CC2420_MESSAGE_DELAY;
    t_start = (uint16_t)t_start_vht - ((uint16_t)(calib_cap_l1 << 7)) + calib_cap_h1;
    tx_times_updated = 1;
    next_tx_time_idx = 0;
    PIN_CLR(FLOCKLAB_LED3);
  } else {
    PIN_SET(FLOCKLAB_LED3);
    // receiver: set Glossy state
    state = GLOSSY_STATE_WAITING;
    next_tx_time_idx = 1;
#if GLOSSY_DO_HOPPING
    if (!dohopping) {
      glossy_channel_offset = get_good_random_channel();
      channel_offset_adjusted = 0;
    }
#else
    glossy_channel_offset = 0;
#endif
    PIN_CLR(FLOCKLAB_LED3);
  }
  if (sync) {
    // set the relay_cnt field to 0
    GLOSSY_RELAY_CNT_FIELD = 0;
    // the reference time has not been updated yet
    t_ref_l_updated = 0;
  }
  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();
  CALC_NEXT_CHANNEL(GLOSSY_RELAY_CNT_FIELD);
  while(!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));
  glossy_set_channel(next_channel);
  CAPTURE_NEXT_CLOCK_TICK(calib_cap_h2, calib_cap_l2);
  dco_int_h = calib_cap_h2 - calib_cap_h1;
  dco_int_l = calib_cap_l2 - calib_cap_l1;
  CALC_DCO_COMPENSATION(dco_int_h, dco_int_l, dco_comp, dco_comp_threshold);
  if (initiator) {
    rtimer_clock_t tmp_time = RTIMER_TIME(rtimer) + GLOSSY_INITIATOR_DELAY-4;
    measured_slack_time = tmp_time - RTIMER_NOW();
    while(RTIMER_CLOCK_LT(RTIMER_NOW(), tmp_time));
    ARM_CAPTURE_NEXT_CLOCK_TICK();
    // write the packet to the TXFIFO
    radio_write_tx();
    PIN_SET(FLOCKLAB_LED3);
    start_tx_timed();
    CALC_NEXT_CHANNEL(GLOSSY_RELAY_CNT_FIELD + 1);
    // sync dco and calculate DCO compensation while radio is on
    msp430_sync_dco();
    CAPTURE_NEXT_CLOCK_TICK(calib_cap_h1, calib_cap_l1); // --> second step of calibration takes place in tx done interrupt
    PIN_CLR(FLOCKLAB_INT1);
  } else {
    glossy_tx_tx_wait = (GLOSSY_TX_TX_WAIT * (uint32_t)dco_int_h) / (128 * dco_int_l);
    glossy_rx_tx_delay = (GLOSSY_RX_TX_DELAY * (uint32_t)dco_int_h) / (128 * dco_int_l);

    int16_t slack_burn_time = 0;
    if (dohopping) {
      n_timeouts = -1;
#if COOJA
      rtimer_clock_t t_cap_l = RTIMER_NOW();
      rtimer_clock_t t_cap_h = RTIMER_NOW_DCO();
#else
      // capture the next low-frequency clock tick
      rtimer_clock_t t_cap_h, t_cap_l;
      CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l);
#endif /* COOJA */
      t_start = (uint16_t)t_start_vht - ((uint16_t)(t_cap_l << 7)) + t_cap_h;
      glossy_schedule_channel_timeout();

      // busy wait on slack time
      PIN_SET(FLOCKLAB_INT1);
      slack_burn_time = measured_slack_time - GLOSSY_RECEIVER_MIN_SLACK;
      if (slack_burn_time > 0 && slack_burn_time < GLOSSY_MAX_SLACK_BURN_TIME) {
        rtimer_clock_t targettime = RTIMER_NOW() + slack_burn_time;
        while (RTIMER_CLOCK_LT(RTIMER_NOW(), targettime));
      } else {
        slack_burn_time = 0;
      }
      PIN_CLR(FLOCKLAB_INT1);    
    }
    else {
      n_timeouts = 0;
    }
    // turn on the radio
    radio_on();
    msp430_sync_dco();
    CAPTURE_NEXT_CLOCK_TICK(calib_cap_h1, calib_cap_l1);
    calib_cap_l2 = calib_cap_l1;
    while (calib_cap_l2 - calib_cap_l1 < 12) {
      CAPTURE_NEXT_CLOCK_TICK(calib_cap_h2, calib_cap_l2);
    }
    dco_int_rx_h = calib_cap_h2 - calib_cap_h1;
    dco_int_rx_l = calib_cap_l2 - calib_cap_l1;
    CALC_DCO_COMPENSATION(dco_int_rx_h, dco_int_rx_l, dco_comp, dco_comp_threshold);
    
    if (dohopping) { 
      measured_slack_time = (((int16_t)TBCCR4 - (int16_t)RTIMER_NOW_DCO()) >> 7) + slack_burn_time;
    }
  }
  // activate the Glossy busy waiting process
  process_poll(&glossy_process);
}

uint8_t glossy_stop(void) {
  // stop the initiator timeout, in case it is still active
  glossy_stop_initiator_timeout();
  // stop the channel timeout, in case it is still active
  glossy_stop_channel_timeout();
  // turn off the radio
  radio_off();
  PIN_CLR(FLOCKLAB_LED1);
  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();
  FASTSPI_STROBE(CC2420_SXOSCOFF);
  state = GLOSSY_STATE_OFF;
  // re-enable non Glossy-related interrupts
  glossy_enable_other_interrupts();
  // store succesful channel
  if (rx_cnt > 0) {
    if (dohopping) {
      add_channel_to_history((glossy_channel_offset + relay_cnt) % NUM_CHANNELVALUES);
    }
    else {
      add_channel_to_history(glossy_channel_offset % NUM_CHANNELVALUES);
    }
  }
  // return the number of times the packet has been received
  return rx_cnt;
}

uint8_t get_rx_cnt(void) {
  return rx_cnt;
}

uint8_t get_relay_cnt(void) {
  return relay_cnt;
}

rtimer_clock_t get_t_ref_local_l(void) {
  return t_ref_local_l;
}

rtimer_clock_t get_t_ref_local_h(void) {
  return t_ref_local_h;
}

rtimer_clock_t get_tx_slot_len(void) {
  if (packet_len_cached != packet_len) {
    packet_len_cached = packet_len;
    tx_slot_len_cached = ((packet_len + CC2420_TXTOPAYLOADBYTES) * F_CPU) / 31250 + GLOSSY_TX_TX_DELAY;
  }
  return tx_slot_len_cached;
}

rtimer_clock_t get_T_slot_h(void) {
  return T_slot_h;
}

rtimer_clock_t get_T_tx_h(void) {
  return T_tx_h;
}

int16_t get_s(uint8_t idx) {
  return s[idx];
}

rtimer_clock_t get_T_32_h(void) {
  return T_32_h;
}

uint8_t is_t_ref_l_updated(void) {
  return t_ref_l_updated;
}

void set_tx_times_updated(uint8_t updated) {
  tx_times_updated = updated;
}

rtimer_clock_t get_t_first_rx_l(void) {
  return t_first_rx_l;
}

rtimer_clock_t get_t_ref_l(void) {
  return t_ref_l;
}

void set_t_ref_l(rtimer_clock_t t) {
  t_ref_l = t;
}

void set_t_ref_l_updated(uint8_t updated) {
  t_ref_l_updated = updated;
}

uint8_t get_state(void) {
  return state;
}

int16_t get_measured_slack_time(void) {
  return measured_slack_time;
}

static inline void estimate_slot_length(rtimer_clock_t t_tx_start_tmp) {
  // estimate slot length if tx_cnt == 2
  if (tx_cnt == 2) {
    rtimer_clock_t T_slot_h_tmp = (t_tx_start_tmp - t_tx_start) - (packet_len * F_CPU) / 31250;
#if GLOSSY_SYNC_WINDOW
    T_slot_h_sum += T_slot_h_tmp;
    if ((++win_cnt) == GLOSSY_SYNC_WINDOW) {
      // update the slot length estimation
      T_slot_h = T_slot_h_sum / GLOSSY_SYNC_WINDOW;
      // halve the counters
      T_slot_h_sum /= 2;
      win_cnt /= 2;
    } else {
      if (win_cnt == 1) {
        // at the beginning, use the first estimation of the slot length
        T_slot_h = T_slot_h_tmp;
      }
    }
#else
    T_slot_h = T_slot_h_tmp;
#endif /* GLOSSY_SYNC_WINDOW */
  }
}

static inline void compute_sync_reference_time(void) {
  rtimer_clock_t T_rx_to_cap_h = sfd_cap_h - t_rx_start;
  unsigned long T_ref_to_rx_h = (GLOSSY_RELAY_CNT_FIELD - 1) * ((unsigned long)T_slot_h + (packet_len * F_CPU) / 31250);
  unsigned long T_ref_to_cap_h = T_ref_to_rx_h + (unsigned long)T_rx_to_cap_h;
  rtimer_clock_t T_ref_to_cap_l = 1 + T_ref_to_cap_h / CLOCK_PHI;
  // high-resolution offset of the reference time
  T_offset_h = (CLOCK_PHI - 1) - (T_ref_to_cap_h % CLOCK_PHI);
  // low-resolution value of the reference time
  t_ref_l = sfd_cap_l - T_ref_to_cap_l;
  // the reference time has been updated
  t_ref_l_updated = 1;
  if (sfd_cap_h - t_rx_start > dco_comp_threshold)
    sfd_cap_h-=dco_comp;
  t_ref_local_l = sfd_cap_l - 1 - (sfd_cap_h - t_rx_start - 1) / CLOCK_PHI; // CLOCK_PHI is F_CPU / RTIMER_SECOND = 128
  t_ref_local_h = CLOCK_PHI - 1 - (sfd_cap_h - t_rx_start - 1) % CLOCK_PHI;
}

/* ----------------------- Interrupt functions ---------------------- */
inline void glossy_begin_rx(void) {
  PIN_SET(FLOCKLAB_LED2);
  t_rx_start = TBCCR1;
  rx_try_cnt++;
  state = GLOSSY_STATE_RECEIVING;
  if (packet_len) {
    // Rx timeout: packet duration + 200 us
    // (packet duration: 32 us * packet_length, 1 DCO tick ~ 0.23 us)
    t_rx_timeout = t_rx_start + ((rtimer_clock_t)packet_len_tmp * 35 + 200) * 4;
  }
  READ_CAPTURE_NEXT_CLOCK_TICK(sfd_cap_h, sfd_cap_l);

  // wait until the FIFO pin is 1 (i.e., until the first byte is received)
  while (!FIFO_IS_1) {
    if (packet_len && !RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_abort_rx();
#if GLOSSY_DEBUG
      rx_timeout++;
#endif /* GLOSSY_DEBUG */
      return;
    }
  };
  // read the first byte (i.e., the len field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(GLOSSY_LEN_FIELD);
  // keep receiving only if it has the right length
  if ((packet_len && (GLOSSY_LEN_FIELD != packet_len_tmp))
      || (GLOSSY_LEN_FIELD < FOOTER_LEN) || (GLOSSY_LEN_FIELD > 127)) {
    // packet with a wrong length: abort packet reception
    radio_abort_rx();
#if GLOSSY_DEBUG
    bad_length++;
#endif /* GLOSSY_DEBUG */
    return;
  }
  bytes_read = 1;
  if (!packet_len) {
    packet_len_tmp = GLOSSY_LEN_FIELD;
    t_rx_timeout = t_rx_start + ((rtimer_clock_t)packet_len_tmp * 35 + 200) * 4;
  }

#if !COOJA
  // wait until the FIFO pin is 1 (i.e., until the second byte is received)
  while (!FIFO_IS_1) {
    if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_abort_rx();
#if GLOSSY_DEBUG
      rx_timeout++;
#endif /* GLOSSY_DEBUG */
      return;
    }
  }
  // read the second byte (i.e., the header field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(GLOSSY_HEADER_FIELD);
  // keep receiving only if it has the right header
  if ((GLOSSY_HEADER_FIELD & GLOSSY_HEADER_MASK) != GLOSSY_HEADER) {
    // packet with a wrong header: abort packet reception
    radio_abort_rx();
#if GLOSSY_DEBUG
    bad_header++;
#endif /* GLOSSY_DEBUG */
    return;
  }
  bytes_read = 2;
  
  // wait until the FIFO pin is 1 (i.e., until the third byte is received)
  while (!FIFO_IS_1) {
    if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_abort_rx();
#if GLOSSY_DEBUG
      rx_timeout++;
#endif /* GLOSSY_DEBUG */
      return;
    }
  }
  // read the third byte (i.e., the relay cnt field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(GLOSSY_RELAY_CNT_FIELD);
  bytes_read = 3;
  
  if (packet_len_tmp > 8) {
    // if packet is longer than 8 bytes, read all bytes but the last 8
    while (bytes_read <= packet_len_tmp - 8) {
      // wait until the FIFO pin is 1 (until one more byte is received)
      while (!FIFO_IS_1) {
        if (!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
          radio_abort_rx();
#if GLOSSY_DEBUG
          rx_timeout++;
#endif /* GLOSSY_DEBUG */
          return;
        }
      };
      // read another byte from the RXFIFO
      FASTSPI_READ_FIFO_BYTE(packet[bytes_read]);
      bytes_read++;
    }
  }
#endif /* COOJA */
  // set channel for next transmission slot
  CALC_NEXT_CHANNEL(n_timeouts + 1);
  if (!dohopping) {
    // round was started using random channel offset
    n_timeouts = GLOSSY_RELAY_CNT_FIELD + 1;
    // infer proper channel offset
    if (!channel_offset_adjusted) {
      glossy_channel_offset = (glossy_channel_offset + (GLOSSY_RELAY_CNT_FIELD/NUM_CHANNELVALUES + 1) * NUM_CHANNELVALUES - GLOSSY_RELAY_CNT_FIELD) % NUM_CHANNELVALUES;
      channel_offset_adjusted = 1;
    }
    next_tx_time_idx = n_timeouts;
  }
  else {
    next_tx_time_idx = n_timeouts + 1;
  }
  DBG_first_tx_time_idx = next_tx_time_idx;
  glossy_schedule_rx_timeout();
}

inline void glossy_end_rx(void) {
  rtimer_clock_t t_rx_stop_tmp = TBCCR1;
  // read the remaining bytes from the RXFIFO
  PIN_CLR(FLOCKLAB_LED2);
  FASTSPI_READ_FIFO_NO_WAIT(&packet[bytes_read], packet_len_tmp - bytes_read + 1);
  bytes_read = packet_len_tmp + 1;
#if COOJA
  if ((GLOSSY_CRC_FIELD & FOOTER1_CRC_OK) && ((GLOSSY_HEADER_FIELD & GLOSSY_HEADER_MASK) == GLOSSY_HEADER)) {
#else
  if (GLOSSY_CRC_FIELD & FOOTER1_CRC_OK) {
#endif /* COOJA */
    header = GLOSSY_HEADER_FIELD & ~GLOSSY_HEADER_MASK;
    // packet correctly received
    if (sync) {
      // increment relay_cnt field
      GLOSSY_RELAY_CNT_FIELD++;
    }
    if (tx_cnt >= tx_max) {
      // no more Tx to perform: stop Glossy
      radio_off();
      state = GLOSSY_STATE_OFF;
    } else {
      // write Glossy packet to the TXFIFO
      radio_write_tx();
      state = GLOSSY_STATE_RECEIVED;
    }
    if (rx_cnt == 0) {
      // first successful reception:
      // store current time and received relay counter
      t_first_rx_l = RTIMER_NOW();
      if (sync) {
        relay_cnt = GLOSSY_RELAY_CNT_FIELD - 1;
      }
    }
    rx_cnt++;
    t_rx_stop = t_rx_stop_tmp;
    if (initiator) {
      // a packet has been successfully received: stop the initiator timeout
      glossy_stop_initiator_timeout();
    }
    if (!packet_len) {
      packet_len = packet_len_tmp;
      data_len = (sync) ?
          packet_len_tmp - FOOTER_LEN - GLOSSY_RELAY_CNT_LEN - GLOSSY_HEADER_LEN :
          packet_len_tmp - FOOTER_LEN - GLOSSY_HEADER_LEN;
    }
    if ((!initiator) && (rx_cnt == 1)) {
      // copy the application data from the data field
      memcpy(data, &GLOSSY_DATA_FIELD, data_len);            
#if GLOSSY_INDICATE_EVT_DET
      // indicate that this node has received the event
      if(data[2] & 0x1) {
        PIN_SET(SINK_REPORT_PIN);
      } else {
        PIN_CLR(SINK_REPORT_PIN);
      }
#endif /* GLOSSY_INDICATE_EVT_DET */
    }
    PIN_SET(FLOCKLAB_LED3);
    if ((sync) && (T_slot_h) && (!t_ref_l_updated) && (rx_cnt)) {
      // compute the reference time after the first reception (higher accuracy)
      compute_sync_reference_time();
    }
    PIN_CLR(FLOCKLAB_LED3);
  } else {
#if GLOSSY_DEBUG
    bad_crc++;
#endif /* GLOSSY_DEBUG */
    // packet corrupted, abort the transmission before it actually starts
    radio_abort_tx();
    state = GLOSSY_STATE_WAITING;
  }
}

inline void glossy_begin_tx(void) {
  if (initiator && tx_cnt == 0) { // <--- second step of DCO compensation
    rtimer_clock_t calib_cap_h2, calib_cap_l2;
    CAPTURE_NEXT_CLOCK_TICK(calib_cap_h2, calib_cap_l2);
    dco_int_rx_h = calib_cap_h2 - calib_cap_h1;
    dco_int_rx_l = calib_cap_l2 - calib_cap_l1;
    CALC_DCO_COMPENSATION(dco_int_rx_h, dco_int_rx_l, dco_comp, dco_comp_threshold);
  }
  if (sync) {
    estimate_slot_length(TBCCR1);
  }
  t_tx_start = TBCCR1;
  state = GLOSSY_STATE_TRANSMITTING;
  tx_relay_cnt_last = GLOSSY_RELAY_CNT_FIELD;
    CAPTURE_NEXT_CLOCK_TICK(tx_try_times_h[tx_try_idx], tx_try_times_l[tx_try_idx]);
    tx_try_times_h[tx_try_idx] -= t_tx_start;
    tx_try_idx++;
  if (tx_times_updated)
    calculate_next_tx_time_idx();
    
  if (tx_cnt == 0 && !initiator) // first tx after non-hopping rx, prepare next channel for second tx
    CALC_NEXT_CHANNEL(relay_cnt + 2);
}

inline void glossy_end_tx(void) {
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  if (sync) {
    uint8_t n;
    // increment relay_cnt field
    GLOSSY_RELAY_CNT_FIELD++;
    FASTSPI_WRITE_RAM_LE(&GLOSSY_RELAY_CNT_FIELD,(&GLOSSY_RELAY_CNT_FIELD - packet),1,n);
  }
  t_tx_stop = TBCCR1;
  T_tx_h = t_tx_stop - t_tx_start;
  // stop Glossy if tx_cnt reached tx_max (and tx_max > 1 at the initiator)
  if ((++tx_cnt >= tx_max) && ((tx_max - initiator) > 0)) {
#if GLOSSY_RX_ONE_ON_INITIATOR
    if (initiator) {
      // wait for one more slot to receive transmission from next hop for message delay calculation
      glossy_set_channel_off_on(next_channel);
      // set rx timeout
      n_timeouts = GLOSSY_RELAY_CNT_FIELD;
      glossy_schedule_channel_timeout();
      state = GLOSSY_STATE_WAITING;
    }
    else {
#endif
      radio_off();
      state = GLOSSY_STATE_OFF;
      radio_flush_tx();
#if GLOSSY_RX_ONE_ON_INITIATOR
    }
#endif
  } else {
    CALC_NEXT_CHANNEL(GLOSSY_RELAY_CNT_FIELD+1);
  }
}

inline void glossy_set_channel(uint8_t cvidx) {
  FASTSPI_SETREG(CC2420_FSCTRL, channelvalues[cvidx]);
  
#if GLOSSY_SHOW_CHANNEL
  /* indicate current channel with pin */
  cvidx++;
  while (cvidx) {
    PIN_XOR(FLOCKLAB_INT1);
    PIN_XOR(FLOCKLAB_INT1);
    cvidx--;
  }
#endif /* GLOSSY_SHOW_CHANNEL */
}

inline void glossy_set_channel_off_on(uint8_t cvidx) {
  SPI_ENABLE();
  FASTSPI_TX_NOP(CC2420_SRFOFF);
  FASTSPI_TX_NOP(CC2420_FSCTRL);
  FASTSPI_TX_NOP((u8_t)(channelvalues[cvidx] >> 8));
  FASTSPI_TX_NOP((u8_t)channelvalues[cvidx]);
  FASTSPI_TX_NOP(CC2420_SRXON);
  SPI_WAITFORTx_ENDED();
  SPI_DISABLE();

#if GLOSSY_SHOW_CHANNEL
  /* indicate current channel with pin */
  cvidx++;
  while (cvidx) {
    PIN_XOR(FLOCKLAB_INT1);
    PIN_XOR(FLOCKLAB_INT1);
    cvidx--;
  }
#endif /* GLOSSY_SHOW_CHANNEL */
}

inline void glossy_set_channel_off(uint8_t cvidx) {
  SPI_ENABLE();
  FASTSPI_TX_NOP(CC2420_SRFOFF);
  FASTSPI_TX_NOP(CC2420_FSCTRL);
  FASTSPI_TX_NOP((u8_t)(channelvalues[cvidx] >> 8));
  FASTSPI_TX_NOP((u8_t)channelvalues[cvidx]);
  SPI_WAITFORTx_ENDED();
  SPI_DISABLE();

#if GLOSSY_SHOW_CHANNEL
  /* indicate current channel with pin */
  cvidx++;
  while (cvidx) {
    PIN_XOR(FLOCKLAB_INT1);
    PIN_XOR(FLOCKLAB_INT1);
    cvidx--;
  }
#endif /* GLOSSY_SHOW_CHANNEL */
}

/* ------------------------------ Timeouts -------------------------- */
inline void glossy_schedule_rx_timeout(void) {
  TBCCR5 = t_rx_timeout;
  TBCCTL5 = CCIE;
}

inline void glossy_stop_rx_timeout(void) {
  TBCCTL5 = 0;
}

inline void glossy_schedule_initiator_timeout(void) { }
#define MIN_TIMEOUT_SCHEDULE_TIME 2000
inline void glossy_schedule_channel_timeout_time(void) {
#if !COOJA
  PIN_SET(FLOCKLAB_LED3);
  rtimer_clock_t passed_time = RTIMER_NOW_DCO() - t_start + MIN_TIMEOUT_SCHEDULE_TIME;
  n_timeouts = (passed_time + (CC2420_TXTOPAYLOADBYTES * F_CPU) / 31250 + GLOSSY_CHANNEL_SWITCH_TIME_H) / ((unsigned long)T_slot_h + (packet_len * F_CPU) / 31250);
  glossy_schedule_channel_timeout();
  PIN_CLR(FLOCKLAB_LED3);
#endif
}

inline void glossy_schedule_channel_timeout(void) {
// #if !COOJA
  TBCCR4 = t_start + (n_timeouts+1) * ((unsigned long)T_slot_h + (packet_len * F_CPU) / 31250) - (CC2420_TXTOPAYLOADBYTES * F_CPU) / 31250 - GLOSSY_CHANNEL_SWITCH_TIME_H;
  TBCCTL4 = CCIE;
// #endif
}

inline void glossy_stop_initiator_timeout(void) {}
inline void glossy_stop_channel_timeout(void) {
  TBCCTL4 = 0;
}

void add_channel_to_history(uint8_t channelidx) {
  if (channel_history_count < GLOSSY_CHANNEL_HISTORY_SIZE) {
    channel_history[channel_history_count++] = channelidx;
  }
  else {
    channel_history[channel_history_pos++] = channelidx;
    if (channel_history_pos == GLOSSY_CHANNEL_HISTORY_SIZE)
      channel_history_pos = 0;
  }
}

uint8_t get_last_rx_channel(void) {
  if (channel_history_count == 0) {
    return 255;
  }
  if (channel_history_count < GLOSSY_CHANNEL_HISTORY_SIZE) {
    return channel_history[channel_history_count - 1];
  }
  if (channel_history_pos == 0) {
    return channel_history[GLOSSY_CHANNEL_HISTORY_SIZE - 1];
  }
  return channel_history[channel_history_pos - 1];
}

uint8_t get_good_random_channel() {
  uint8_t ret;
  // 50% random values, 50% from history
  if ((RTIMER_NOW_DCO() & 0x01) && (channel_history_count == GLOSSY_CHANNEL_HISTORY_SIZE)) {
    PIN_SET(FLOCKLAB_LED1);
    PIN_SET(FLOCKLAB_INT1);
    ret = channel_history[RTIMER_NOW_DCO() % GLOSSY_CHANNEL_HISTORY_SIZE];
    PIN_CLR(FLOCKLAB_LED1);
    PIN_CLR(FLOCKLAB_INT1);
  }
  else {
    ret = RTIMER_NOW_DCO() % NUM_CHANNELVALUES;
  }
  return ret;
}

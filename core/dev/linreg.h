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
 */
#include "contiki.h"
#include "vht.h"

// Linear Regression
#define TABLE_LEN  	8 // max 0xFFFF, should be fairly safe to go up to 100...
#define SUM_SHIFT 	(1LL << 18)
#define PERIOD_SHIFT 64 // power of 2
// VAR

extern uint32_t t_local_table[TABLE_LEN];
extern uint32_t t_global_table[TABLE_LEN];

extern vht_clock_t t_local_anchor;
extern vht_clock_t t_global_anchor;

extern vht_clock_t sum_t_ref_squared;
extern vht_clock_t sum_numerator;

extern vht_clock_t t_ref_avg;
extern vht_clock_t round_avg;

extern uint8_t sample_cnt;

// FUNC
void init_reg_tables(void);

void update_tables(vht_clock_t localtime, vht_clock_t globaltime);

vht_clock_t map_to_global(vht_clock_t t_local);

void map_n_to_global(vht_clock_t t_local, vht_clock_t period, uint8_t n, vht_clock_t * t_global_ptr);

uint8_t get_init_phase_done(void);

uint8_t time_mapping_available(void);
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
#include "linreg.h"

vht_clock_t t_local_anchor;
vht_clock_t t_global_anchor;

uint32_t t_local_table[TABLE_LEN] = {}; // stores values relative to t_local_anchor
uint32_t t_global_table[TABLE_LEN] = {}; // stores values relative to t_global_anchor

vht_clock_t t_ref_avg;
vht_clock_t round_avg;

vht_clock_t sum_t_ref;
vht_clock_t sum_round;

vht_clock_t sum_t_ref_squared;
vht_clock_t sum_numerator;

uint8_t sample_cnt = 0;
uint8_t init_done = 0;
uint8_t sums_ok = 0;

static vht_clock_t map_to_global_(vht_clock_t t_local, vht_clock_t * remainder);

/*
 * Sets all values in both tables to zero
 * 
 */
void init_reg_tables(void){
	uint8_t i;
	for (i=0; i<TABLE_LEN;i++){
		t_local_table[i] = 0;
		t_global_table[i] = 0;  
	}
	sum_t_ref = 0;
	sum_round = 0;
	sample_cnt = 0;
	init_done = 0;
}

/*
 * Adds a new reference time and flood-round number to the corresponding table
 * Additionally checks if the tables are full and need shifting or filled first otherwise
 * 
 * Assumes time samples are updated in ascending order
 */
void update_tables(vht_clock_t localtime, vht_clock_t globaltime){
	uint8_t i;
	vht_clock_t d_g, d_l;
	if (sample_cnt == 0) {
		t_local_anchor = localtime;
		t_global_anchor = globaltime;
	}
	if (sample_cnt < TABLE_LEN){
		// arrays not full yet
		sample_cnt += 1;
	}
	else {
		init_done = 1;
		d_l = t_local_table[1];
		d_g = t_global_table[1];
		t_local_anchor += d_l;
		t_global_anchor += d_g;
		sum_t_ref -= d_l * (sample_cnt-1);
		sum_round -= d_g * (sample_cnt-1);
		for (i = 0; i < TABLE_LEN - 1; i++){
			//shift t_ref array
			t_local_table[i] = t_local_table[i+1] - d_l;
			// shift round array
			t_global_table[i] = t_global_table[i+1] - d_g;
		}
	}
	t_local_table[sample_cnt - 1] =  localtime - t_local_anchor;
	t_global_table[sample_cnt - 1] =  globaltime - t_global_anchor;
	sum_t_ref += t_local_table[sample_cnt - 1];
	sum_round += t_global_table[sample_cnt - 1];
	sums_ok = 0;
}

/*
 * Maps a local timestamp to a global one by applying the following linear regression
 * 
 * t_global = avg(round) - avg(tref) + t_local + beta_star * (t_local - avg(t_ref))
 * beta_star = sum((tref_i - avg(tref))*(round_i - avg(round) - (tref_i - avg(tref))^2)/sum((tref_i - avg(tref))^2)
 * 
 */

vht_clock_t map_to_global(vht_clock_t t_local) {
  return map_to_global_(t_local, NULL);
}

static vht_clock_t map_to_global_(vht_clock_t t_local, vht_clock_t * remainder){
  uint8_t i;
  vht_clock_t t_global;
  long long m,n;
  
  if (sample_cnt <=2)
	  return t_local - t_local_anchor + t_global_anchor;
  if (!sums_ok) {
	sum_t_ref_squared = 0;
	sum_numerator = 0;
	
	t_ref_avg = (sum_t_ref + sample_cnt/2) / sample_cnt;
	round_avg = (sum_round + sample_cnt/2) / sample_cnt;
	for (i = 0; i < sample_cnt; i++){
		// what follows here is a more explicit way of computing this:
		// sum_t_ref_squared += (vht_clock_t)((((signed long long int)(t_local_table[i] - t_ref_avg)) * ((signed long long int)(t_local_table[i]  - t_ref_avg))) / SUM_SHIFT);
		// sum_numerator +=     (vht_clock_t)((((signed long long int)(t_local_table[i] - t_ref_avg)) * ((signed long long int)(t_global_table[i] - round_avg))) / SUM_SHIFT);
		// it seems as if msp430 gcc (4.6.3) has problems with 64 bit signed values
		if (t_local_table[i] > t_ref_avg)
			m = t_local_table[i] - t_ref_avg;
		else
			m = t_ref_avg - t_local_table[i];
		if (t_global_table[i] > round_avg)
			n = t_global_table[i] - round_avg;
		else
			n = round_avg - t_global_table[i];
		sum_t_ref_squared += (vht_clock_t)((m * m) / SUM_SHIFT);
		if (((t_local_table[i] > t_ref_avg) && (t_global_table[i] > round_avg))
			|| ((t_local_table[i] < t_ref_avg) && (t_global_table[i] < round_avg)))
			sum_numerator +=     (vht_clock_t)((m * n) / SUM_SHIFT);
		else
			sum_numerator -=     (vht_clock_t)((m * n) / SUM_SHIFT);
	}
	sums_ok = 1;
  }
  // apply simple linear regression
	t_global = round_avg + t_global_anchor + (t_local - (t_ref_avg + t_local_anchor));
	if (sum_t_ref_squared > sum_numerator) {
		m = ((long long)(sum_t_ref_squared - sum_numerator) * ((long long)(t_local - (t_ref_avg + t_local_anchor))));
		n = m / sum_t_ref_squared;
    if (remainder != NULL) {
      *remainder = ((m % sum_t_ref_squared) * PERIOD_SHIFT) / sum_t_ref_squared;
    }
		t_global -= n;
	} else {
		m = ((long long)(sum_numerator - sum_t_ref_squared) * ((long long)(t_local - (t_ref_avg + t_local_anchor))));
		n = m / sum_t_ref_squared;
    if (remainder != NULL) {
      *remainder = ((m % sum_t_ref_squared) * PERIOD_SHIFT) / sum_t_ref_squared;
    }
		t_global += n;
	}
	return t_global;
}

// map a sequence of n local values with equal distance to a global value
void map_n_to_global(vht_clock_t t_local, vht_clock_t period, uint8_t n, vht_clock_t * t_global_ptr){
  vht_clock_t sumperiod, remainder;
  int32_t step, sumstep;
  uint8_t i;
  // calculate sums (same as for a single value)
  // map first value
  remainder = 0;
  *t_global_ptr = map_to_global_(t_local, &remainder);
  
  if (n > 1) {
    // calculate step for each period (including fractions up to some precision)
    if (sample_cnt <=2) {
      step = 0;
      sumstep = 0;
    }
    else {
      if (sum_t_ref_squared > sum_numerator) {
        step = ((long long)period * (sum_t_ref_squared - sum_numerator) * PERIOD_SHIFT) / sum_t_ref_squared;
        step = -step;
        sumstep = -remainder;
      }
      else {
        step = ((long long)period * (sum_numerator - sum_t_ref_squared) * PERIOD_SHIFT) / sum_t_ref_squared;
        sumstep = remainder;
      }
    }
  
    // add this step to all further values
    sumperiod = 0;
    for (i=1;i<n;i++) {
      sumstep += step;
      sumperiod += period;
      *(t_global_ptr+i) = *t_global_ptr + sumperiod + sumstep / PERIOD_SHIFT;
    }
  }
}

uint8_t get_init_phase_done(void){
 return init_done;
}

uint8_t time_mapping_available(void) {
  return sample_cnt > 2;
}
/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#define LOGK_MODULE_ID "timm"

#include <string.h>

#include <mutek/printk.h>

#include <hexo/types.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/radio.h>
#include <ble/util/timing_mapper.h>

static uint32_t cu_tk(struct ble_timing_mapper_s *tm, uint32_t units);
static uint32_t cu_ww_tk(struct ble_timing_mapper_s *tm, uint32_t units);

error_t ble_timing_mapper_slave_init(struct ble_timing_mapper_s *tm,
                                     struct device_timer_s *timer,
                                     const struct ble_adv_connect_s *connect,
                                     dev_timer_value_t reference)
{
  dev_timer_delay_t conn_packet_tk;
  error_t err;

  err = ble_conn_timing_validate(&connect->timing);
  if (err)
    return err;

  memset(tm, 0, sizeof(*tm));

  err = device_copy_accessor(&tm->timer.base, &timer->base);
  if (err)
    return err;

  tm->master_sca = connect->sca;
  tm->pending = connect->timing;
  tm->last_event = 0;
  tm->update_instant = 0;
  tm->update_pending = 1;

  dev_timer_init_sec_round(&tm->timer, &tm->pending_timeout_tk,
                           NULL, tm->pending.timeout, 100);

  dev_timer_init_sec_ceil(&tm->timer, &tm->imprecision_tk,
                           NULL, 32, 1000000);

  dev_timer_init_sec_ceil(&tm->timer, &tm->min_conn_event_tk,
                          NULL, BLE_PACKET_TIME(CONFIG_BLE_PACKET_SIZE)
                          + 16 + BLE_T_IFS * 2, 1000000);

  dev_timer_init_sec_ceil(&tm->timer, &conn_packet_tk,
                          NULL, BLE_PACKET_TIME(34), 1000000);

  tm->pending_win_offset = connect->win_offset + 1;
  tm->pending_win_size = connect->win_size;

  tm->last_anchor = reference - tm->imprecision_tk + conn_packet_tk;
  tm->drops_at = tm->last_anchor + cu_tk(tm, tm->pending.interval * 7 + tm->pending_win_offset);

  return 0;
}

error_t ble_timing_mapper_master_init(struct ble_timing_mapper_s *tm,
                                      struct device_timer_s *timer,
                                      const struct ble_adv_connect_s *connect,
                                      dev_timer_value_t reference)
{
  dev_timer_delay_t conn_packet_tk;
  error_t err;

  err = ble_conn_timing_validate(&connect->timing);
  if (err)
    return err;

  memset(tm, 0, sizeof(*tm));

  err = device_copy_accessor(&tm->timer.base, &timer->base);
  if (err)
    return err;

  tm->master_sca = 0;
  tm->pending = connect->timing;
  tm->last_event = 0;
  tm->update_instant = 0;
  tm->update_pending = 1;
  tm->imprecision_tk = 0;

  dev_timer_init_sec_round(&tm->timer, &tm->pending_timeout_tk,
                           NULL, tm->pending.timeout, 100);

  dev_timer_init_sec_ceil(&tm->timer, &tm->min_conn_event_tk,
                          NULL, BLE_PACKET_TIME(CONFIG_BLE_PACKET_SIZE)
                          + 16 + BLE_T_IFS * 2, 1000000);

  dev_timer_init_sec_ceil(&tm->timer, &conn_packet_tk,
                          NULL, BLE_PACKET_TIME(34), 1000000);

  tm->pending_win_offset = connect->win_offset + 1;
  tm->pending_win_size = connect->win_size;

  tm->last_anchor = reference + conn_packet_tk;
  tm->drops_at = tm->last_anchor + cu_tk(tm, tm->pending.interval * 7 + tm->pending_win_offset);

  return 0;
}

void ble_timing_mapper_cleanup(struct ble_timing_mapper_s *tm)
{
  device_put_accessor(&tm->timer.base);
}

void ble_timing_mapper_event_set(struct ble_timing_mapper_s *tm,
                                 uint16_t event,
                                 dev_timer_value_t anchor)
{
  tm->last_event = event;
  tm->last_anchor = anchor;

  if (tm->update_pending) {
    int16_t after_update = tm->last_event - tm->update_instant;

    if (after_update >= 0) {
      tm->update_pending = 0;
      tm->current = tm->pending;
      tm->timeout_tk = tm->pending_timeout_tk;
    }
  }

  tm->drops_at = anchor + tm->timeout_tk;
}

void ble_timing_mapper_window_master_get(struct ble_timing_mapper_s *tm,
                                         uint16_t event,
                                         dev_timer_value_t *begin,
                                         dev_timer_value_t *end,
                                         dev_timer_delay_t *max_dur)
{
  uint32_t to_event_unit;
  uint16_t delta = event - tm->last_event;
  int16_t after_update = event - tm->update_instant;

  if (tm->update_pending && after_update >= 0) {
    to_event_unit = (delta - after_update) * tm->current.interval
      + tm->pending_win_offset
      + after_update * tm->pending.interval
      + tm->pending_win_size / 2;

    *max_dur = cu_tk(tm, tm->pending.interval);
  } else {
    to_event_unit = delta * tm->current.interval;

    *max_dur = cu_tk(tm, tm->current.interval);
  }

  *begin = tm->last_anchor + cu_tk(tm, to_event_unit);
  *end = 0;
}

void ble_timing_mapper_window_slave_get(struct ble_timing_mapper_s *tm,
                                        uint16_t event,
                                        dev_timer_value_t *begin,
                                        dev_timer_value_t *end,
                                        dev_timer_delay_t *max_dur)
{
  uint32_t to_event_unit;
  dev_timer_delay_t to_event_tk;
  dev_timer_delay_t transmit_window_tk;
  dev_timer_delay_t ww_tk;
  dev_timer_value_t theorical_begin;
  uint16_t delta = event - tm->last_event;
  int16_t after_update = event - tm->update_instant;

  if (tm->update_pending && after_update >= 0) {
    to_event_unit = (delta - after_update) * tm->current.interval
      + tm->pending_win_offset
      + after_update * tm->pending.interval;

    transmit_window_tk = cu_tk(tm, tm->pending_win_size);
    *max_dur = cu_tk(tm, tm->pending.interval);
  } else {
    to_event_unit = delta * tm->current.interval;

    transmit_window_tk = tm->min_conn_event_tk;
    *max_dur = cu_tk(tm, tm->current.interval);
  }

  to_event_tk = cu_tk(tm, to_event_unit);
  ww_tk = cu_ww_tk(tm, to_event_unit);

  theorical_begin = tm->last_anchor + to_event_tk;
  *begin = theorical_begin - ww_tk - tm->imprecision_tk;
  *end = theorical_begin + transmit_window_tk + ww_tk + tm->imprecision_tk;
}

error_t ble_timing_mapper_update_push(struct ble_timing_mapper_s *tm,
                                      const struct ble_conn_params_update *update)
{
  int16_t after_update = update->instant - tm->last_event;

  //logk_trace("Timing update instant %d, last event %d", update->instant, tm->last_event);

  if (after_update < 0)
    return -ETIMEDOUT;

  error_t err = ble_conn_timing_validate(&update->timing);
  if (err)
    return err;

  if (update->win_offset > update->timing.interval)
    return -EINVAL;

  if (update->win_size == 0)
    return -EINVAL;

  if (update->win_size > __MIN(8, update->timing.interval))
    return -EINVAL;

  tm->update_pending = 1;
  tm->pending = update->timing;
  tm->update_instant = update->instant;
  tm->pending_win_offset = update->win_offset;
  tm->pending_win_size = update->win_size;

  dev_timer_init_sec_round(&tm->timer, &tm->pending_timeout_tk,
                           NULL, tm->pending.timeout, 100);

  return 0;
}

/* Timing/data helpers */

__unused__
static uint32_t cu_tk_dynamic(struct ble_timing_mapper_s *tm, uint32_t units)
{
  dev_timer_delay_t tk;

  dev_timer_init_sec(&tm->timer, &tk, NULL,
                     units,
                     1000000 / BLE_T_CONN_UNIT / 2);

  return (tk + 1) / 2;
}

#if defined(CONFIG_BLE_SLEEP_CLOCK_HZ) && CONFIG_BLE_SLEEP_CLOCK_HZ == 32768
__unused__
static uint32_t cu_tk_32k(struct ble_timing_mapper_s *tm, uint32_t units)
{
  /*
    Do units * 32768 / 800 + .5
     = units * 40.96 + .5
     = units * 41 - units * .04 + .5
     = units * 41 - units * .00001010001111010111b + .1b
  */
  units <<= 12;
  uint32_t tmp = (units >> 5) + (units >> 7) + (units >> 10);

  return (units * 41 - tmp + (tmp >> 10) + (1 << 11)) >> 12;
}
#endif

#if defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
__unused__
static uint32_t cu_tk_fixed(struct ble_timing_mapper_s *tm, uint32_t units)
{
  return (units * CONFIG_BLE_SLEEP_CLOCK_HZ + 400) / 800;
}
#endif

static uint32_t cu_tk(struct ble_timing_mapper_s *tm, uint32_t units)
{
#if 0
  uint32_t tkd = cu_tk_dynamic(tm, units);
  uint32_t tk3 = cu_tk_32k(tm, units);
  uint32_t tkf = cu_tk_fixed(tm, units);

  logk_trace("%d CU ticks dynamic: %d, 32k: %d, fixed: %d\n",
             units,
             tkd, tk3, tkf);

  return __MAX(tkd, __MAX(tk3, tkf));
#endif

#if !defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
  return cu_tk_dynamic(tm, units);
#elif CONFIG_BLE_SLEEP_CLOCK_HZ == 32768
  return cu_tk_32k(tm, units);
#else
  return cu_tk_fixed(tm, units);
#endif
}

__attribute__((unused))
static uint32_t cu_ww_tk_dynamic(struct ble_timing_mapper_s *tm, uint32_t units)
{
  /**
     Nothing is known constantly, so compute everything the slow way.
     Get local clock accuracy, assume worst case, get PPMs for
     everyone, and widen accordingly.
   */
  dev_timer_delay_t tk;
  struct dev_timer_config_s config;

  DEVICE_OP(&tm->timer, config, &config, 0);

  dev_timer_init_sec(&tm->timer, &tk, NULL,
    units * (ble_sca_wc[tm->master_sca] + dev_freq_acc_ppb(&config.freq) / 1000),
    1000000000000ULL / BLE_T_CONN_UNIT);

  return tk;
}

#if defined(CONFIG_BLE_SLEEP_CLOCK_PPM)
/**
   At least local clock accuracy is known, but not its speed
   (assume variable divisor/PLL somewhere).
*/

__attribute__((unused))
static uint32_t cu_ww_tk_timer(struct ble_timing_mapper_s *tm, uint32_t units)
{
  dev_timer_delay_t tk;

  dev_timer_init_sec(&tm->timer, &tk, NULL,
    units * (ble_sca_wc[tm->master_sca] + CONFIG_BLE_SLEEP_CLOCK_PPM),
    1000000000000ULL / BLE_T_CONN_UNIT);

  return tk;
}
#endif

#if defined(CONFIG_BLE_SLEEP_CLOCK_PPM) && defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
/**
   tm_sca_ppm is compile time constant, master_sca_ppm is one out
   of 8 possible values, so precompute a table.

   In the table, a number of conn units that may pass before we have
   to increment window widening of one timer tick. This is stored with
   a shift amount of 8 bits for better precision.

   Final calculation is round((passed_cus << 8) / table[sca])
*/

__attribute__((unused))
static uint32_t cu_ww_tk_static(struct ble_timing_mapper_s *tm, uint32_t units)
{
# define UNITS_PER_WW_TICK_SHIFT 8
# define UNITS_PER_WW_TICK(master_sca_ppm)                              \
  (((1e12 * (1 << UNITS_PER_WW_TICK_SHIFT))                             \
    / ((double)BLE_T_CONN_UNIT * CONFIG_BLE_SLEEP_CLOCK_HZ              \
       * ((master_sca_ppm) + CONFIG_BLE_SLEEP_CLOCK_PPM))))

  static const uint32_t sca_units_per_ww_tick[] = {
    UNITS_PER_WW_TICK(500), UNITS_PER_WW_TICK(250),
    UNITS_PER_WW_TICK(150), UNITS_PER_WW_TICK(100),
    UNITS_PER_WW_TICK(75), UNITS_PER_WW_TICK(50),
    UNITS_PER_WW_TICK(30), UNITS_PER_WW_TICK(20),
  };

  uint32_t units_per_ww_tick = sca_units_per_ww_tick[tm->master_sca];
  uint32_t d = 1 << UNITS_PER_WW_TICK_SHIFT;
  uint32_t ret = 0;

  units += units_per_ww_tick >> (UNITS_PER_WW_TICK_SHIFT + 1);

  while (d && units) {
    while (units >= units_per_ww_tick) {
      ret += d;
      units -= units_per_ww_tick;
    }
    d >>= 1;
    units_per_ww_tick >>= 1;
  }

# undef UNITS_PER_WW_TICK
# undef UNITS_PER_WW_TICK_SHIFT

  return ret;
}
#endif

static uint32_t cu_ww_tk(struct ble_timing_mapper_s *tm, uint32_t units)
{
#if 0
  uint32_t wws = cu_ww_tk_static(tm, units);
  uint32_t wwt = cu_ww_tk_timer(tm, units);
  uint32_t wwd = cu_ww_tk_dynamic(tm, units);
  struct dev_timer_config_s config;

  DEVICE_OP(&tm->timer, config, &config, 0);

  logk_trace("%d CU WW clock acc: %dppm (%d ppm) static: %d, timer: %d, dynamic: %d",
         units,
         dev_freq_acc_ppb(&config.freq) / 1000,
         CONFIG_BLE_SLEEP_CLOCK_PPM,
         wws, wwt, wwd);

  //  assert(wws >= wwt && wwt >= wwd);

  return __MAX(wws, __MAX(wwt, wwd));
#endif

#if defined(CONFIG_BLE_SLEEP_CLOCK_PPM) && defined(CONFIG_BLE_SLEEP_CLOCK_HZ)
  return cu_ww_tk_static(tm, units);
#elif defined(CONFIG_BLE_SLEEP_CLOCK_PPM)
  return cu_ww_tk_timer(tm, units);
#else
  return cu_ww_tk_dynamic(tm, units);
#endif
}

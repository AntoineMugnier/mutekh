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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Institut Telecom / Telecom ParisTech (c) 2011
    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2011
*/

#include <time.h>
#include <mutek/timer.h>

static time_t rtc_base_time = 0;

static const timer_delay_t _timer_unit_scale = (timer_delay_t)(1/CONFIG_MUTEK_TIMER_UNIT);

static inline time_t convert_tu_sec(timer_delay_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT >= 1e0,
                               t * (timer_delay_t)(1/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1/CONFIG_MUTEK_TIMER_UNIT));
}

static inline uint64_t convert_tu_msec(timer_delay_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT >= 1e-3,
                               t * (timer_delay_t)(1000/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1000/CONFIG_MUTEK_TIMER_UNIT));
}

static inline uint64_t convert_tu_usec(timer_delay_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT >= 1e-6,
                               t * (timer_delay_t)(1000000/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1000000/CONFIG_MUTEK_TIMER_UNIT));
}

static inline uint64_t convert_tu_nsec(timer_delay_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT >= 1e-9,
                               t * (timer_delay_t)(1000000000/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1000000000/CONFIG_MUTEK_TIMER_UNIT));
}

static inline timer_delay_t convert_sec_tu(time_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT < 1e0,
                               t * (timer_delay_t)(1/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1/CONFIG_MUTEK_TIMER_UNIT));
}

static inline timer_delay_t convert_msec_tu(uint64_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT < 1e-3,
                               t * (timer_delay_t)(1000/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1000/CONFIG_MUTEK_TIMER_UNIT));
}

static inline timer_delay_t convert_usec_tu(uint64_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT < 1e-6,
                               t * (timer_delay_t)(1000000/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1000000/CONFIG_MUTEK_TIMER_UNIT));
}

static inline timer_delay_t convert_nsec_tu(uint64_t t)
{
  return __builtin_choose_expr(CONFIG_MUTEK_TIMER_UNIT < 1e-9,
                               t * (timer_delay_t)(1000000000/CONFIG_MUTEK_TIMER_UNIT),
                               t / (timer_delay_t)(1000000000/CONFIG_MUTEK_TIMER_UNIT));
}

error_t gettimeofday(struct timeval *tv, struct timezone *tz)
{
  timer_delay_t t = timer_get_tick(&timer_ms);

  tv->tv_sec = rtc_base_time + convert_tu_sec(t);
  tv->tv_usec = convert_tu_usec(t % (timer_delay_t)(1/CONFIG_MUTEK_TIMER_UNIT));

  return 0;
}

error_t settimeofday(const struct timeval *tv, const struct timezone *tz)
{
  rtc_base_time = tv->tv_sec;

  return 0;
}

time_t time(time_t *r_)
{
  timer_delay_t t = timer_get_tick(&timer_ms);
  time_t r = rtc_base_time + convert_tu_sec(t);

  if (r_)
    *r_ = r;

  return r;
}

error_t clock_getres(clockid_t clk_id, struct timespec *res)
{
  switch (clk_id)
    {
    case CLOCK_REALTIME:
      res->tv_sec = convert_tu_sec(1);
      if (CONFIG_MUTEK_TIMER_UNIT >= 1e-9)
        res->tv_nsec = convert_tu_nsec(1);
      else
        res->tv_nsec = 1;
      return 0;
    }

  return -1;
}

int clock_gettime(clockid_t clk_id, struct timespec *tp)
{
  switch (clk_id)
    {
    case CLOCK_REALTIME: {
      timer_delay_t t = timer_get_tick(&timer_ms);

      tp->tv_sec = rtc_base_time + convert_tu_sec(t);
      tp->tv_nsec = convert_tu_nsec(t % (timer_delay_t)(1/CONFIG_MUTEK_TIMER_UNIT));
      return 0;
    }
    }

  return -1;
}

int clock_settime(clockid_t clk_id, const struct timespec *tp)
{
  switch (clk_id)
    {
    case CLOCK_REALTIME:
      rtc_base_time = tp->tv_sec;
      return 0;
    }

  return -1;
}

#ifdef CONFIG_MUTEK_TIMER_EVENTS

/* unistd.h */
error_t usleep(uint_fast32_t usec)
{
  return timer_sleep(&timer_ms, convert_usec_tu(usec));
}

/* unistd.h */
error_t sleep(uint_fast32_t usec)
{
  return timer_sleep(&timer_ms, convert_sec_tu(usec));
}

error_t nanosleep(const struct timespec *rqtp, struct timespec *rmtp)
{
  return timer_sleep(&timer_ms, convert_sec_tu(rqtp->tv_sec) +
                                convert_nsec_tu(rqtp->tv_nsec));
}

#endif


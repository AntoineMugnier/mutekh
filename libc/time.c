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

#include <mutek/startup.h>
#include <mutek/printk.h>
#include <sys/time.h>
#include <time.h>

#include <device/class/timer.h>
#include <device/device.h>
#include <device/driver.h>

struct device_timer_s    libc_timer_dev = DEVICE_ACCESSOR_INIT;
static dev_timer_value_t libc_timer_offset;      // time value offset

static uint64_t          libc_time_sec_num;
static uint64_t          libc_time_sec_den;
static uint64_t          libc_time_usec_num;
static uint64_t          libc_time_usec_den;
static uint64_t          libc_time_nsec_num;
static uint64_t          libc_time_nsec_den;

static void adjust_frac(uint64_t *num, uint64_t *den)
{
  uint64_t a = *num, b = *den;

  // gcd
  while (b)
    {
      uint64_t t = b;
      b = a % b;
      a = t;
    }

  *num /= a;
  *den /= a;
}

void libc_time_initsmp()
{
  if (!cpu_isbootstrap())
    return;

  if (device_get_accessor_by_path(&libc_timer_dev, NULL,
                                  CONFIG_LIBC_TIMER_DEVICE_PATHS,
                                  DRIVER_CLASS_TIMER))
    {
      printk("error: libc: No entry found matching `"
             CONFIG_LIBC_TIMER_DEVICE_PATHS "' in the device tree.\n");
    }
  else
    {
      DEVICE_OP(&libc_timer_dev, start_stop, 1);

      if (!DEVICE_OP(&libc_timer_dev, get_value, &libc_timer_offset))
        {
          dev_timer_res_t r = 0;
          dev_timer_value_t m;
          if (!DEVICE_OP(&libc_timer_dev, resolution, &r, &m))
            {
              uint64_t freq = CONFIG_DEVICE_TIMER_DEFAULT_FREQ;
              if (!device_res_get_uint64(libc_timer_dev.dev, DEV_RES_FREQ, libc_timer_dev.number, &freq))
                freq >>= 24;

              if (freq)
                {
                  libc_time_sec_num = r;
                  libc_time_sec_den = freq;
                  adjust_frac(&libc_time_sec_num, &libc_time_sec_den);

                  libc_time_usec_num = 1000000ULL * r;
                  libc_time_usec_den = freq;
                  adjust_frac(&libc_time_usec_num, &libc_time_usec_den);

                  libc_time_nsec_num = 1000000000ULL * r;
                  libc_time_nsec_den = freq;
                  adjust_frac(&libc_time_nsec_num, &libc_time_nsec_den);

                  uint64_t w = m / libc_time_sec_den * libc_time_sec_num;

                  printk("libc: using timer device `%p' for libc time functions, wrapping period is %llu seconds\n", libc_timer_dev.dev, w);
                  return;
                }
            }

          DEVICE_OP(&libc_timer_dev, start_stop, 0);
        }

      printk("libc: unable to use `%p' timer device for libc time.\n", libc_timer_dev.dev);
      device_put_accessor(&libc_timer_dev);
    }
}

void libc_time_cleanupsmp()
{
  if (!cpu_isbootstrap())
    return;

  if (device_check_accessor(&libc_timer_dev))
    {
      DEVICE_OP(&libc_timer_dev, start_stop, 0);
      device_put_accessor(&libc_timer_dev);
    }
}

struct device_timer_s *libc_timer()
{
  return &libc_timer_dev;
}

error_t gettimeofday(struct timeval *tv, struct timezone *tz)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  dev_timer_value_t t;
  if (DEVICE_OP(&libc_timer_dev, get_value, &t))
    return -1;

  t += libc_timer_offset;
  t = (libc_time_usec_num * t) / libc_time_usec_den;

  tv->tv_sec  = t / 1000000;
  tv->tv_usec = t % 1000000;

  return 0;
}

error_t settimeofday(const struct timeval *tv, const struct timezone *tz)
{
  struct timespec tp;
  tp.tv_sec = tv->tv_sec;
  tp.tv_nsec = tv->tv_usec * 1000;

  return clock_settime(CLOCK_REALTIME, &tp);
}

time_t time(time_t *r_)
{
  if (!device_check_accessor(&libc_timer_dev))
    return (time_t)-1;

  dev_timer_value_t t;
  if (DEVICE_OP(&libc_timer_dev, get_value, &t))
    return (time_t)-1;

  t += libc_timer_offset;
  t = (libc_time_sec_num * t) / libc_time_sec_den;

  if (r_)
    *r_ = t;

  return t;
}

error_t clock_getres(clockid_t clk_id, struct timespec *res)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  switch (clk_id)
    {
    case CLOCK_REALTIME: {
      return 0;
      dev_timer_value_t t = libc_time_nsec_num / libc_time_nsec_den;
      res->tv_sec  = t / 1000000000;
      res->tv_nsec = t % 1000000000;
    }

    default:
      return -1;
    }
}

error_t clock_gettime(clockid_t clk_id, struct timespec *tp)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  switch (clk_id)
    {
    case CLOCK_REALTIME: {
      dev_timer_value_t t;
      if (DEVICE_OP(&libc_timer_dev, get_value, &t))
        return -1;

      t += libc_timer_offset;
      t = (libc_time_nsec_num * t) / libc_time_nsec_den;

      tp->tv_sec  = t / 1000000000;
      tp->tv_nsec = t % 1000000000;
      return 0;
    }

    default:
      return -1;
    }
}

error_t clock_settime(clockid_t clk_id, const struct timespec *tp)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  switch (clk_id)
    {
    case CLOCK_REALTIME: {
      dev_timer_value_t t;
      if (DEVICE_OP(&libc_timer_dev, get_value, &t))
        return -1;

      dev_timer_value_t v;
      libc_time_to_timer(tp, &v);

      libc_timer_offset = v - t;
      return 0;
    }

    default:
      return -1;
    }
}

error_t libc_time_to_timer(const struct timespec *delay, dev_timer_value_t *value)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  if (delay->tv_sec < 60)
    {
      uint64_t ns = 1000000000ULL * delay->tv_sec + delay->tv_nsec;
      *value = libc_time_nsec_den * ns / libc_time_nsec_num - libc_timer_offset;
    }
  else
    {
      uint64_t us = 1000000ULL * delay->tv_sec + delay->tv_nsec / 1000;
      *value = libc_time_usec_den * us / libc_time_usec_num - libc_timer_offset;
    }

  return 0;
}

/* unistd.h */
error_t usleep(uint_fast32_t usec)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  struct dev_timer_rq_s rq;
  rq.delay = libc_time_usec_den * usec / libc_time_usec_num;

  if (dev_timer_sleep(&libc_timer_dev, &rq)) 
    return -1;

  return 0;
}

/* unistd.h */
error_t sleep(uint_fast32_t sec)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  struct dev_timer_rq_s rq;
  rq.delay = libc_time_sec_den * sec / libc_time_sec_num;

  if (dev_timer_sleep(&libc_timer_dev, &rq))
    return -1;

  return 0;
}

error_t nanosleep(const struct timespec *rqtp, struct timespec *rmtp)
{
  if (!device_check_accessor(&libc_timer_dev))
    return -1;

  struct dev_timer_rq_s rq;
  uint64_t ns = 1000000000ULL * rqtp->tv_sec + rqtp->tv_nsec;
  rq.delay = libc_time_nsec_den * ns / libc_time_nsec_num;

  if (dev_timer_sleep(&libc_timer_dev, &rq))
    return -1;

  if (rmtp)
    rmtp->tv_sec = rmtp->tv_nsec = 0;

  return 0;
}


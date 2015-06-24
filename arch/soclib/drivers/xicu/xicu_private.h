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

    Copyright (c) UPMC, Lip6
         Alexandre Becoulet <alexandre.becoulet@lip6.fr>, 2006-2009
         Nicolas Pouillon <nipo@ssji.net>, 2009

*/

#ifndef __SOCLIB_XICU_PRIVATE_H_
#define __SOCLIB_XICU_PRIVATE_H_

#include <hexo/types.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU

#include <device/class/icu.h>

DEV_IRQ_EP_PROCESS(soclib_xicu_source_process);

struct soclib_xicu_sink_s
{
  struct dev_irq_ep_s sink;
  uint32_t     affinity;
  uint_fast8_t current;
  uint_fast8_t counter;
};
#endif

#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER

#include <device/class/timer.h>

void soclib_xicu_pti_irq_process(struct device_s *dev, uint_fast8_t number);

# define SOCLIB_XICU_PTI_MIN_PERIOD 2000
# define SOCLIB_XICU_PTI_DEFAULT_PERIOD 250000

struct soclib_xicu_pti_s
{
  /* bit 0 indicates if some requests are using the timer, other bits
     are start count. The start count is positive if the timer has
     been started in mode 1 and negative if the timer has been started
     in mode 0. */
  int_fast8_t start_count;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
  dev_timer_cfgrev_t rev;
  dev_request_pqueue_root_t queue;
  dev_timer_value_t value;
  dev_timer_res_t   period;
#endif
};
#endif

struct soclib_xicu_private_s
{
  uintptr_t addr;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
  uintptr_t hwi_count;
  struct soclib_xicu_sink_s *sinks;

  uintptr_t irq_count;
  struct dev_irq_ep_s *srcs;

# ifdef CONFIG_HEXO_IPI
  uintptr_t wti_count;
# endif
#endif

#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
  uintptr_t pti_count;
  struct soclib_xicu_pti_s pti[0];
#endif
};

DEV_USE(soclib_xicu_timer_use);

/******** hw registers */

#define XICU_WTI_REG 0
#define XICU_PTI_PER 1
#define XICU_PTI_VAL 2
#define XICU_PTI_ACK 3
#define XICU_MSK_PTI 4
#define XICU_MSK_PTI_ENABLE 5
#define XICU_MSK_PTI_DISABLE 6
#define XICU_PTI_ACTIVE 6
#define XICU_MSK_HWI 8
#define XICU_MSK_HWI_ENABLE 9
#define XICU_MSK_HWI_DISABLE 10
#define XICU_HWI_ACTIVE 10
#define XICU_MSK_WTI 12
#define XICU_MSK_WTI_ENABLE 13
#define XICU_MSK_WTI_DISABLE 14
#define XICU_WTI_ACTIVE 14
#define XICU_PRIO 15

#define XICU_REG_ADDR(base, op, id) (uintptr_t)((uint32_t*)(base) + ((op)<<5) + (id))

#define XICU_PRIO_WTI(val) (((val) >> 24) & 0x1f)
#define XICU_PRIO_HWI(val) (((val) >> 16) & 0x1f)
#define XICU_PRIO_PTI(val) (((val) >> 8) & 0x1f)
#define XICU_PRIO_HAS_WTI(val) ((val) & 0x4)
#define XICU_PRIO_HAS_HWI(val) ((val) & 0x2)
#define XICU_PRIO_HAS_PTI(val) ((val) & 0x1)

#define XICU_MAX_HWI 32
#define XICU_MAX_PTI 32

#endif


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

    Copyright (c) 2013 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#ifndef _ARM_DRIVER_M_H_
#define _ARM_DRIVER_M_H_

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/local.h>
#include <hexo/iospace.h>

#if defined(CONFIG_DEVICE)
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
#include <device/clock.h>
#include <device/irq.h>
#endif

DRIVER_PV(struct arm_dev_private_s
{
#if defined(CONFIG_DEVICE)
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_sink_s	sinks[CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT];
# if CONFIG_CPU_ARM32M_M_IRQ_COUNT != CONFIG_CPU_ARM32M_M_IRQ_MAPPED_COUNT
  const uint8_t *sink_mapping;
# endif
#endif

  struct cpu_tree_s node;

#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
  int_fast8_t dwt_cycnt_start;
#endif

#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  int_fast8_t systick_start;
  dev_request_pqueue_root_t systick_queue;
  dev_timer_value_t systick_value;
  dev_timer_res_t   systick_period;
# ifdef CONFIG_DEVICE_IRQ
  dev_timer_cfgrev_t systick_rev;
# endif
#endif

#if defined (CONFIG_CPU_ARM32M_TIMER_SYSTICK) \
  || defined(CONFIG_CPU_ARM32M_TIMER_DWTCYC)
  struct dev_freq_s freq;
#endif
#ifdef CONFIG_CPU_ARM32M_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
#endif
#endif
});

void arm_timer_systick_irq(struct device_s *dev);
error_t arm_timer_systick_use(const struct device_accessor_s *accessor,
                              enum dev_use_op_e op);

extern DEV_TIMER_REQUEST(arm_timer_request);
extern DEV_TIMER_CANCEL(arm_timer_cancel);
extern DEV_TIMER_GET_VALUE(arm_timer_get_value);
extern DEV_TIMER_CONFIG(arm_timer_config);

#define ARM_M_SYSTICK_MIN_PERIOD            10000

#define ARM_M_DWT_CTRL_ADDR                 0xe0001000
# define ARM_M_DWT_CTRL_NOCYCCNT            (1 << 25)
# define ARM_M_DWT_CTRL_CYCCNTENA           (1 << 0)
#define ARM_M_DWT_CYCCNT_ADDR               0xe0001004

#endif


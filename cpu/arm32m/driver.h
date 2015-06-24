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

#ifndef __MUTEK_ASM__

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/local.h>
#include <hexo/iospace.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
#include <device/class/clock.h>
#include <device/irq.h>

struct arm_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s	sinks[CONFIG_CPU_ARM32M_M_IRQ_COUNT];
#endif

  struct cpu_tree_s node;

#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  int_fast8_t systick_start;
  dev_request_pqueue_root_t systick_queue;
  dev_timer_value_t systick_value;
  dev_timer_res_t   systick_period;
# ifdef CONFIG_DEVICE_IRQ
  dev_timer_cfgrev_t systick_rev;
# endif
#endif
#ifdef CONFIG_CPU_ARM32M_TIMER_DWTCYC
  int_fast8_t dwt_cycnt_start;
#endif

  struct dev_freq_s freq;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
#endif
#ifdef CONFIG_CPU_ARM32M_TIMER_SYSTICK
  struct dev_freq_accuracy_s acc;
#endif
};

void arm_timer_systick_irq(struct device_s *dev);
DEV_USE(arm_timer_systick_use);

extern DEV_TIMER_REQUEST(arm_timer_request);
extern DEV_TIMER_CANCEL(arm_timer_cancel);
extern DEV_USE(arm_timer_systick_use);
extern DEV_TIMER_GET_VALUE(arm_timer_get_value);
extern DEV_TIMER_CONFIG(arm_timer_config);

#endif /* !defined(__MUTEK_ASM__) */

/* number of 32 bytes sub region of the MPU needed to cover
   CONFIG_CPU_ARM32M_MPU_STACK_GUARD_SIZE bytes */
#define ARM_M_STACK_GUARD_MPU_SUBREGION_COUNT \
  (CONFIG_CPU_ARM32M_MPU_STACK_GUARD_SIZE / 32)

/* number of region slots of the MPU needed to cover
   CONFIG_CPU_ARM32M_MPU_STACK_GUARD_SIZE bytes, taking overlapping into
   account. */
#define ARM_M_STACK_GUARD_MPU_REGION_COUNT \
  ((CONFIG_CPU_ARM32M_MPU_STACK_GUARD_SIZE / 32 + 6) / 8 + 1)

#define ARM_M_SYSTICK_MIN_PERIOD            10000

#define ARM_M_DWT_CTRL_ADDR                 0xe0001000
# define ARM_M_DWT_CTRL_NOCYCCNT            (1 << 25)
# define ARM_M_DWT_CTRL_CYCCNTENA           (1 << 0)
#define ARM_M_DWT_CYCCNT_ADDR               0xe0001004

#endif


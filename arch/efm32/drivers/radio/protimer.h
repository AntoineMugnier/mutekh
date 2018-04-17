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

   Copyright (c) 2017 Sebastien Cerdan <sebcerdan@gmail.com>

 */

#ifndef PROTIMER_H_
# define PROTIMER_H_

#include <hexo/iospace.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/class/timer.h>
#include <device/class/cmu.h>

#include <arch/efm32/devaddr.h>
#include <arch/efm32/efr/protimer.h>

/* Timer class parameters */
#define EFR32_PROTIMER_CHANNEL           0
#define EFR32_PRECNT_WIDTH               16
#define EFR32_PRECNT_MASK                ((1 << EFR32_PRECNT_WIDTH) - 1)
#define EFR32_BASECNT_WIDTH              16
#define EFR32_BASECNT_MASK               ((1 << EFR32_BASECNT_WIDTH) - 1)
#define EFR32_PROTIMER_HW_WIDTH          (32 + EFR32_BASECNT_WIDTH + EFR32_PRECNT_WIDTH)

struct efr32_protimer_s
{
#if EFR32_PROTIMER_HW_WIDTH < 64
  uint64_t                       swvalue;
#endif
  /* Timer queue */
  dev_request_pqueue_root_t      queue;
  dev_timer_cfgrev_t             rev;
  enum dev_timer_capabilities_e  cap:8;
  struct dev_clock_sink_ep_s     clk_ep;
};

void efr32_protimer_stop_counter(struct efr32_protimer_s *pv);
void efr32_protimer_start_counter(struct efr32_protimer_s *pv);
void efr32_protimer_init(struct efr32_protimer_s *pv);
void efr32_protimer_disable_compare(struct efr32_protimer_s *pv, uint8_t channel);

dev_timer_value_t efr32_protimer_get_value(struct efr32_protimer_s *pv);
bool_t efr32_protimer_request_start(struct efr32_protimer_s *pv, dev_timer_value_t value,
                                          dev_timer_value_t deadline, uint8_t channel);

#endif /* !PROTIMER_H_ */

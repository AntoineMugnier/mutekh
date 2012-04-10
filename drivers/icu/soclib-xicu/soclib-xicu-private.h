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

struct soclib_xicu_private_s
{
  uintptr_t addr;

  uintptr_t pti_count;

#ifdef CONFIG_HEXO_IRQ
  uintptr_t hwi_count;
  uintptr_t irq_count;
  struct dev_irq_ep_s *sinks;
  struct dev_irq_ep_s *srcs;
#endif

#ifdef CONFIG_HEXO_IPI
  uintptr_t wti_count;
#endif

};

#endif


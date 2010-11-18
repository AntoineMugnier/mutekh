/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with MutekH; if not, write to the Free Software Foundation,
  Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

  Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006
  Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009-2010
*/

#include <hexo/types.h>
#include <hexo/asm.h>
#include <hexo/cpu.h>

extern __ldscript_symbol_t __initial_stack;
extern void arm_boot();
extern void arm_exc_nmi();
extern void arm_exc_hfault();
extern void arm_exc_mpu();
extern void arm_exc_dabt();
extern void arm_exc_undef();
extern void arm_exc_swi();
extern void arm_exc_irq();
extern void arm_exc_tick();

typedef void (*isr_handler_t) ();

__attribute__((section(".boot"),used))
              static isr_handler_t cpu_boot[] =
  {
      &__initial_stack,
      arm_boot,
      arm_exc_nmi,
      arm_exc_hfault,
      arm_exc_mpu,
      arm_exc_dabt,
      arm_exc_undef,
      0,
      0,
      0,
      0,
      arm_exc_swi,
      0,
      0,
      arm_exc_swi,
      0,
      0,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      arm_exc_irq,
      };
        

// Local Variables:
// tab-width: 4;
// c-basic-offset: 4;
// indent-tabs-mode: nil;
// End:
//
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

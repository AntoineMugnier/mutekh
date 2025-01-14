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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/


#include <hexo/types.h>
#include <hexo/local.h>
#include <hexo/cpu.h>
#include <hexo/ipi.h>
#include <hexo/interrupt.h>

#include <device/class/icu.h>
#include <device/device.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>
#include <hexo/error.h>

#include <mutek/printk.h>

#include <device/device.h>
#include <hexo/atomic.h>

#define CONFIG_DRIVER_ICU_EMU_IRQCOUNT 32

struct icu_emu_handler_s
{
  dev_irq_t		*hndl;
  void			*data;
};

DRIVER_PV(struct icu_emu_private_s
{
  cpu_id_t cpuid;
  atomic_t mask;
  atomic_t pending;
  struct icu_emu_handler_s	table[CONFIG_DRIVER_ICU_EMU_IRQCOUNT];
});

#ifdef CONFIG_HEXO_IPI
DEV_ICU_SETUP_IPI_EP(icu_emu_setup_ipi_ep)
{
  endpoint->icu_dev = dev;
  endpoint->priv = (void*)(uintptr_t)ipi_no;

  return 0;
}

DEV_ICU_SENDIPI(icu_emu_sendipi)
{
  uint32_t dst_id = (uintptr_t)endpoint->priv;

  emu_interrupts_post(dst_id, 1);

  return 0;
}
#endif

DEV_ICU_ENABLE(icu_emu_enable)
{
  struct icu_emu_private_s	*pv = dev->drv_pv;

  if (irq >= CONFIG_DRIVER_ICU_EMU_IRQCOUNT)
    return -EINVAL;

  if (enable)
    {
      atomic_bit_set(&pv->mask, irq);

      if (atomic_bit_test(&pv->pending, irq))
        emu_interrupts_post(pv->cpuid, 0);
    }
  else
    {
      atomic_bit_clr(&pv->mask, irq);
    }

  return 0;
}

DEV_ICU_SETHNDL(icu_emu_sethndl)
{
  struct icu_emu_private_s	*pv = dev->drv_pv;
  struct icu_emu_handler_s	*h = pv->table + irq;

  if (irq >= CONFIG_DRIVER_ICU_EMU_IRQCOUNT)
    return -EINVAL;

  if (h->hndl)
    return -ENOMEM;

  h->hndl = hndl;
  h->data = data;

  return 0;
}

DEV_ICU_DELHNDL(icu_emu_delhndl)
{
  struct icu_emu_private_s	*pv = dev->drv_pv;
  struct icu_emu_handler_s	*h = pv->table + irq;

  if (irq >= CONFIG_DRIVER_ICU_EMU_IRQCOUNT)
    return -EINVAL;

  if (h->hndl != hndl)
    return -EINVAL;

  h->hndl = NULL;

  return 0;
}

static CPU_INTERRUPT_HANDLER(icu_emu_cpu_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(cpu_interrupt_handler_dev);
  struct icu_emu_private_s *pv = dev->drv_pv;

  switch (irq)
    {
    case 0: {
      uint32_t i;

      while ((i = atomic_get(&pv->mask) & atomic_get(&pv->pending)))
        {
          uint_fast8_t irq = ffs(i) - 1;
          struct icu_emu_handler_s *h = pv->table + irq;

          /* call interrupt handler */
          if (h->hndl)
            h->hndl(h->data);
          else
            printk("EMU: lost interrupt %i\n", irq);

          atomic_bit_clr(&pv->mask, irq);
        }
      break;
    }

#ifdef CONFIG_HEXO_IPI
    case 1:
      ipi_process_rq();
      break;
#endif
    }
}

#define icu_emu_use dev_use_generic

DRIVER_DECLARE(icu_emu_drv, 0, "Emu ICU", icu_emu,
               DRIVER_ICU_METHODS(icu_emu));

DRIVER_REGISTER(icu_emu_drv);

DEV_CLEANUP(icu_emu_cleanup)
{
  struct icu_emu_private_s	*pv = dev->drv_pv;

  mem_free(pv);
}

DEV_INIT(icu_emu_init)
{
  struct icu_emu_private_s	*pv;

  if (!(pv = mem_alloc(sizeof(*pv), mem_scope_sys)))
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  pv->cpuid = cpu_id();
  atomic_set(&pv->mask, 0);
  atomic_set(&pv->pending, 0);

  /* we are directly bound to cpu */
  cpu_interrupt_sethandler(icu_emu_cpu_handler);

  return 0;
}

/* for use by emu devices processes */
void icu_emu_post(struct device_s *source)
{
  struct device_s *dev = source->icudev;
  struct icu_emu_private_s *pv = dev->drv_pv;

  atomic_bit_set(&pv->pending, source->irq);

  emu_interrupts_post(pv->cpuid, 0);
}


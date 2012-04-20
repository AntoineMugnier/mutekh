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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include "xicu.h"
#include "xicu_private.h"

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU

static DEVICU_GET_SINK(soclib_xicu_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;

  if (icu_in_id >= pv->hwi_count)
    return NULL;

  /* enable hwi on output 0 FIXME SMP */
  cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_HWI_ENABLE, 0), endian_le32(1 << icu_in_id));

  return pv->sinks + icu_in_id;
}

static DEVICU_DISABLE_SINK(soclib_xicu_icu_disable_sink)
{
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(soclib_xicu_icu_setup_ipi_ep)
{
}
#endif

static DEV_IRQ_EP_PROCESS(soclib_xicu_source_process)
{
  struct dev_irq_ep_s  *src = ep;
  struct device_s *dev = ep->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t out = src - pv->srcs;

  assert(out < pv->irq_count);

  while (1)
    {
      uint32_t prio = endian_le32(cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PRIO, out)));

      if (!XICU_PRIO_HAS_HWI(prio))
        break;

      struct dev_irq_ep_s *sink = pv->sinks + XICU_PRIO_HWI(prio);
      sink->process(sink, id);
    }
}

const struct driver_icu_s  soclib_xicu_icu_drv =
{
  .class_         = DEVICE_CLASS_ICU,
  .f_get_sink     = soclib_xicu_icu_get_sink,
  .f_disable_sink = soclib_xicu_icu_disable_sink,
# ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = soclib_xicu_icu_setup_ipi_ep,
# endif
};

#endif

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
# warning FIXME implement XICU timer driver
#endif

/************************************************************************/

static const struct devenum_ident_s  soclib_xicu_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("soclib:vci_xicu"),
  { 0 }
};

const struct driver_s  soclib_xicu_drv =
{
  .desc           = "Soclib VciXicu",
  .id_table       = soclib_xicu_ids,

  .f_init         = soclib_xicu_init,
  .f_cleanup      = soclib_xicu_cleanup,

  .classes        = {
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
    &soclib_xicu_icu_drv,
#endif
    0
  }
};

DEV_INIT(soclib_xicu_init)
{
  struct soclib_xicu_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DRIVER_SOCLIB_XICU_TIMER
  device_get_param_uint_default(dev, "pti-count", &pv->pti_count, 0);
#endif

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU

  device_get_param_uint_default(dev, "irq-count", &pv->irq_count, 1);
  if (pv->irq_count)
    {
      /* init soclib_xicu irq source end-points */
      pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->irq_count, (mem_scope_sys));
      if (!pv->srcs)
        goto err_mem;

      device_irq_icu_source_init(dev, pv->srcs, pv->irq_count, &soclib_xicu_source_process);

      if (device_irq_source_link(dev, pv->srcs, pv->irq_count))
        goto err_mem2;
    }

  device_get_param_uint_default(dev, "hwi-count", &pv->hwi_count, 0);
  if (pv->hwi_count)
    {
      /* init soclib_xicu irq sink end-points */
      pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * pv->hwi_count, (mem_scope_sys));
      if (!pv->sinks)
        goto err_unlink;

      device_irq_sink_init(dev, pv->sinks, pv->hwi_count, NULL);
    }

# ifdef CONFIG_HEXO_IPI
  device_get_param_uint_default(dev, "wti-count", &pv->wti_count, 0);
# endif

#endif

  dev->drv = &soclib_xicu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
 err_unlink:
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);
 err_mem2:
  if (pv->srcs)
    mem_free(pv->sinks);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(soclib_xicu_cleanup)
{
  struct soclib_xicu_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  /* detach soclib_xicu irq end-points */
  device_irq_sink_unlink(dev, pv->sinks, pv->hwi_count);
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);

  if (pv->srcs)
    mem_free(pv->srcs);
  if (pv->sinks)
    mem_free(pv->sinks);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(soclib_xicu_drv);


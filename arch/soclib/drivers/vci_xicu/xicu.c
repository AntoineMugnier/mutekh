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

    Copyright (c) 2012 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2012 Institut Telecom / Telecom ParisTech

*/

#include "xicu_private.h"

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

static const struct devenum_ident_s  soclib_xicu_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("soclib:vci_xicu"),
  { 0 }
};

static DEV_INIT(soclib_xicu_init);
static DEV_CLEANUP(soclib_xicu_cleanup);

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
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
    &soclib_xicu_timer_drv,
#endif
    0
  }
};

static DEV_INIT(soclib_xicu_init)
{
  struct soclib_xicu_private_s  *pv;
  uint_fast8_t i;

  uintptr_t pti_count = 0;
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
  device_get_param_uint_default(dev, "pti-count", &pti_count, 0);

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  dev_timer_res_t resolution = SOCLIB_XICU_PTI_DEFAULT_PERIOD;
  device_get_param_uint(dev, "period", &resolution);
  if (resolution < SOCLIB_XICU_PTI_MIN_PERIOD)
    resolution = SOCLIB_XICU_PTI_MIN_PERIOD;
# endif
#endif

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv)
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
                 + pti_count * sizeof(struct soclib_xicu_pti_s)
#endif
                 , (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU

  device_get_param_uint_default(dev, "irq-count", &pv->irq_count, 1);
  if (pv->irq_count)
    {
      /* init soclib_xicu irq source end-points */
      pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->irq_count, (mem_scope_sys));
      if (!pv->srcs)
        goto err_mem;

      device_irq_source_init(dev, pv->srcs, pv->irq_count,
                             &soclib_xicu_source_process, DEV_IRQ_SENSE_HIGH_LEVEL);

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
      if (device_irq_source_link(dev, pv->srcs, pv->irq_count, -1))
        goto err_mem2;
# else
      if (device_irq_source_link(dev, pv->srcs, pv->irq_count, 0))
        goto err_mem2;
# endif
    }

  device_get_param_uint_default(dev, "hwi-count", &pv->hwi_count, 0);
  if (pv->hwi_count)
    {
      /* init soclib_xicu irq sink end-points */
      pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * pv->hwi_count, (mem_scope_sys));
      if (!pv->sinks)
        goto err_unlink;

      for (i = 0; i < pv->hwi_count; i++)
        {
          device_irq_sink_init(dev, &pv->sinks[i].sink, 1, DEV_IRQ_SENSE_HIGH_LEVEL);
          pv->sinks[i].affinity = 0;
        }
    }

# ifdef CONFIG_HEXO_IPI
  device_get_param_uint_default(dev, "wti-count", &pv->wti_count, 0);
# endif

# ifdef CONFIG_ARCH_SMP
  /* enable WTI 0 for all outputs, used to start cpus */
  for (i = 0; i < pv->irq_count; i++)
    cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_WTI_ENABLE, i), endian_le32(1));
# endif

#endif

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_TIMER
  pv->pti_count = pti_count;
  for (i = 0; i < pti_count; i++)
    {
      struct soclib_xicu_pti_s *p = pv->pti + i;
      p->start_count = 0;
      cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_ACK, i));
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, i), 0);

# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
      dev_timer_queue_init(&p->queue);
      p->period = resolution;
      p->value = 0;
      // FIXME timer irq routing
      cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_MSK_PTI_ENABLE, i), 0xffffffff);
# endif
    }
#endif

  dev->drv = &soclib_xicu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
 err_unlink:
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);
 err_mem2:
  if (pv->srcs)
    mem_free(pv->srcs);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_xicu_cleanup)
{
  struct soclib_xicu_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  /* detach soclib_xicu irq end-points */
  uint_fast8_t i;
  for (i = 0; i < pv->hwi_count; i++)
    device_irq_sink_unlink(dev, &pv->sinks[i].sink, 1);
  device_irq_source_unlink(dev, pv->srcs, pv->irq_count);

  if (pv->srcs)
    mem_free(pv->srcs);
  if (pv->sinks)
    mem_free(pv->sinks);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(soclib_xicu_drv);


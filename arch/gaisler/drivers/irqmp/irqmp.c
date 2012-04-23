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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech

*/

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/irq.h>

#include <assert.h>
#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
# define GAISLER_IRQMP_SINKS_COUNT 31
#else
# define GAISLER_IRQMP_SINKS_COUNT 15
#endif

struct gaisler_irqmp_private_s
{
  uintptr_t addr;

  struct dev_irq_ep_s *sinks;
  struct dev_irq_ep_s *srcs;
  uint_fast8_t srcs_count;
};

static DEVICU_GET_SINK(gaisler_irqmp_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;

  if (icu_in_id >= GAISLER_IRQMP_SINKS_COUNT)
    return NULL;

  /* enable hwi on output 0 FIXME SMP */
  cpu_mem_write_32(pv->addr + 0x40, endian_be32(2 << icu_in_id));

  return pv->sinks + icu_in_id;
};

static DEVICU_DISABLE_SINK(gaisler_irqmp_icu_disable_sink)
{
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(gaisler_irqmp_icu_setup_ipi_ep)
{
}
#endif

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
static DEV_IRQ_EP_PROCESS(gaisler_irqmp_source_process_eirq)
{
  struct device_s *dev = ep->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;
  uint_fast8_t irq = *id;

#ifdef CONFIG_ARCH_SMP
  struct dev_irq_ep_s  *src = ep;
  uint_fast8_t cpu = src - pv->srcs;
  assert(cpu < pv->srcs_count);
#else
  uint_fast8_t cpu = 0;
#endif

  if (irq < 15)
    {
      struct dev_irq_ep_s *sink;
      uint32_t eack = endian_be32(cpu_mem_read_32(pv->addr + 0xc0 + cpu * 4)) & 0x1f;

      assert(eack > 15);

      if (eack)
        sink = pv->sinks + eack - 1;
      else
        sink = pv->sinks + irq;
      int_fast16_t next_id = 0;
      sink->process(sink, &next_id);
    }
}
#endif

static DEV_IRQ_EP_PROCESS(gaisler_irqmp_source_process)
{
  struct device_s *dev = ep->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;
  uint_fast8_t irq = *id;

  if (irq < GAISLER_IRQMP_SINKS_COUNT)
    {
      struct dev_irq_ep_s *sink = pv->sinks + irq;
      int_fast16_t next_id = 0;
      sink->process(sink, &next_id);
    }
}

static const struct devenum_ident_s	gaisler_irqmp_ids[] =
{
  DEVENUM_GAISLER_ENTRY(0x01, 0x00d),
  { 0 }
};

const struct driver_icu_s  gaisler_irqmp_icu_drv =
{
  .class_         = DRIVER_CLASS_ICU,
  .f_get_sink     = gaisler_irqmp_icu_get_sink,
  .f_disable_sink = gaisler_irqmp_icu_disable_sink,
# ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = gaisler_irqmp_icu_setup_ipi_ep,
# endif
};

static DEV_CLEANUP(gaisler_irqmp_cleanup);
static DEV_INIT(gaisler_irqmp_init);

const struct driver_s  gaisler_irqmp_drv =
{
  .desc           = "Gaisler IRQMP irq controller",
  .id_table       = gaisler_irqmp_ids,

  .f_init         = gaisler_irqmp_init,
  .f_cleanup      = gaisler_irqmp_cleanup,

  .classes        = {
    &gaisler_irqmp_icu_drv,
    0
  }
};

REGISTER_DRIVER(gaisler_irqmp_drv);

static DEV_INIT(gaisler_irqmp_init)
{
  struct gaisler_irqmp_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  __attribute__((unused))
  uint32_t status = endian_be32(cpu_mem_read_32(pv->addr + 0x10));

#ifdef CONFIG_ARCH_SMP
  pv->srcs_count = (status >> 28) + 1; // number of cpus
#else
  pv->srcs_count = 1;
#endif

  /* init gaisler_irqmp irq source end-points */
  pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->srcs_count, (mem_scope_sys));
  if (!pv->srcs)
    goto err_mem;

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
  if (((status >> 16) & 0xf) != 0)
    device_irq_icu_source_init(dev, pv->srcs, pv->srcs_count, &gaisler_irqmp_source_process_eirq);
  else
#endif
    device_irq_icu_source_init(dev, pv->srcs, pv->srcs_count, &gaisler_irqmp_source_process);

  if (device_irq_source_link(dev, pv->srcs, pv->srcs_count))
    goto err_mem2;

#warning FIXME check error handling
  /* init gaisler_irqmp irq sink end-points */
  pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * GAISLER_IRQMP_SINKS_COUNT, (mem_scope_sys));
  if (!pv->sinks)
    goto err_unlink;

  device_irq_sink_init(dev, pv->sinks, GAISLER_IRQMP_SINKS_COUNT, NULL);


  dev->drv = &gaisler_irqmp_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_unlink:
  device_irq_source_unlink(dev, pv->srcs, pv->srcs_count);
 err_mem2:
  if (pv->srcs)
    mem_free(pv->sinks);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(gaisler_irqmp_cleanup)
{
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;

  /* detach gaisler_irqmp irq end-points */
  device_irq_sink_unlink(dev, pv->sinks, GAISLER_IRQMP_SINKS_COUNT);
  device_irq_source_unlink(dev, pv->srcs, pv->srcs_count);

  mem_free(pv->srcs);
  mem_free(pv->sinks);
  mem_free(pv);
}


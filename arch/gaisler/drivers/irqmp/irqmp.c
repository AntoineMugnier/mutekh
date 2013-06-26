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

#include <assert.h>
#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU

#include <device/class/icu.h>
#include <device/irq.h>

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
# define GAISLER_IRQMP_SINKS_COUNT 31
#else
# define GAISLER_IRQMP_SINKS_COUNT 15
#endif

struct gaisler_irqmp_sink_s
{
  struct dev_irq_ep_s sink;
  uint32_t     affinity;
  uint_fast8_t current;
  uint_fast8_t counter;
};

#endif

struct gaisler_irqmp_private_s
{
  uintptr_t addr;

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU
  struct gaisler_irqmp_sink_s *sinks;
  struct dev_irq_ep_s *srcs;
  uint_fast8_t srcs_count;
  uint_fast8_t eirq;
#endif
};

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU

static DEVICU_GET_ENDPOINT(gaisler_irqmp_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < GAISLER_IRQMP_SINKS_COUNT)
        return &pv->sinks[id].sink;
      return NULL;

    case DEV_IRQ_EP_SOURCE:
      if (id < pv->srcs_count)
        return pv->srcs + id;

    default:
      return NULL;
    }
};

static DEVICU_ENABLE_IRQ(gaisler_irqmp_icu_enable_irq)
{
  struct device_s *dev = idev->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;
  struct gaisler_irqmp_sink_s *xsink = (struct gaisler_irqmp_sink_s*)sink;
  uint_fast8_t icu_in_id = xsink - pv->sinks;

  if (irq_id > 0) // inputs are single wire, logical irq id must be 0
    return 0;

#ifdef CONFIG_CPU_SPARC
  if (icu_in_id == 14)
    {
      printk("irqmp: won't enable non maskable irq (line 15)\n");
      return 0;
    }
#endif

  if (pv->eirq && icu_in_id == pv->eirq - 1)
    {
      printk("irqmp: can't use eirq line as regular irq line\n");
      return 0;
    }

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
  if (icu_in_id >= 31 || (icu_in_id >= 15 && !pv->eirq))
#else
  if (icu_in_id >= 15)
#endif
    return 0;  

  uint32_t affinity = 0;
  uint_fast8_t i;

  for (i = 0; i < pv->srcs_count; i++)
    {
      struct dev_irq_ep_s *irqmp_src = pv->srcs + i;

      if (icu_in_id >= 15)
#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
        {
          /* irqmp can not be bypassed when using extended interrupt
             line because we need to read a register to find the
             actual raised irq line. eirq processor interrupt must be enabled. */
          if (!device_icu_irq_enable(irqmp_src, pv->eirq - 1, irqmp_src, dev_ep))
            continue;
        }
      else
#endif
        {
          /* request bypassing of irqmp, irq may be handled by src end-point directly. */
          if (!device_icu_irq_enable(irqmp_src, icu_in_id, src, dev_ep))
            continue;
        }

      affinity |= (1 << i);
    }

  if (!affinity)
    {
      printk("irqmp: found no source end-point which can relay interrupt for %p device\n", dev_ep->dev);
      return 0;
    }
  else if (xsink->affinity && affinity != xsink->affinity)
    { 
      xsink->affinity |= affinity;
      printk("irqmp: shared interrupt on sink %u will change affinity\n", icu_in_id);
    }
  else
    {
      xsink->affinity = affinity;
    }

#warning add IRQ load balance policy
  /* enable hwi on first affinity source */
  uint_fast8_t first = ffs(affinity) - 1;
  uintptr_t ra = pv->addr + 0x40 + 4 * first; // source mask register
  cpu_mem_write_32(ra, cpu_mem_read_32(ra) | endian_be32(2 << icu_in_id));

  return 1;
}

static DEVICU_DISABLE_IRQ(gaisler_irqmp_icu_disable_irq)
{
  struct device_s *dev = idev->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;
  struct gaisler_irqmp_sink_s *xsink = (struct gaisler_irqmp_sink_s*)sink;
  uint_fast8_t icu_in_id = xsink - pv->sinks;

  uint_fast8_t first = ffs(xsink->affinity) - 1;
  uintptr_t ra = pv->addr + 0x40 + 4 * first; // source mask register
  cpu_mem_write_32(ra, cpu_mem_read_32(ra) & ~endian_be32(2 << icu_in_id));
}

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
      struct gaisler_irqmp_sink_s *xsink;
      uint32_t eack = endian_be32(cpu_mem_read_32(pv->addr + 0xc0 + cpu * 4)) & 0x1f;

      if (eack)
        xsink = pv->sinks + eack - 1;
      else
        xsink = pv->sinks + irq;

      struct dev_irq_ep_s *sink = &xsink->sink;
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
      struct gaisler_irqmp_sink_s *xsink = pv->sinks + irq;
      struct dev_irq_ep_s *sink = &xsink->sink;
      int_fast16_t next_id = 0;
      sink->process(sink, &next_id);
    }
}

const struct driver_icu_s  gaisler_irqmp_icu_drv =
{
  .class_         = DRIVER_CLASS_ICU,
  .f_get_endpoint = gaisler_irqmp_icu_get_endpoint,
  .f_enable_irq = gaisler_irqmp_icu_enable_irq,
  .f_disable_irq = gaisler_irqmp_icu_disable_irq,
# ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = gaisler_irqmp_icu_setup_ipi_ep,
# endif
};

#endif /*  CONFIG_DRIVER_GAISLER_IRQMP_ICU */

static const struct devenum_ident_s	gaisler_irqmp_ids[] =
{
  DEVENUM_GAISLER_ENTRY(0x01, 0x00d),
  { 0 }
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
#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU
    &gaisler_irqmp_icu_drv,
#endif
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

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU

# ifdef CONFIG_ARCH_SMP
  pv->srcs_count = (status >> 28) + 1; // number of cpus
# else
  pv->srcs_count = 1;
# endif

  /* init gaisler_irqmp irq source end-points */
  pv->srcs = mem_alloc(sizeof(pv->srcs[0]) * pv->srcs_count, (mem_scope_sys));
  if (!pv->srcs)
    goto err_mem;

  pv->eirq = ((status >> 16) & 0xf);

# ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
#  ifdef CONFIG_CPU_SPARC
  if (pv->eirq == 15)
    {
      printk("irqmp: won't use non maskable irq (line 15) as eirq\n");
      pv->eirq = 0;
    }
#  endif

  if (pv->eirq)
    device_irq_source_init(dev, pv->srcs, pv->srcs_count, &gaisler_irqmp_source_process_eirq);
  else
# endif
    device_irq_source_init(dev, pv->srcs, pv->srcs_count, &gaisler_irqmp_source_process);

  cpu_mem_write_32(pv->addr + 0, 0);  // set level register

  if (device_irq_source_link(dev, pv->srcs, pv->srcs_count, 0))
    goto err_mem2;

  /* init gaisler_irqmp irq sink end-points */
  pv->sinks = mem_alloc(sizeof(pv->sinks[0]) * GAISLER_IRQMP_SINKS_COUNT, (mem_scope_sys));
  if (!pv->sinks)
    goto err_unlink;

  uint_fast8_t i;
  for (i = 0; i < GAISLER_IRQMP_SINKS_COUNT; i++)
    {
      device_irq_sink_init(dev, &pv->sinks[i].sink, 1);
      pv->sinks[i].affinity = 0;
    }
#endif

  dev->drv = &gaisler_irqmp_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU
 err_unlink:
  device_irq_source_unlink(dev, pv->srcs, pv->srcs_count);
 err_mem2:
  if (pv->srcs)
    mem_free(pv->srcs);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(gaisler_irqmp_cleanup)
{
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU
  /* detach gaisler_irqmp irq end-points */
  uint_fast8_t i;
  for (i = 0; i < GAISLER_IRQMP_SINKS_COUNT; i++)
    device_irq_sink_unlink(dev, &pv->sinks[i].sink, 1);

  device_irq_source_unlink(dev, pv->srcs, pv->srcs_count);

  mem_free(pv->srcs);
  mem_free(pv->sinks);
#endif
  mem_free(pv);
}

#ifdef CONFIG_ARCH_SMP

#include <mutek/startup.h>

/* find mask of processors from device tree */
static DEVICE_TREE_WALKER(irqmp_start_cpus_mask)
{
  uint32_t *mask = priv;

  if (dev->node.flags & DEVICE_FLAG_CPU &&
      dev->status == DEVICE_DRIVER_INIT_DONE)
    {
      uintptr_t maj, min;
      if (!device_res_get_uint(dev, DEV_RES_ID, 0, &maj, &min))
        *mask |= 1 << maj;
    }

  return 0;
}

void gaisler_irqmp_start_cpus()
{
  struct device_s *icu = device_get_by_path(NULL, "/icu");

  if (!icu || icu->drv != &gaisler_irqmp_drv)
    {
      printk("error: no default IRQMP device found to start other processors\n");
      return;
    }

  uint32_t mask = 0;
  device_tree_walk(NULL, &irqmp_start_cpus_mask, &mask);

  struct gaisler_irqmp_private_s *pv = icu->drv_pv;
  cpu_mem_write_32(pv->addr + 0x10, mask);
}

#endif


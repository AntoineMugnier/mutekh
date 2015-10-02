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
#include <device/resources.h>
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
  struct dev_irq_sink_s sink;
  uint_fast8_t          affinity;
};

#endif

struct gaisler_irqmp_private_s
{
  uintptr_t addr;

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU
  struct gaisler_irqmp_sink_s *sinks;
  struct dev_irq_src_s *srcs;
  uint_fast8_t srcs_count;
  uint_fast8_t eirq;
#endif
};

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_ICU

static DEV_ICU_GET_SINK(gaisler_irqmp_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;

  if (id < GAISLER_IRQMP_SINKS_COUNT)
    return &pv->sinks[id].sink;
  return NULL;
};

static DEV_ICU_LINK(gaisler_irqmp_icu_link)
{
  if (!route_mask || *bypass)
    return 0;

  struct device_s *dev = accessor->dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;
  struct gaisler_irqmp_sink_s *xsink = (struct gaisler_irqmp_sink_s*)sink;
  uint_fast8_t sink_id = xsink - pv->sinks;

#ifdef CONFIG_CPU_SPARC
  if (sink_id == 14)
    {
      printk("irqmp: won't enable non maskable irq (line 15)\n");
      return -EINVAL;
    }
#endif

  if (pv->eirq && sink_id == pv->eirq - 1)
    {
      printk("irqmp: can't use eirq line as regular irq line\n");
      return -EINVAL;
    }

  uint_fast8_t i = ffs(*route_mask) - 1;
  xsink->affinity = i;

  if (sink_id >= 15)
    {
#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
      if (!pv->eirq)
#endif
        return -ENOTSUP;
    }

  return 0;
}

static DEV_IRQ_SINK_UPDATE(gaisler_irqmp_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;
  struct gaisler_irqmp_sink_s *xsink = (struct gaisler_irqmp_sink_s*)sink;
  uint_fast8_t sink_id = xsink - pv->sinks;

  uintptr_t a = pv->addr + 0x40;
#ifdef CONFIG_ARCH_SMP
  a += 4 * xsink->affinity; // source mask register
#endif

  uint32_t x = cpu_mem_read_32(a);

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      x &= ~endian_be32(2 << sink_id);
      break;
    case DEV_IRQ_SENSE_RISING_EDGE:
      x |= endian_be32(2 << sink_id);
      break;
    default:
      return;
    }

  cpu_mem_write_32(a, x);
}

#ifdef CONFIG_DRIVER_GAISLER_IRQMP_EIRQ
static DEV_IRQ_SRC_PROCESS(gaisler_irqmp_source_process_eirq)
{
  struct device_s *dev = ep->base.dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  struct dev_irq_src_s  *src = ep;
  uint_fast8_t cpu = src - pv->srcs;
  assert(cpu < pv->srcs_count);
#else
  uint_fast8_t cpu = 0;
#endif

  if (id < 15)
    {
      struct gaisler_irqmp_sink_s *xsink;

      if (pv->eirq - 1 == id)
        {
          uint32_t eack = endian_be32(cpu_mem_read_32(pv->addr + 0xc0 + cpu * 4)) & 0x1f;
          xsink = pv->sinks + eack - 1;
        }
      else
        {
          xsink = pv->sinks + id;
        }

      struct dev_irq_sink_s *sink = &xsink->sink;
      device_irq_sink_process(sink, 0);
    }
}
#endif

static DEV_IRQ_SRC_PROCESS(gaisler_irqmp_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct gaisler_irqmp_private_s *pv = dev->drv_pv;

  if (id < 15)
    {
      struct gaisler_irqmp_sink_s *xsink = pv->sinks + id;
      struct dev_irq_sink_s *sink = &xsink->sink;
      device_irq_sink_process(sink, 0);
    }
}

#endif /*  CONFIG_DRIVER_GAISLER_IRQMP_ICU */

static DEV_CLEANUP(gaisler_irqmp_cleanup);
static DEV_INIT(gaisler_irqmp_init);

#define gaisler_irqmp_use dev_use_generic

DRIVER_DECLARE(gaisler_irqmp_drv, 0, "Gaisler IRQMP irq controller", gaisler_irqmp,
               DRIVER_ICU_METHODS(gaisler_irqmp_icu));

DRIVER_REGISTER(gaisler_irqmp_drv,
                DEV_ENUM_GAISLER_ENTRY(0x01, 0x00d));

static void gaisler_irqmp_disable_irqs(struct gaisler_irqmp_private_s *pv)
{
  uint_fast8_t i;
  for (i = 0; i < pv->srcs_count; i++)
    cpu_mem_write_32(pv->addr + 0x40 + 4 * i, 0);
}

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
    device_irq_source_init(dev, pv->srcs, pv->srcs_count,
                           &gaisler_irqmp_source_process_eirq);
  else
# endif
    device_irq_source_init(dev, pv->srcs, pv->srcs_count,
                           &gaisler_irqmp_source_process);

  //  gaisler_irqmp_disable_irqs(pv);

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
      device_irq_sink_init(dev, &pv->sinks[i].sink, 1, &gaisler_irqmp_sink_update,
                           DEV_IRQ_SENSE_RISING_EDGE);
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

  gaisler_irqmp_disable_irqs(pv);

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
  struct device_s *icu = NULL;
  
  if (device_get_by_path(&icu, "/icu", &device_filter_init_done) ||
      icu->drv != &gaisler_irqmp_drv)
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


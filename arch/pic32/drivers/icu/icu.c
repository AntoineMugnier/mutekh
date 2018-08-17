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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2015

*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <device/class/iomux.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/irq.h>

#include <arch/pic32/irq.h>
#include <arch/pic32/icu.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#define PIC32ICU_MAX_VECTOR	(191)

DRIVER_PV(struct pic32_icu_private_s
{
  uintptr_t addr;

  struct dev_irq_sink_s sinks[PIC32ICU_MAX_VECTOR];
  struct dev_irq_src_s src;
});

static DEV_ICU_GET_SINK(pic32_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct pic32_icu_private_s *pv = dev->drv_pv;

  if (id < PIC32ICU_MAX_VECTOR)
    return &pv->sinks[id];
  return NULL;
}

static DEV_IRQ_SINK_UPDATE(pic32_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct pic32_icu_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;

  uint8_t ridx = sink_id / 32;
  uint8_t bidx = sink_id % 32;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  /* Priority bit */
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_ICU_IPC_ADDR(sink_id / 4)));

  if (sense == DEV_IRQ_SENSE_NONE)
    {
      /* Disable and clear IRQ */
      cpu_mem_write_32(pv->addr + PIC32_ICU_IEC_CLR_ADDR(ridx), PIC32_ICU_IEC_EN(bidx));
      cpu_mem_write_32(pv->addr + PIC32_ICU_IFS_CLR_ADDR(ridx), PIC32_ICU_IFS_IRQ(bidx));
      PIC32_ICU_IPC_IP_SET(sink_id % 4, x, 0);
    }
  else
    {
      /* Clear and Enable IRQ */
      cpu_mem_write_32(pv->addr + PIC32_ICU_IFS_CLR_ADDR(ridx), PIC32_ICU_IFS_IRQ(bidx));
      cpu_mem_write_32(pv->addr + PIC32_ICU_IEC_SET_ADDR(ridx), PIC32_ICU_IEC_EN(bidx));
      PIC32_ICU_IPC_IP_SET(sink_id % 4, x, 1);
    }

  cpu_mem_write_32(pv->addr + PIC32_ICU_IPC_ADDR(sink_id / 4), x);
  sink->icu_pv = sense;

  switch (sink_id)
    {
      /* External interrupt */
      case 3:
      case 8:
      case 13:
      case 18:
      case 23:
        switch (sense)
          {
            case DEV_IRQ_SENSE_FALLING_EDGE:
            case DEV_IRQ_SENSE_RISING_EDGE:
              x = endian_le32(cpu_mem_read_32(pv->addr + PIC32_ICU_INTCON_ADDR));
              PIC32_ICU_INTCON_INTEP_SETVAL(sink_id / 5, x, sense == DEV_IRQ_SENSE_FALLING_EDGE ? 0 : 1);
              cpu_mem_write_32(pv->addr + PIC32_ICU_INTCON_ADDR, x);
            default:
              break;
          }
        break;
       default:
         break;
    }

  CPU_INTERRUPT_RESTORESTATE
}

static DEV_IRQ_SRC_PROCESS(pic32_icu_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct pic32_icu_private_s *pv = dev->drv_pv;

  while (1)
    {
      uint32_t status = endian_le32(cpu_mem_read_32(pv->addr + PIC32_ICU_INTSTAT_ADDR));

      uint8_t vnbr =  PIC32_ICU_INTSTAT_SIRQ_GET(status);

      uint32_t msk = endian_le32(cpu_mem_read_32(pv->addr + PIC32_ICU_IEC_ADDR(vnbr/32)));
      if (!(msk & PIC32_ICU_IEC_EN(vnbr%32)))
        return;

      struct dev_irq_sink_s *sink = pv->sinks + vnbr;

      if (sink->icu_pv & (DEV_IRQ_SENSE_RISING_EDGE | DEV_IRQ_SENSE_FALLING_EDGE))
          cpu_mem_write_32(pv->addr + PIC32_ICU_IFS_CLR_ADDR(vnbr/32), PIC32_ICU_IFS_IRQ(vnbr%32));

      device_irq_sink_process(sink, 0);

      if (sink->icu_pv & DEV_IRQ_SENSE_HIGH_LEVEL)
          cpu_mem_write_32(pv->addr + PIC32_ICU_IFS_CLR_ADDR(vnbr/32), PIC32_ICU_IFS_IRQ(vnbr%32));
    }
}

#define pic32_icu_use dev_use_generic
#define pic32_icu_link device_icu_dummy_link


static DEV_INIT(pic32_icu_init)
{
  struct pic32_icu_private_s  *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* setup pinmux */
  if (device_iomux_setup(dev, "<ei0? <ei1? <ei2? <ei3? <ei4?", NULL, NULL, NULL))
    goto err_mem;

  /* disable all irqs */
  for (uint8_t i = 0; i< PIC32_ICU_IEC_COUNT; i++)
    cpu_mem_write_32(pv->addr + PIC32_ICU_IEC_ADDR(i), 0);

  device_irq_source_init(dev, &pv->src, 1, &pic32_icu_source_process);
  if (device_irq_source_link(dev, &pv->src, 1, 0))
    goto err_mem;

  device_irq_sink_init(dev, pv->sinks, PIC32ICU_MAX_VECTOR,
                       &pic32_icu_sink_update, DEV_IRQ_SENSE_HIGH_LEVEL |
                                               DEV_IRQ_SENSE_RISING_EDGE |
                                               DEV_IRQ_SENSE_FALLING_EDGE);

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(pic32_icu_cleanup)
{
  struct pic32_icu_private_s *pv = dev->drv_pv;

  /* disable all irqs */
  for (uint8_t i = 0; i< PIC32_ICU_IEC_COUNT; i++)
    cpu_mem_write_32(pv->addr + PIC32_ICU_IEC_ADDR(i), 0);

  /* detach pic32_icu irq endpoints */
  device_irq_source_unlink(dev, &pv->src, 1);

  if (pv->sinks)
    mem_free(pv->sinks);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(pic32_icu_drv, 0, "PIC32 irq controller", pic32_icu,
               DRIVER_ICU_METHODS(pic32_icu));

DRIVER_REGISTER(pic32_icu_drv,
                DEV_ENUM_FDTNAME_ENTRY("pic32_icu"));


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
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/class/enum.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>

#include <arch/device_ids.h>

#include <string.h>
#include <stdio.h>

static DEV_ENUM_MATCH_DRIVER(apbctrl_match_driver)
{
  size_t i;

  for (i = 0; i < count; i++)
    {
      const struct dev_enum_ident_s *id = ident + i;

      if (id->type != DEV_ENUM_TYPE_GAISLER)
        continue;

      const struct dev_resource_s *rp = device_res_get(dev, DEV_RES_PRODUCT, 0);
      const struct dev_resource_s *rv = device_res_get(dev, DEV_RES_VENDOR, 0);

      if (rv && rp && rv->u.vendor.id == id->grlib.vendor &&
                      rp->u.product.id == id->grlib.device)
        return 1;
    }

  return 0;
}


#ifdef CONFIG_DEVICE_IRQ

/* Traverse device tree to add irq resource between irqmp and processors. */

struct apbctrl_scan_cpu_irq_ctx_s
{
  struct device_s *irqmp;
  uint_fast8_t count;
};


static DEVICE_TREE_WALKER(apbctrl_scan_cpu_irq)
{
  struct apbctrl_scan_cpu_irq_ctx_s *ctx = priv;

  if (dev->node.flags & DEVICE_FLAG_CPU)
    {
      uintptr_t maj, min;
      if (device_res_get_uint(dev, DEV_RES_ID, 0, &maj, &min))
        return 0;

#ifndef CONFIG_ARCH_SMP
      if (maj > 0)
        return 0;
#endif
      char buf[128];
      if (device_get_path(NULL, buf, sizeof(buf), &dev->node, 0) > 0)
        {
          if (device_res_add_irq(ctx->irqmp, maj, 0, 0, buf) == 0)
            ctx->count++;
        }
    }

  return 0;
}
#endif


/* Enumerate devices using Gaisler APB pnp records. */

static void apbctrl_scan(struct device_s *dev, uintptr_t begin)
{
  uint_fast8_t i;

  for (i = 0; i < 16; i++)
    {
      const uint32_t *p = (const uint32_t*)(begin + 0xff000 + 8 * i);

      uint8_t vendor = endian_be32(p[0]) >> 24;

      if (!vendor)
        continue;

      uint16_t device = (endian_be32(p[0]) >> 12) & 0xfff;
      uint8_t version = (endian_be32(p[0]) >> 5) & 0x1f; 

      uint32_t start = begin + (((endian_be32(p[1]) >> 20) & 0xfff) << 8);
      uint32_t mask = ~(((endian_be32(p[1]) >> 4) & 0xfff) << 8) & 0xfffff;

      struct device_s *d = NULL;

      const char *name = NULL;

      /* some specific device node create */
      if (vendor == GAISLER_VENDOR_GAISLER)
        {
          if (device < GAISLER_DEVICE_count)
            name = gaisler_devices_names[device];

          switch (device)
            {
            case GAISLER_DEVICE_IRQMP:
#ifdef CONFIG_DEVICE_IRQ
              d = device_alloc(5 + CONFIG_ARCH_LAST_CPU_ID + 1);
#else
              d = device_alloc(5);                               
#endif
              if (!d)
                continue;

#ifdef CONFIG_DEVICE_IRQ
              struct apbctrl_scan_cpu_irq_ctx_s ctx = { d, 0 };
              /* add irq links from IRQMP to cpus */
              device_tree_walk(NULL, &apbctrl_scan_cpu_irq, &ctx);
              if (ctx.count == 0)
                printk("apbctrl: no processor found to link to irqmp device.\n");
#endif

              break;

#ifdef CONFIG_DEVICE_IRQ
              /* GPTIMER may have multiple irq lines, not described in pnp table.
                 This hack is needed to properly repport timer irqs in device tree. */
            case GAISLER_DEVICE_GPTIMER: {
              uint32_t gpt_cfg = endian_be32(cpu_mem_read_32(start + 0x8));
              uint_fast8_t nirq = gpt_cfg & 0x100 ? gpt_cfg & 7 : 1;
              d = device_alloc(4 + nirq);
              if (!d)
                continue;

              uint8_t j, irq = endian_be32(p[0]) & 0x1f;
              for (j = 0; j < nirq; j++)
                device_res_add_irq(d, j, irq + j - 1, 0, "/icu");
            }
#endif
            }
        }

      /* default device node create */
      if (!d)
        {
          d = device_alloc(5);
          if (!d)
            continue;
#ifdef CONFIG_DEVICE_IRQ
          uint8_t irq = endian_be32(p[0]) & 0x1f;
          if (irq)
            device_res_add_irq(d, 0, irq - 1, 0, "/icu");
#endif
        }

      if (name)
        {
          uint_fast8_t i = 0;
          char n[16];
          struct device_s *tmp;
          do {
            sprintf(n, "%s%u", name, i++);
            tmp = dev;
          } while (!device_get_by_path(&tmp, n, NULL));

          device_set_name(d, n);
        }

#ifdef CONFIG_GAISLER_DEVICE_IDS
      if (vendor < GAISLER_VENDOR_count &&
          gaisler_vendors_longnames[vendor])
        device_res_add_vendor(d, vendor, gaisler_vendors_longnames[vendor]);
      else
#endif
        device_res_add_vendor(d, vendor, NULL);

#ifdef CONFIG_GAISLER_DEVICE_IDS
      if (vendor == GAISLER_VENDOR_GAISLER &&
          device < GAISLER_DEVICE_count &&
          gaisler_devices_longnames[device])
        device_res_add_product(d, device, gaisler_devices_longnames[device]);
      else
#endif
        device_res_add_product(d, device, NULL);

      device_res_add_revision(d, version, 0);

      if (mask & (mask+1))
        {
          printk("apbctrl: %p device address mask with non contiguous range is not supported\n", d);
          d->status = DEVICE_ENUM_ERROR;
        }
      else
        {
          switch (endian_be32(p[1]) & 0xf)
            {
            case 1:
              device_res_add_mem(d, start, start + mask + 1);
              break;
            default:
              ;
            }
        }

      d->enum_dev = dev;
      device_attach(d, dev);
      device_shrink(d);

      if (vendor == GAISLER_VENDOR_GAISLER)
        {
          switch (device)
            {
            case GAISLER_DEVICE_GPTIMER:
            case GAISLER_DEVICE_APBUART:
            case GAISLER_DEVICE_IRQMP: {
              struct device_s *tmp = NULL;
              if (device_get_by_path(&tmp, gaisler_devices_names[device], NULL))
                device_new_alias_to_node(NULL, gaisler_devices_names[device], &d->node);
              break;
            }

            }
        }

    }
}

DEV_CLEANUP(apbctrl_cleanup);
DEV_INIT(apbctrl_init);

#define apbctrl_use dev_use_generic

DRIVER_DECLARE(apbctrl_drv, "Gaisler APB bus controller", apbctrl,
               DRIVER_ENUM_METHODS(apbctrl));

DRIVER_REGISTER(apbctrl_drv,
                DEV_ENUM_GAISLER_ENTRY(GAISLER_VENDOR_GAISLER, GAISLER_DEVICE_APBMST));

DEV_INIT(apbctrl_init)
{
  dev->status = DEVICE_DRIVER_INIT_FAILED;
  dev->drv = &apbctrl_drv;

  uintptr_t begin, end;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &begin, &end))
    return -EINVAL;

  if (end - begin < 0x100000) // range needed for pnp data
    return -EINVAL;

  apbctrl_scan(dev, begin);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

  return -1;
}

DEV_CLEANUP(apbctrl_cleanup)
{
}


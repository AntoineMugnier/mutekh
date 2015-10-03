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

static DEV_ENUM_MATCH_DRIVER(ahbctrl_match_driver)
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

static void ahbctrl_scan(struct device_s *dev, uintptr_t begin, uintptr_t end)
{
  uint_fast8_t i, j;

  for (i = 0; i < 128; i++)
    {
      const uint32_t *p = (const uint32_t*)(begin + 32 * i);

      if ((uintptr_t)p >= end || (uintptr_t)p >= 0xfffffff0)
        break;

      uint8_t vendor = endian_be32(p[0]) >> 24;

      if (!vendor)
        continue;

      uint16_t device = (endian_be32(p[0]) >> 12) & 0xfff;
      uint8_t version = (endian_be32(p[0]) >> 5) & 0x1f; 

      struct device_s *d = device_alloc(12);
      if (!d)
        continue;

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

      /* processor case */
      if (vendor == GAISLER_VENDOR_GAISLER)
        {
          char n[16];
          switch (device)
            {
            case GAISLER_DEVICE_LEON3:
            case GAISLER_DEVICE_LEON3FT:
            case GAISLER_DEVICE_LEON4: {
              device_res_add_id(d, i, 0);
              d->node.flags |= DEVICE_FLAG_CPU;

              sprintf(n, "cpu%u", i);
              device_set_name(d, n);
              break;
            }
            default: {
              if (device >= GAISLER_DEVICE_count)
                break;

              const char *name = gaisler_devices_names[device];
              if (!name)
                break;

              uint_fast8_t i = 0;
              struct device_s *tmp;
              do {
                sprintf(n, "%s%u", name, i++);
                tmp = dev;
              } while (!device_get_by_path(&tmp, n, NULL));
                  
              device_set_name(d, n);
            }
            }
        }

#ifdef CONFIG_DEVICE_IRQ
      uint8_t irq = endian_be32(p[0]) & 0x1f;

      if (irq)
        {
          device_res_add_dev_param(d, "icu", "/icu");
          device_res_add_irq(d, 0, irq - 1, DEV_IRQ_SENSE_RISING_EDGE, 0, 1);
        }
#endif

      uint8_t mtype = 0;
      for (j = 4; j < 8; j++)
        {
          uint8_t type = endian_be32(p[j]) & 0xf;
          mtype |= type;

          if (!type)
            continue;

          uintptr_t start = ((endian_be32(p[j]) >> 20) & 0xfff) << 20;
          uint32_t mask = (endian_be32(p[j]) >> 4) & 0xfff;

          if (!mask)
            continue;

          mask = ~(mask << 20);

          if (mask & (mask+1))
            {
              printk("ahbctrl: %p device address mask with non contiguous range is not supported\n", d);
              d->status = DEVICE_ENUM_ERROR;
            }
          else
            {
              switch (type)
                {
                case 1:
                case 3:
                  device_res_add_io(d, start, start + mask + 1);
                  break;
                case 2:
                  device_res_add_mem(d, start, start + mask + 1);
                }
            }
        }

      /* skip entries without mem/io resource */
      if (!mtype && !(d->node.flags & DEVICE_FLAG_CPU))
        {
          mem_free(d);
          continue;
        }

#ifdef CONFIG_DRIVER_GAISLER_AHBCTRL_USERPARAMS
      for (j = 1; j < 4; j++)
        {
          char pname[16];
          sprintf(pname, "user_param_%u", j);
          device_res_add_uint_param(d, pname, endian_be32(p[j]));
        }
#endif

      d->enum_dev = dev;
      device_shrink(d);
      device_attach(d, dev);
    }
}

static DEV_CLEANUP(ahbctrl_cleanup);
static DEV_INIT(ahbctrl_init);

#define ahbctrl_use dev_use_generic

DRIVER_DECLARE(ahbctrl_drv, DRIVER_FLAGS_EARLY_INIT, "Gaisler AHB controller", ahbctrl,
               DRIVER_ENUM_METHODS(ahbctrl));

DRIVER_REGISTER(ahbctrl_drv);

static DEV_INIT(ahbctrl_init)
{
  dev->status = DEVICE_DRIVER_INIT_FAILED;
  dev->drv = &ahbctrl_drv;

  uintptr_t begin, end;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &begin, &end))
    return -EINVAL;

  ahbctrl_scan(dev, begin, end);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

  return -1;
}

static DEV_CLEANUP(ahbctrl_cleanup)
{
}


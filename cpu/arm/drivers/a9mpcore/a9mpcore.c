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

    Copyright (c) 2013 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2013 Institut Telecom / Telecom ParisTech

*/


#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/class/enum.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <string.h>
#include <stdio.h>

#include "a9mpcore_scu_regs.h"

#define A9MPCORE_SCU_ADDR          0x0000
#define A9MPCORE_INT_CTRL_ADDR     0x0100
#define A9MPCORE_GLOBAL_TIMER_ADDR 0x0200
#define A9MPCORE_PRIV_TIMER_ADDR   0x0600
#define A9MPCORE_INT_DISTRIB_ADDR  0x1000

struct a9mpcore_private_s
{
  uintptr_t addr;
};

static DEVENUM_MATCH_DRIVER(a9mpcore_match_driver)
{
  return 0;
}

static DEV_CLEANUP(a9mpcore_cleanup);
static DEV_INIT(a9mpcore_init);

static const struct driver_enum_s a9mpcore_enum_drv =
{
  .class_	= DRIVER_CLASS_ENUM,
  .f_match_driver = a9mpcore_match_driver,
};

const struct driver_s	a9mpcore_drv =
{
  .desc         = "ARM Cortex-A9 MPCore",
  .f_init	= a9mpcore_init,
  .f_cleanup	= a9mpcore_cleanup,
  .classes	= { &a9mpcore_enum_drv, 0 }
};

REGISTER_DRIVER(a9mpcore_drv);

static DEV_INIT(a9mpcore_init)
{
  struct a9mpcore_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  uint32_t scu_cfg = endian_le32(cpu_mem_read_32(pv->addr + A9MPCORE_SCU_ADDR + A9MPCORE_SCU_CFG_ADDR));
  uint_fast8_t i, cpu_count = A9MPCORE_SCU_CFG_CPU_NUM_GET(scu_cfg) + 1;

#ifdef CONFIG_DRIVER_ARM_A9MPCORE_IRQ
  /* add interrupt controller distributor */
  struct device_s *icu = device_alloc(cpu_count + 2);
  assert(icu != NULL);

  device_set_name(icu, "icu");

  device_res_add_mem(icu, pv->addr + 0x1000, pv->addr + 0x2000); // gic distributor
  device_res_add_mem(icu, pv->addr + 0x0100, pv->addr + 0x0200); // gic cpu interface

  icu->enum_dev = dev;
  device_attach(icu, dev);

  extern const struct driver_s pl390_icu_drv;
  device_bind_driver(icu, &pl390_icu_drv);
#endif

  /* add processors */
  for (i = 0; i < cpu_count; i++)
    {
      struct device_s *d = device_alloc(1);
      assert(d != NULL);

      device_res_add_id(d, i, 0);
      d->node.flags |= DEVICE_FLAG_CPU;

      char name[16];
      sprintf(name, "../cpu%u", i);
      device_set_name(d, name + 3);

      d->enum_dev = dev;
      device_attach(d, dev);

#ifdef CONFIG_DRIVER_ARM_A9MPCORE_IRQ
      device_res_add_irq(icu, i, 0, 0, name);
#endif

      extern const struct driver_s arm_drv;
      device_bind_driver(d, &arm_drv);
    }

  dev->drv = &a9mpcore_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(a9mpcore_cleanup)
{
  struct a9mpcore_private_s *pv = dev->drv_pv;

  mem_free(pv);
}


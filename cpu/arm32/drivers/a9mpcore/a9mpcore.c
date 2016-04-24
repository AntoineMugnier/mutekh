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
#include <device/resources.h>
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
  dev_request_queue_root_t queue;
};

static DEV_ENUM_MATCH_DRIVER(a9mpcore_match_driver)
{
  return 0;
}

static DEV_ENUM_REQUEST(a9mpcore_request)
{
  struct device_s *dev = accessor->dev;
  struct a9mpcore_private_s *pv = dev->drv_pv;

  return dev_drv_enum_request_generic(&pv->queue, dev, rq);
}

static DEV_ENUM_CANCEL(a9mpcore_cancel)
{
  struct device_s *dev = accessor->dev;
  struct a9mpcore_private_s *pv = dev->drv_pv;

  return dev_drv_enum_cancel_generic(&pv->queue, dev, rq);
}

static DEV_USE(a9mpcore_use)
{
  switch (op)
    {
    case DEV_USE_ENUM_CHILD_INIT: {
      struct device_s *cdev = param;
      struct device_s *dev = (void*)cdev->node.parent;
      struct a9mpcore_private_s *pv = dev->drv_pv;
      dev_drv_enum_child_init(&pv->queue, cdev);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_CLEANUP(a9mpcore_cleanup);
static DEV_INIT(a9mpcore_init);

DRIVER_DECLARE(a9mpcore_drv, DRIVER_FLAGS_EARLY_INIT, "ARM Cortex-A9 MPCore", a9mpcore,
               DRIVER_ENUM_METHODS(a9mpcore));

DRIVER_REGISTER(a9mpcore_drv);

static DEV_INIT(a9mpcore_init)
{
  struct a9mpcore_private_s *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  dev_request_queue_init(&pv->queue);

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  uint32_t scu_cfg = endian_le32(cpu_mem_read_32(pv->addr + A9MPCORE_SCU_ADDR + A9MPCORE_SCU_CFG_ADDR));
  uint_fast8_t i, cpu_count = A9MPCORE_SCU_CFG_CPU_NUM_GET(scu_cfg) + 1;

#ifdef CONFIG_DRIVER_ARM_A9MPCORE_IRQ
  /* add interrupt controller distributor */
  struct device_s *icu = device_alloc(cpu_count * 2 + 2);
  assert(icu != NULL);

  device_set_name(icu, "icu");

  device_res_add_mem(icu, pv->addr + 0x1000, pv->addr + 0x2000); // gic distributor
  device_res_add_mem(icu, pv->addr + 0x0100, pv->addr + 0x0200); // gic cpu interface

  extern const struct driver_s pl390_icu_drv;
  device_attach(icu, dev, &pl390_icu_drv);
#endif

  struct dev_freq_s freq;
  error_t has_freq = device_get_res_freq(dev, &freq, 0);

  /* add processors */
  for (i = 0; i < cpu_count; i++)
    {
      struct device_s *d = device_alloc(2);
      assert(d != NULL);

      device_res_add_id(d, i, 0);
      d->node.flags |= DEVICE_FLAG_CPU;

      if (has_freq == 0)
        device_res_add_freq(d, &freq);

      char name[16];
      sprintf(name, "../cpu%u", i);
      device_set_name(d, name + 3);

#ifdef CONFIG_DRIVER_ARM_A9MPCORE_IRQ
      device_res_add_dev_param(icu, "icu", name, DRIVER_CLASS_ICU);
      device_res_add_irq(icu, i, 0, DEV_IRQ_SENSE_LOW_LEVEL, 0, 0);
#endif

      extern const struct driver_s arm32_drv;
      device_attach(d, dev, &arm32_drv);
    }

  return 0;
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(a9mpcore_cleanup)
{
  struct a9mpcore_private_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  mem_free(pv);

  return 0;
}


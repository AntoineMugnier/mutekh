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

#include <hexo/error.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>

error_t device_res_id(const struct device_s *dev,
                      enum dev_resource_type_e type,
                      uint_fast8_t id, uint_fast8_t *res)
{
  uint_fast8_t i;

  for (i = 0; i < dev->res_count; i++)
    if (dev->res[i].type == type && !id--)
      {
        *res = i;
        return 0;
      }

  return -ENOENT;
}

struct dev_resource_s *device_res_get(struct device_s *dev,
                                      enum dev_resource_type_e type,
                                      uint_fast8_t id)
{
  uint_fast8_t i;

  for (i = 0; i < dev->res_count; i++)
    if (dev->res[i].type == type && !id--)
      return dev->res + i;

  return NULL;
}

error_t device_res_get_uint(const struct device_s *dev,
                            enum dev_resource_type_e type,
                            uint_fast8_t id, uintptr_t *res)
{
  uint_fast8_t i;

  for (i = 0; i < dev->res_count; i++)
    if (dev->res[i].type == type && !id--)
      {
        *res = dev->res[i].uint;
        return 0;
      }

  return -ENOENT;
}

struct dev_resource_s * device_res_add(struct device_s *dev)
{
  uint_fast8_t i;

  if (dev->status != DEVICE_NO_DRIVER)
    return NULL;

  for (i = 0; i < dev->res_count; i++)
    {
      struct dev_resource_s *r = dev->res + i;

      if (r->type == DEV_RES_UNUSED)
        return r;
    }

  return NULL;
}

error_t device_res_add_io(struct device_s *dev, uintptr_t start, uintptr_t end)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_IO;
  r->io.start = start;
  r->io.end = end;

  return 0;
}

error_t device_res_add_mem(struct device_s *dev, uintptr_t start, uintptr_t end)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_MEM;
  r->mem.start = start;
  r->mem.end = end;

  return 0;
}

error_t device_res_add_irq(struct device_s *dev, uint_fast16_t dev_out_id,
                           uint_fast16_t icu_in_id, struct device_s *icu)
{
#ifdef CONFIG_HEXO_IRQ
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_IRQ;
  r->irq.dev_out_id = dev_out_id;
  r->irq.icu_in_id = icu_in_id;
  r->irq.icu = icu;

  icu->ref_count++;

  return 0;
#else
  return -EINVAL;
#endif
}

error_t device_res_add_id(struct device_s *dev, uintptr_t major, uintptr_t minor)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_ID;
  r->id.major = major;
  r->id.minor = minor;

  return 0;
}

error_t device_res_add_vendorid(struct device_s *dev, uintptr_t id, const char *name)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_VENDORID;
  r->vendor.id = id;
  r->vendor.name = name;

  return 0;
}

error_t device_res_add_productid(struct device_s *dev, uintptr_t id, const char *name)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_PRODUCTID;
  r->product.id = id;
  r->product.name = name;

  return 0;
}

error_t device_res_add_str_param(struct device_s *dev, const char *name, const char *value)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_STR_PARAM;
  r->str_param.name = name;
  r->str_param.value = value;

  return 0;
}

error_t device_res_add_uint_param(struct device_s *dev, const char *name, uintptr_t value)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_UINT_PARAM;
  r->uint_param.name = name;
  r->uint_param.value = value;

  return 0;
}

error_t device_res_add_uint_array_param(struct device_s *dev, const char *name, uintptr_t *value)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_UINT_ARRAY_PARAM;
  r->uint_array_param.name = name;
  r->uint_array_param.value = value;

  return 0;
}


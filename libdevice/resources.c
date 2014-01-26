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

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>

#include <string.h>

struct dev_resource_s *device_res_get(const struct device_s *dev,
                                      enum dev_resource_type_e type,
                                      uint_fast8_t id)
{
  DEVICE_RES_FOREACH(dev, r, {
      if (r->type == type && !id--)
        return r;
  });

  return NULL;
}

struct dev_resource_s *device_res_get_from_name(const struct device_s *dev,
                                                enum dev_resource_type_e type,
                                                uint_fast8_t id, const char *name)
{
  DEVICE_RES_FOREACH(dev, r, {
      if (r->type == type && r->u.uint[1] &&
          !strcmp(name, (const char*)r->u.uint[1]) && !id--)
        return r;
  });

  return NULL;
}

error_t device_res_alloc(struct device_s *dev, struct dev_resource_s **res,
                         enum dev_resource_type_e type)
{
  if (dev->status == DEVICE_DRIVER_INIT_DONE)
    return -EBUSY;

  struct dev_resource_table_s *tbl, **tbl_;
  uint_fast8_t i;
  for (tbl_ = &dev->res_tbl; *tbl_ != NULL; tbl_ = &tbl->next)
    {
      tbl = *tbl_;

      if (tbl->flags & DEVICE_RES_TBL_FLAGS_STATIC_CONST)
        return -EPERM;

      for (i = 0; i < tbl->count; i++)
        {
          struct dev_resource_s *r = &tbl->table[i];
          if (r->type == DEV_RES_UNUSED)
            {
              r->type = type;
              *res = r;
              return 0;
            }
        }
    }

  const size_t s = 5;
  tbl = mem_alloc(sizeof(struct dev_resource_table_s)
                  + sizeof(struct dev_resource_s) * s, (mem_scope_sys));
  if (tbl == NULL)
    return -ENOMEM;

  *tbl_ = tbl;
  tbl->next = NULL;
  tbl->flags = DEVICE_RES_TBL_FLAGS_ALLOCATED;
  tbl->count = s;

  memset(tbl->table, 0, sizeof(struct dev_resource_s) * s);
  tbl->table[0].type = type;
  *res = &tbl->table[0];

  return 0;
}

void device_res_cleanup(struct dev_resource_s *r)
{
  if (r->flags & DEVICE_RES_FLAGS_FREE_PTR0)
    mem_free((void*)r->u.uint[0]);
  if (r->flags & DEVICE_RES_FLAGS_FREE_PTR1)
    mem_free((void*)r->u.uint[1]);
  r->flags = 0;
  r->type = DEV_RES_UNUSED;
}

error_t device_res_alloc_str(struct device_s *dev,
                             enum dev_resource_type_e type,
                             const char *a, const char *b,
                             struct dev_resource_s **r_)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, type);
  if (err)
    return err;

  if (a != NULL)
    {
      if (!(a = strdup(a)))
        return -ENOMEM;
    }

  if (b != NULL)
    {
      if (!(b = strdup(b)))
        {
          mem_free((void *)a);
          return -ENOMEM;
        }

      r->flags |= DEVICE_RES_FLAGS_FREE_PTR1;
      r->u.uint[1] = (uintptr_t)b;
    }

  if (a != NULL)
    {
      r->flags |= DEVICE_RES_FLAGS_FREE_PTR0;
      r->u.uint[0] = (uintptr_t)a;
    }

  if (r_)
    *r_ = r;
  return 0;
}

error_t device_res_add_uint_array_param(struct device_s *dev, const char *name, uintptr_t *value)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, DEV_RES_UINT_ARRAY_PARAM);
  if (err)
    return err;

  name = strdup(name);
  if (!name)
    return -ENOMEM;

  uintptr_t i;
  uintptr_t *v = mem_alloc(sizeof(uintptr_t) * (value[0] + 1), mem_scope_sys);

  if (!v)
    {
      mem_free((void*)name);
      return -ENOMEM;
    }

  for (i = 0; i <= value[0]; i++)
    v[i] = value[i];
  value = v;

  r->flags = DEVICE_RES_FLAGS_FREE_PTR0 | DEVICE_RES_FLAGS_FREE_PTR1;
  r->u.uint_array_param.name = name;
  r->u.uint_array_param.value = value;

  return 0;
}

error_t device_get_param_dev_accessor(struct device_s *dev,
                                      const char *name, void *accessor,
                                      enum driver_class_e cl)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_DEV_PARAM, 0, name)))
    return -ENOENT;

  return device_get_accessor_by_path(accessor, &dev->node, (const char*)r->u.uint[0], cl);
}

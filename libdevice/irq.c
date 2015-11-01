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

    Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2012
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech

*/

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/icu.h>
#include <device/class/enum.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <hexo/endian.h>
#include <stdlib.h>
#include <string.h>

#include <assert.h>

extern inline error_t device_irq_src_enable(struct dev_irq_src_s *src);

extern inline void device_irq_sink_process(struct dev_irq_sink_s *sink, dev_irq_id_t id);

#ifdef CONFIG_DEVICE_RESOURCE_ALLOC
extern inline error_t device_res_add_irq(struct device_s *dev, uint_fast8_t src_id,
                                         uint_fast8_t sink_id, enum dev_irq_sense_modes_e trig_mode,
                                         dev_irq_id_t irq_id, dev_irq_route_t route_mask);
#endif

static DEV_IRQ_SRC_PROCESS(device_irq_dummy_process)
{
  printk("device: spurious interrupt\n");
}

static const struct dev_irq_src_s device_irq_dummy_src_ep = {
  .process = device_irq_dummy_process
};

static void device_irq_sink_cleanup(struct dev_irq_sink_s *sink)
{
  if (!sink->base.link_count)
    {
      if (sink->sense_all != DEV_IRQ_SENSE_ID_BUS)
        sink->update(sink, DEV_IRQ_SENSE_NONE, 0);
      sink->base.links.single = (void*)&device_irq_dummy_src_ep;
    }
}

/****************************************/

static error_t device_irq_ep_link_half(struct dev_irq_ep_s *a, struct dev_irq_ep_s *b)
{
#if defined(CONFIG_DEVICE_IRQ_SHARING) || defined(CONFIG_DEVICE_IRQ_MULTI_SINK)
  uint_fast8_t c = a->link_count;

  switch (c)
    {
    case 0:
#endif
      a->links.single = b;
      a->link_count = 1;
      return 0;

#if defined(CONFIG_DEVICE_IRQ_SHARING) || defined(CONFIG_DEVICE_IRQ_MULTI_SINK)
    case 1: {
      struct dev_irq_ep_s **r, *t = a->links.single;
      r = mem_alloc(sizeof(*r) * 2, (mem_scope_sys));
      if (!r)
        return -ENOMEM;
      r[0] = t;
      r[1] = b;
      a->links.array = r;
      a->link_count = 2;
      return 0;
    }

    default: {
      struct dev_irq_ep_s **r = a->links.array;
      r = realloc(r, sizeof(*r) * (c + 1));
      if (!r)
        return -ENOMEM;
      r[c] = b;
      a->links.array = r;
      a->link_count++;
      return 0;
    }

    }
#endif
}

static error_t device_irq_ep_unlink_half(struct dev_irq_ep_s *a, struct dev_irq_ep_s *b)
{
#if defined(CONFIG_DEVICE_IRQ_SHARING) || defined(CONFIG_DEVICE_IRQ_MULTI_SINK)
  struct dev_irq_ep_s **r = a->links.array;
#endif

  switch (a->link_count)
    {
    case 0:
      break;

    case 1:
      if (a->links.single != b)
        return -ENOENT;
      a->link_count = 0;
      return 0;

#if defined(CONFIG_DEVICE_IRQ_SHARING) || defined(CONFIG_DEVICE_IRQ_MULTI_SINK)
    case 2: {
      if (r[0] == b)
        a->links.single = r[1];
      else if (r[1] == b)
        a->links.single = r[0];
      else
        return -ENOENT;
      mem_free(r);
      a->link_count = 1;
      return 0;
    }

    default: {
      uint_fast8_t i;
      for (i = 0; i < a->link_count; i++)
        if (r[i] == b)
          {
            memmove(r + i, r + i + 1, (a->link_count - i - 1) * sizeof(struct dev_irq_ep_s *));
            a->link_count--;
            return 0;
          }
    }
#endif
    }

  return -ENOENT;
}

static void device_irq_ep_unlink_all(struct dev_irq_ep_s *a)
{
  switch (a->link_count)
    {
    case 0:
      return;

    case 1:
      device_irq_ep_unlink_half(a->links.single, a);
      a->link_count = 0;
      return;

#if defined(CONFIG_DEVICE_IRQ_SHARING) || defined(CONFIG_DEVICE_IRQ_MULTI_SINK)
    default: {
      struct dev_irq_ep_s **r = a->links.array;
      uint_fast8_t i;
      for (i = 0; i < a->link_count; i++)
        ensure(device_irq_ep_unlink_half(r[i], a));
      mem_free(r);
      a->link_count = 0;
      return;
    }
#endif
    }
}

static
error_t device_irq_ep_link(struct dev_irq_src_s *source, struct dev_irq_sink_s *sink)
{
#ifndef CONFIG_DEVICE_IRQ_SHARING
  if (sink->base.link_count)
    return -ENOTSUP;
#endif
#ifndef CONFIG_DEVICE_IRQ_MULTI_SINK
  if (source->base.link_count)
    return -ENOTSUP;
#endif

  if (device_irq_ep_link_half(&source->base, &sink->base))
    return -ENOMEM;
  if (device_irq_ep_link_half(&sink->base, &source->base))
    {
      ensure(!device_irq_ep_unlink_half(&source->base, &sink->base));
      return -ENOMEM;
    }

#warning sink device lock
  sink->base.dev->ref_count++;

  return 0;
}

static
error_t device_irq_ep_unlink(struct dev_irq_src_s *source, struct dev_irq_sink_s *sink)
{
  assert(source != NULL || sink != NULL);

  if (device_irq_ep_unlink_half(&source->base, &sink->base))
    return -ENOENT;
  ensure(!device_irq_ep_unlink_half(&sink->base, &source->base));
  device_irq_sink_cleanup(sink);

  sink->base.dev->ref_count--;

  return 0;
}

void device_irq_source_init(struct device_s *dev, struct dev_irq_src_s *sources,
                            uint_fast8_t src_count, dev_irq_src_process_t *handler)
{
  uint_fast8_t i;

  for (i = 0; i < src_count; i++)
    {
      struct dev_irq_src_s *ep = sources + i;
      ep->base.dev = dev;
      ep->base.link_count = 0;
      ep->process = handler;
      ep->trig_mode = 0;
    }
}

void device_irq_sink_init(struct device_s *dev, struct dev_irq_sink_s *sinks,
                          uint_fast8_t sink_count, dev_irq_sink_update_t *update,
                          enum dev_irq_sense_modes_e sense_capabilities)
{
  uint_fast8_t i;

  for (i = 0; i < sink_count; i++)
    {
      struct dev_irq_sink_s *ep = sinks + i;
      ep->base.dev = dev;
      ep->base.link_count = 0;
      ep->base.links.single = (void*)&device_irq_dummy_src_ep;
      ep->update = update;
      ep->sense_link = ep->sense_all = sense_capabilities;
      ep->icu_pv = 0;
    }
}

static error_t device_icu_source_link(struct dev_irq_src_s *src, dev_irq_route_t *route_mask)
{
  struct dev_irq_ep_s **array;
  uint_fast8_t count = src->base.link_count;
  uint_fast8_t j = 0;
  error_t err = 0;

  switch (count)
    {
    case 0:
      return 0;
    case 1:
      array = &src->base.links.single;
#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
      break;
    default:
      array = src->base.links.array;
      break;
    }

  dev_irq_route_t rt = route_mask != NULL ? *route_mask : 0;

  for (; j < count && !err; j++)
    {
#endif
      struct dev_irq_sink_s *sink = dev_irq_sink_s_cast(array[j]);
      struct dev_irq_src_s *bypass = NULL;

#ifdef CONFIG_DEVICE_IRQ_BYPASS
      while (1)
        {
#endif
          struct device_icu_s icu;
          if (device_get_accessor(&icu, sink->base.dev, DRIVER_CLASS_ICU, 0))
            return -EINVAL;

          err = DEVICE_OP(&icu, link, sink, src, route_mask, &bypass);

          device_put_accessor(&icu);

#ifdef CONFIG_DEVICE_IRQ_BYPASS
          if (err != -EAGAIN)
            break;

          if (bypass->base.link_count != 1)
            return -EINVAL;

          sink = dev_irq_sink_s_cast(bypass->base.links.single);
        }
#else
      if (err != -EAGAIN)
        err = 0;
#endif

#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
      if (route_mask != NULL)
        *route_mask = rt;
#endif
    }

  return err;
}

void device_irq_source_unlink(struct device_s *dev, struct dev_irq_src_s *sources, uint_fast8_t src_count)
{
  uint_fast8_t i = 0;

  for (; i < src_count; i++)
    {
      struct dev_irq_src_s *src = sources + i;
      device_icu_source_link(src, NULL);
      device_irq_ep_unlink_all(dev_irq_src_s_base(src));
    }
}

error_t device_irq_source_link(struct device_s *dev, struct dev_irq_src_s *srcs,
                               uint_fast8_t src_count, uint32_t enable_mask)
{
  uint_fast8_t i;
  error_t err;
  bool_t done[src_count];
  const char *icu_path = NULL;

  memset(done, 0, sizeof(done));

  DEVICE_RES_FOREACH(dev, r, {

      if (r->type == DEV_RES_DEV_PARAM && !strcmp(r->u.dev_param.name, "icu"))
        icu_path = r->u.dev_param.dev;

      if (r->type != DEV_RES_IRQ)
        continue;

      if (!icu_path)
        {
          printk("device: missing icu device resource for device %p.\n", dev);
          return -ENOENT;
        }

      uint_fast8_t src_id = r->u.irq.src_id;

      if (src_id >= src_count)
        {
          printk("device: driver for device %p does not provide source end-point for IRQ output %u.\n",
                 dev, src_id);
          err = -ENOENT;
          goto error;
        }

      if (done[src_id])
        {
          printk("device: multiple resource entries for IRQ source end-point %u.\n", dev, src_id);
          err = -ENOENT;
          goto error;
        }

      struct device_icu_s icu;
      if (device_get_accessor_by_path(&icu, &dev->node, icu_path, DRIVER_CLASS_ICU))
        {
          printk("device: can not use %p device as an interrupt controller.\n", icu_path);
          err = -EINVAL;
          goto error;
        }

      struct device_s *icu_dev = icu.dev;
      struct dev_irq_src_s *src = srcs + src_id;
      struct dev_irq_sink_s *sink = DEVICE_OP(&icu, get_sink, r->u.irq.sink_id);

      device_put_accessor(&icu);

      if (!sink)
        {
          printk("device: interrupt controller %p does not have sink end-point for IRQ input %u.\n", icu_dev, r->u.irq.sink_id);
          err = -EINVAL;
          goto error;
        }

      /* create link between end-points */
      if ((err = device_irq_ep_link(src, sink)))
        goto error;

      uint_fast16_t irq_id = r->u.irq.irq_id;
      uint_fast16_t trig_mode = r->u.irq.trig_mode;
      dev_irq_route_t route_mask = r->u.irq.route_mask;

      assert(!irq_id || trig_mode == DEV_IRQ_SENSE_ID_BUS);
      assert(sink->sense_all);

#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
      if (src->base.link_count > 1)
        {
          if (irq_id != src->irq_id)
            {
              printk("device: irq linked to multiple sinks with different logical ids.\n", icu_dev);
              err = -EINVAL;
              goto error;
            }
        }
#endif

      src->irq_id = irq_id;

#ifdef CONFIG_DEVICE_IRQ_SHARING
      if (sink->base.link_count > 1)
        {
          trig_mode &= sink->sense_link;
          if (!trig_mode)
            {
              printk("device: icu %p can't share a sink end-point with a different trigger mode.\n", icu_dev);
              err = -EINVAL;
              goto error;
            }
          src->trig_mode = dev_irq_src_s_cast(sink->base.links.array[0])->trig_mode;
        }
      else
#endif
        {
          /* update sink sense modes */
          if (trig_mode)
            {
              trig_mode &= sink->sense_all;
              if (!trig_mode)
                {
                  printk("device: trigger modes of IRQ source %u of device %p not supported by sink\n", src_id, dev);
                  err = -EINVAL;
                  goto error;
                }
              trig_mode &= ~(trig_mode - 1); /* keep a single trigger mode */
            }

          sink->sense_link = trig_mode;
        }

      /* setup interrupt controller */
      if ((err = device_icu_source_link(src, &route_mask)))
        {
          printk("device: unable to link IRQ source %u of device %p\n", src_id, dev);
          err = -EINVAL;
          goto error;
        }

#ifdef CONFIG_DEVICE_IRQ_SHARING
      if (sink->base.link_count == 1)
#endif
        {
          if ((enable_mask >> src_id) & 1)
            {
              sink->update(sink, trig_mode, irq_id);
              src->trig_mode = trig_mode;
            }
          else
            {
              src->trig_mode = DEV_IRQ_SENSE_NONE;
            }
        }

      done[src_id]++;
  });

  for (i = 0; i < src_count; i++)
    if (!done[i])
      {
        printk("device: Unable to link IRQ source end-point %u of device %p, no IRQ resource entry.\n", i, dev);
        err = -ENOENT;
        goto error;
      }

  return 0;

 error:
  device_irq_source_unlink(dev, srcs, src_count);
  return err;
}

error_t device_irq_src_update(struct dev_irq_src_s *src,
                              enum dev_irq_sense_modes_e trig_mode)
{
  if (src->trig_mode == trig_mode)
    return 0;

  struct dev_irq_ep_s **array = &src->base.links.single;
  uint_fast8_t count = src->base.link_count;
  uint_fast8_t j = 0;

  switch (count)
    {
    case 0:
      return -ENOENT;
    default:
#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
      array = src->base.links.array;
    case 1:
#endif
      break;
    }

#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
  for (j = 0; j < count; j++)
#endif
    {
      struct dev_irq_sink_s *sink = dev_irq_sink_s_cast(array[j]);
#ifdef CONFIG_DEVICE_IRQ_SHARING
      if (sink->base.link_count > 1)
        return -EBUSY;
#endif
      if (trig_mode)
        {
          if (!(sink->sense_link & trig_mode))
            return -ENOTSUP;
          assert(ALIGN_ISPOWTWO(trig_mode));
        }
    }

#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
  for (j = 0; j < count; j++)
#endif
    {
      struct dev_irq_sink_s *sink = dev_irq_sink_s_cast(array[j]);
      sink->update(sink, trig_mode, src->irq_id);
    }

  src->trig_mode = trig_mode;

  return 0;
}

inline error_t device_irq_src_enable(struct dev_irq_src_s *src)
{
  struct dev_irq_ep_s **array = &src->base.links.single;
  uint_fast8_t count = src->base.link_count;
  uint_fast8_t j = 0;

  switch (count)
    {
    case 0:
      return -ENOENT;
    default:
#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
      array = src->base.links.array;
    case 1:
#endif
      break;
    }

  struct dev_irq_sink_s *sink;
#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
  for (j = 0; j < count; j++)
#endif
    {
      sink = dev_irq_sink_s_cast(array[j]);
#ifdef CONFIG_DEVICE_IRQ_SHARING
      if (sink->base.link_count > 1)
        return -EBUSY;
#endif
    }

  enum dev_irq_sense_modes_e trig_mode = sink->sense_link;
  assert(trig_mode && ALIGN_ISPOWTWO(trig_mode));

  if (src->trig_mode == trig_mode)
    return 0;

#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
  for (j = 0; j < count; j++)
    {
      sink = dev_irq_sink_s_cast(array[j]);
#endif
      sink->update(sink, trig_mode, src->irq_id);
#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
    }
#endif

  src->trig_mode = trig_mode;

  return 0;
}

void device_irq_modes(struct dev_irq_src_s *src,
                      enum dev_irq_sense_modes_e *cur_,
                      enum dev_irq_sense_modes_e *modes_)
{
  struct dev_irq_ep_s **array = &src->base.links.single;
  struct dev_irq_sink_s *sink;
  uint_fast8_t count = src->base.link_count;
  enum dev_irq_sense_modes_e cur, modes;

  switch (count)
    {
    case 0:
      modes = cur = 0;
      break;
    default:
#ifdef CONFIG_DEVICE_IRQ_MULTI_SINK
      array = src->base.links.array;
    case 1:
#endif
      sink = dev_irq_sink_s_cast(array[0]);
      modes = sink->sense_link;
      cur = src->trig_mode;
      break;
    }
  if (modes_)
    *modes_ = modes;
  if (cur_)
    *cur_ = cur;
}

error_t device_icu_irq_bind(struct dev_irq_src_s *src, const char *icu_name,
                            uint_fast16_t sink_id, uint8_t irq_id,
                            enum dev_irq_sense_modes_e trig_mode, dev_irq_route_t route_mask)
{
    error_t err;
    struct device_icu_s icu;
    struct dev_irq_sink_s *sink;

    err = device_get_accessor_by_path(&icu, NULL, icu_name, DRIVER_CLASS_ICU);
    if (err) {
        printk("Error while getting icu \"%s\": %d\n", icu_name, err);
        return err;
    }

    sink = DEVICE_OP(&icu, get_sink, sink_id);

    if (!sink)
      {
        err = -ENOENT;
        goto out;
      }

    trig_mode &= sink->sense_link;

    if (!trig_mode)
      {
        err = -ENOTSUP;
        goto out;
      }

    trig_mode &= ~(trig_mode - 1); /* keep a single trigger mode */
    src->trig_mode = trig_mode;

    err = device_irq_ep_link(src, sink);
    if (err)
        goto out;

    if (!device_icu_source_link(src, &route_mask))
      {
        device_irq_ep_unlink(src, sink);
        err = -EBUSY;
        goto out;
      }

    sink->update(sink, trig_mode, irq_id);

    err = 0;

  out:
    device_put_accessor(&icu);

    return err;
}

DEV_ICU_LINK(device_icu_dummy_link)
{
  return 0;
}


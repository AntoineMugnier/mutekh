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
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/icu.h>
#include <device/class/enum.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <stdlib.h>
#include <string.h>

#include <assert.h>

/* anchor source_process */
static DEV_IRQ_EP_PROCESS(device_irq_source_process)
{
  struct dev_irq_ep_s  *src = ep;

#ifdef CONFIG_DEBUG
  assert(src->type != DEV_IRQ_EP_SINK);
#endif

  /* process irq and get next sink */
  int_fast16_t         next_id = *id;
  struct dev_irq_ep_s  *sink = src->f_irq(src, &next_id);

#ifdef CONFIG_DEBUG
  assert(!sink || sink->type != DEV_IRQ_EP_SOURCE);
#endif

  if (sink)
    sink->process(sink, &next_id);
}
/* anchor end */

void device_irq_spurious_process(struct dev_irq_ep_s *sink, int_fast16_t *id)
{
  /* FIXME disable lost irq here */
}

#if 0
/* anchor sink_process */
static DEV_IRQ_EP_PROCESS(device_irq_sink_process)
{
  struct dev_irq_ep_s  *sink = ep;

#ifdef CONFIG_DEBUG
  assert(sink->type != DEV_IRQ_EP_SOURCE);
#endif

  switch (sink->links_count)
    {
      struct dev_irq_ep_s *src;

    case 0: /* no source end-point connected to sink */
      device_irq_spurious_process(sink, id);
      break;

    case 1: /* single source end-point connected */
      src = sink->links.single;
      src->process(src, id);
      break;

    default: { /* multiple source end-points sharing the same irq line */
      uint_fast8_t i;
      for (i = 0; i < sink->links_count; i++)
        {
          src = sink->links.array[i];
          src->process(src, id);
        }
      break;
    }
    }

  /* for icu devs only */
  if (sink->f_irq_ack)
    sink->f_irq_ack(sink, *id);
}
/* anchor end */
#endif

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_0a)
{
  struct dev_irq_ep_s  *sink = ep;
  device_irq_spurious_process(sink, id);
  sink->f_irq_ack(sink, *id);
}

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_0)
{
  struct dev_irq_ep_s  *sink = ep;
  device_irq_spurious_process(sink, id);
}

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_1a)
{
  struct dev_irq_ep_s  *sink = ep;
  struct dev_irq_ep_s *src = sink->links.single;
  src->process(src, id);
  sink->f_irq_ack(sink, *id);
}

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_1)
{
  struct dev_irq_ep_s  *sink = ep;
  struct dev_irq_ep_s *src = sink->links.single;
  src->process(src, id);
}

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_Na)
{
  struct dev_irq_ep_s  *sink = ep;
  uint_fast8_t i;
  for (i = 0; i < sink->links_count; i++)
    {
      struct dev_irq_ep_s *src = sink->links.array[i];
      src->process(src, id);
    }
  sink->f_irq_ack(sink, *id);
}

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_N)
{
  struct dev_irq_ep_s  *sink = ep;
  uint_fast8_t i;
  for (i = 0; i < sink->links_count; i++)
    {
      struct dev_irq_ep_s *src = sink->links.array[i];
      src->process(src, id);
    }
}

static void device_irq_sink_fcn_set(struct dev_irq_ep_s *sink)
{
  switch (sink->links_count)
    {
      case 0:
        sink->process = sink->f_irq_ack
          ? &device_irq_sink_process_0a
          : &device_irq_sink_process_0;
        return;
      case 1:
        sink->process = sink->f_irq_ack
          ? &device_irq_sink_process_1a
          : &device_irq_sink_process_1;
        return;
      default:
        sink->process = sink->f_irq_ack
          ? &device_irq_sink_process_Na
          : &device_irq_sink_process_N;
        return;
    }
}

/****************************************/

static error_t device_irq_ep_link_half(struct dev_irq_ep_s *a, struct dev_irq_ep_s *b)
{
  uint_fast8_t c = a->links_count;

  switch (c)
    {
    case 0:
      a->links.single = b;
      a->links_count = 1;
      return 0;

    case 1: {
      struct dev_irq_ep_s **r, *t = a->links.single;
      r = mem_alloc(sizeof(*r) * 2, (mem_scope_sys));
      if (!r)
        return -ENOMEM;
      r[0] = t;
      r[1] = b;
      a->links.array = r;
      a->links_count = 2;
      return 0;
    }

    default: {
      struct dev_irq_ep_s **r = a->links.array;
      r = realloc(r, sizeof(*r) * (c + 1));
      if (!r)
        return -ENOMEM;
      r[c] = b;
      a->links.array = r;
      a->links_count++;
      return 0;
    }

    }
}

static error_t device_irq_ep_unlink_half(struct dev_irq_ep_s *a, struct dev_irq_ep_s *b)
{
  struct dev_irq_ep_s **r = a->links.array;

  switch (a->links_count)
    {
    case 0:
      return -ENOENT;

    case 1:
      if (a->links.single != b)
        return -ENOENT;
      a->links_count = 0;
      return 0;

    case 2: {
      if (r[0] == b)
        a->links.single = r[1];
      else if (r[1] == b)
        a->links.single = r[0];
      else
        return -ENOENT;
      mem_free(r);
      a->links_count = 1;
      return 0;
    }

    default: {
      uint_fast8_t i;
      for (i = 0; i < a->links_count; i++)
        if (r[i] == b)
          {
            memmove(r + i, r + i + 1, (a->links_count - i - 1) * sizeof(struct dev_irq_ep_s *));
            a->links_count--;
            return 0;
          }
      return -ENOENT;
    }
    }

}

static void device_irq_ep_unlink_all(struct dev_irq_ep_s *a)
{
  switch (a->links_count)
    {
    case 0:
      return;

    case 1:
      device_irq_ep_unlink_half(a->links.single, a);
      a->links_count = 0;
      return;

    default: {
      struct dev_irq_ep_s **r = a->links.array;
      uint_fast8_t i;
      for (i = 0; i < a->links_count; i++)
        ensure(device_irq_ep_unlink_half(r[i], a));
      mem_free(r);
      a->links_count = 0;
      return;
    }
    }
}

error_t device_irq_ep_link(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink)
{
#ifdef CONFIG_DEBUG
  assert(source->type != DEV_IRQ_EP_SINK);
  assert(sink->type != DEV_IRQ_EP_SOURCE);
#endif

  if (device_irq_ep_link_half(source, sink))
    return -ENOMEM;
  if (device_irq_ep_link_half(sink, source))
    {
      ensure(!device_irq_ep_unlink_half(source, sink));
      return -ENOMEM;
    }

  device_irq_sink_fcn_set(sink);
  return 0;
}

error_t device_irq_ep_unlink(struct dev_irq_ep_s *source, struct dev_irq_ep_s *sink)
{
  assert(source != NULL || sink != NULL);
#ifdef CONFIG_DEBUG
  assert(!source || source->type != DEV_IRQ_EP_SINK);
  assert(!sink || sink->type != DEV_IRQ_EP_SOURCE);
#endif

  if (source == NULL)
    {
      device_irq_ep_unlink_all(sink);
      device_irq_sink_fcn_set(sink);
      return 0;
    }
  else if (sink == NULL)
    {
      device_irq_ep_unlink_all(source);
      return 0;
    }
  else
    {
      if (device_irq_ep_unlink_half(source, sink))
        return -ENOENT;
      ensure(!device_irq_ep_unlink_half(sink, source));
      device_irq_sink_fcn_set(sink);
      return 0;
    }
}

void device_irq_source_init(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count, dev_irq_t *handler)
{
  uint_fast8_t i;

  for (i = 0; i < src_count; i++)
    {
      struct dev_irq_ep_s *ep = sources + i;
      ep->dev = dev;
      ep->process = &device_irq_source_process;
      ep->links_count = 0;
      ep->f_irq = handler;
#ifdef CONFIG_DEBUG
      ep->type = DEV_IRQ_EP_SOURCE;
#endif
    }
}

void device_irq_tail_source_init(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count, dev_irq_t *handler)
{
  uint_fast8_t i;

  for (i = 0; i < src_count; i++)
    {
      struct dev_irq_ep_s *ep = sources + i;
      ep->dev = dev;
      ep->process = (dev_irq_ep_process_t*)handler;
      ep->links_count = 0;
      ep->f_irq = NULL;
#ifdef CONFIG_DEBUG
      ep->type = DEV_IRQ_EP_SOURCE;
#endif
    }
}

void device_irq_icu_source_init(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count, dev_irq_ep_process_t *process)
{
  uint_fast8_t i;

  for (i = 0; i < src_count; i++)
    {
      struct dev_irq_ep_s *ep = sources + i;
      ep->dev = dev;
      ep->process = process;
      ep->links_count = 0;
      ep->f_irq = NULL;
#ifdef CONFIG_DEBUG
      ep->type = DEV_IRQ_EP_SOURCE;
#endif
    }
}

void device_irq_sink_init(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t sink_count, dev_irq_ack_t *ack_handler)
{
  uint_fast8_t i;

  for (i = 0; i < sink_count; i++)
    {
      struct dev_irq_ep_s *ep = sinks + i;
      ep->dev = dev;
      ep->links_count = 0;
      ep->f_irq_ack = ack_handler;
#ifdef CONFIG_DEBUG
      ep->type = DEV_IRQ_EP_SINK;
#endif
      device_irq_sink_fcn_set(ep);
    }
}

void device_irq_source_unlink(struct device_s *dev, struct dev_irq_ep_s *sources, uint_fast8_t src_count)
{
  uint_fast8_t i;

  for (i = 0; i < src_count; i++)
    {
#ifdef CONFIG_DEBUG
      assert(src[i].type != DEV_IRQ_EP_SINK);
#endif
      device_irq_ep_unlink_all(sources + i);
    }
}

void device_irq_sink_unlink(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t sink_count)
{
  uint_fast8_t i;

  for (i = 0; i < sink_count; i++)
    {
#ifdef CONFIG_DEBUG
      assert(src[i].type != DEV_IRQ_EP_SOURCE);
#endif
      device_irq_ep_unlink_all(sinks + i);
      device_irq_sink_fcn_set(sinks + i);
    }
}

error_t device_irq_source_link(struct device_s *dev, struct dev_irq_ep_s *src, uint_fast8_t src_count)
{
  uint_fast8_t i;
  uint_fast8_t j = -1;
  error_t err;

  for (i = 0; i < dev->res_count; i++)
    {
      struct dev_resource_s *r = dev->res + i;

      if (r->type != DEV_RES_IRQ)
        continue;

      if (r->irq.dev_out_id >= src_count)
        {
          printk("device: driver for device %p `%s' does not provide source end-point for IRQ output %u.\n",
                 dev, dev->name, r->irq.dev_out_id);
          err = -ENOENT;
          goto error;
        }

      struct device_s *icu_dev = r->irq.icu;
      if (!icu_dev)
        icu_dev = device_get_default_icu(dev);

      if (!icu_dev)
        {
          printk("device: no interrupt controller available for %p `%s' device.\n", dev, dev->name);
          err = -ENOENT;
          goto error;
        }

      struct device_icu_s icu;

      if (device_get_accessor(&icu, icu_dev, DRIVER_CLASS_ICU, 0))
        {
          printk("device: can not use %p `%s' device as an interrupt controller.\n", r->irq.icu, r->irq.icu->name);
          err = -EINVAL;
          goto error;
        }

      struct dev_irq_ep_s *sink = DEVICE_OP(&icu, get_sink, r->irq.icu_in_id);

      if (!sink)
        {
          printk("device: interrupt controller %p `%s' does not have sink end-point for IRQ input %u.\n");
          err = -EINVAL;
        }
      else
        {
          err = device_irq_ep_link(src + r->irq.dev_out_id, sink);
        }

      if (err)
        goto error;

      device_put_accessor(&icu);

      j++;
    }

  return 0;

 error:
  device_irq_source_unlink(dev, src, src_count);
  return err;
}

struct device_s * device_get_default_icu(struct device_s *dev)
{
#ifdef CONFIG_DEVICE_TREE
  struct device_enum_s e;

  if (!dev->enum_dev)
    return NULL;
  if (device_get_accessor(&e, dev->enum_dev, DRIVER_CLASS_ENUM, 0))
    return NULL;

  struct device_s *icu = NULL;

  if (DEVICE_HAS_OP(&e, get_default_icu))
    icu = DEVICE_OP(&e, get_default_icu, dev);

  device_put_accessor(&e);

  return icu;
#else
  return NULL;
#endif
}


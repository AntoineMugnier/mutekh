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

#include <mutek/printk.h>

#include <assert.h>
#include <mutek/mem_alloc.h>

static bool_t device_irq_source_process(struct dev_irq_ep_s *src, int_fast16_t id)
{
#ifdef CONFIG_DEBUG
  assert(src->type != DEV_IRQ_EP_SINK);
#endif

  /* process irq and get next sink */
  int_fast16_t         next_id = id;
  struct dev_irq_ep_s  *sink = src->f_irq(src, &next_id);

#ifdef CONFIG_DEBUG
  assert(sink->type != DEV_IRQ_EP_SOURCE);
#endif

  if (sink == NULL)
    return next_id == 0;                /* not an icu device */

  bool_t done = sink->process(sink, next_id);

  /* for icu devs only */
  sink->f_irq_ack(sink, id, next_id);

  return done;
}

static bool_t device_irq_sink_process(struct dev_irq_ep_s *sink, int_fast16_t id)
{
#ifdef CONFIG_DEBUG
  assert(sink->type != DEV_IRQ_EP_SOURCE);
#endif

  switch (sink->links_count)
    {
      struct dev_irq_ep_s *src;

    case 0: /* no source end-point connected to sink */
      /* FIXME disable lost irq here */
      return 0;

    case 1: /* single source end-point connected */
      src = sink->links.single;
      return src->process(src, id);

    default: { /* multiple source end-points sharing the same irq line */
      bool_t done = 0;
      uint_fast8_t i;
      for (i = 0; i < sink->links_count; i++)
        {
          src = sink->links.array[i];
          done |= src->process(src, id);
        }
      return done;
    }
    }
}

/****************************************/

void device_irq_ep_source_init(struct dev_irq_ep_s *ep, struct device_s *dev, dev_irq_t *handler)
{
  ep->dev = dev;
  ep->process = &device_irq_source_process;
  ep->links_count = 0;
  ep->f_irq = handler;
#ifdef CONFIG_DEBUG
  ep->type = DEV_IRQ_EP_SOURCE;
#endif
}

static DEV_IRQ_ACK(device_irq_no_ack)
{
}

void device_irq_ep_sink_init(struct dev_irq_ep_s *ep, struct device_s *dev, dev_irq_ack_t *ack_handler)
{
  ep->dev = dev;
  ep->process = &device_irq_sink_process;
  ep->links_count = 0;
  ep->f_irq_ack = ack_handler ? ack_handler : &device_irq_no_ack;
#ifdef CONFIG_DEBUG
  ep->type = DEV_IRQ_EP_SINK;
#endif
}

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
      struct dev_irq_ep_s **r;
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
      return 0;
    }
}

void device_irq_unlink(struct device_s *dev, struct dev_irq_ep_s *src, uint_fast8_t src_count)
{
  uint_fast8_t i;

  for (i = 0; i < src_count; i++)
    {
#ifdef CONFIG_DEBUG
      assert(src[i].type != DEV_IRQ_EP_SINK);
#endif
      device_irq_ep_unlink_all(src + i);
    }
}

error_t device_irq_link(struct device_s *dev, dev_irq_t *handler,
                        struct dev_irq_ep_s *src, uint_fast8_t src_count)
{
  uint_fast8_t i;
  uint_fast8_t j = -1;
  error_t err;

  for (i = 0; i < src_count; i++)
    device_irq_ep_source_init(src + i, dev, handler);

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

      struct device_icu_s icu;

      if (device_get_accessor(&icu, r->irq.icu, DEVICE_CLASS_ICU, 0))
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
  device_irq_unlink(dev, src, src_count);
  return err;
}

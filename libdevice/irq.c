
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
#include <stdlib.h>
#include <string.h>

#include <assert.h>

#if 0
/* anchor source_process */
static DEV_IRQ_EP_PROCESS(device_icu_source_process)
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

/* anchor sink_process */
static DEV_IRQ_EP_PROCESS(device_irq_sink_process)
{
  struct dev_irq_ep_s  *sink = ep;

#ifdef CONFIG_DEBUG
  assert(sink->type != DEV_IRQ_EP_SOURCE);
#endif

  switch (sink->link_count)
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
      for (i = 0; i < sink->link_count; i++)
        {
          src = sink->links.array[i];
          src->process(src, id);
        }
      break;
    }
    }
}
/* anchor end */
#endif

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_0)
{
  struct dev_irq_ep_s  *sink = ep;

  printk("device: spurious interrupt on sink end-point %p with no connection (icu device %p)\n", sink, sink->dev);
}

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_1)
{
  struct dev_irq_ep_s  *sink = ep;
  struct dev_irq_ep_s *src = sink->links.single; 
  //  printk("irq1 %u %p\n", id, src->dev);
  src->process(src, id);
}

static DEV_IRQ_EP_PROCESS(device_irq_sink_process_N)
{
  struct dev_irq_ep_s  *sink = ep;
  uint_fast8_t i;
  for (i = 0; i < sink->link_count; i++)
    {
      struct dev_irq_ep_s *src = sink->links.array[i];
      //  printk("irqN %u %p\n", id, src->dev);
      src->process(src, id);
    }
}

static void device_irq_sink_fcn_set(struct dev_irq_ep_s *sink)
{
  switch (sink->link_count)
    {
    case 0: {
      struct device_icu_s icu;

      if (sink->dev &&
          !device_get_accessor(&icu, sink->dev, DRIVER_CLASS_ICU, 0))
        {
          if (DEVICE_HAS_OP(&icu, disable_irq))
            DEVICE_OP(&icu, disable_irq, sink);
          device_put_accessor(&icu);
        }
      sink->process = &device_irq_sink_process_0;
      return;
    }
    case 1:
      sink->process = &device_irq_sink_process_1;
      return;
    default:
      sink->process = &device_irq_sink_process_N;
      return;
    }
}

/****************************************/

#ifdef CONFIG_DEVICE_IRQ_BYPASS

void device_irq_bypass_init(struct dev_irq_bypass_s *bypass, uint_fast8_t count)
{
  memset(bypass, 0, sizeof(*bypass) * count);
}

void device_irq_bypass_cleanup(struct dev_irq_bypass_s *bypass, uint_fast8_t count)
{
  uint_fast8_t i;
  for (i = 0; i < count; i++)
    device_irq_bypass_unlink(bypass + i);
}

void device_irq_bypass_unlink(struct dev_irq_bypass_s *bypass)
{
  struct dev_irq_ep_s *src = bypass->src;
  struct dev_irq_bypass_s **head, *i;

  if (src == NULL)
    return;

  for (head = &src->bypass_list; (i = *head); head = &i->next)
    if (i == bypass)
      {
        *head = i->next;
        i->src = NULL;
        i->next = NULL;
        return;
      }

  abort();
}

error_t device_irq_bypass_link(struct dev_irq_ep_s *src, struct dev_irq_bypass_s *bypass)
{
  if (!src)
    {
      device_irq_bypass_unlink(bypass);
      bypass->next = bypass; // unusable mark
      return -EBUSY;
    }
  else if (bypass->src)
    {
      if (bypass->src == src)
        return -EEXISTS;
      device_irq_bypass_unlink(bypass);
      bypass->next = bypass; // unusable mark
      return -EBUSY;
    }
  else if (bypass->next == bypass)
    {
      return -EBUSY;
    }
  else
    {
      bypass->next = src->bypass_list;
      src->bypass_list = bypass;
      bypass->src = src;
      return 0;
    }
}

void device_irq_bypass_src_cleanup(struct dev_irq_ep_s *src)
{
  struct dev_irq_bypass_s *bypass = src->bypass_list;

  while (bypass)
    {
      assert(bypass->src == src);
      bypass->src = NULL;

      struct dev_irq_bypass_s *next = bypass->next;
      bypass->next = NULL;
      bypass = next;
    }

  src->bypass_list = NULL;
}

#endif

/****************************************/

static error_t device_irq_ep_link_half(struct dev_irq_ep_s *a, struct dev_irq_ep_s *b)
{
  uint_fast8_t c = a->link_count;

  switch (c)
    {
    case 0:
      a->links.single = b;
      a->link_count = 1;
      return 0;

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
}

static error_t device_irq_ep_unlink_half(struct dev_irq_ep_s *a, struct dev_irq_ep_s *b)
{
  struct dev_irq_ep_s **r = a->links.array;

  switch (a->link_count)
    {
    case 0:
      return -ENOENT;

    case 1:
      if (a->links.single != b)
        return -ENOENT;
      a->link_count = 0;
      return 0;

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
      return -ENOENT;
    }
    }

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

    default: {
      struct dev_irq_ep_s **r = a->links.array;
      uint_fast8_t i;
      for (i = 0; i < a->link_count; i++)
        ensure(device_irq_ep_unlink_half(r[i], a));
      mem_free(r);
      a->link_count = 0;
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

  if (device_irq_ep_unlink_half(source, sink))
    return -ENOENT;
  ensure(!device_irq_ep_unlink_half(sink, source));
  device_irq_sink_fcn_set(sink);
  return 0;
}

void device_irq_source_init(struct device_s *dev, struct dev_irq_ep_s *sources,
                            uint_fast8_t src_count, dev_irq_ep_process_t *handler,
                            enum dev_irq_sense_modes_e sense_capabilities)
{
  uint_fast8_t i;

  for (i = 0; i < src_count; i++)
    {
      struct dev_irq_ep_s *ep = sources + i;
      ep->dev = dev;
      ep->process = handler;
      ep->link_count = 0;
      ep->sense = sense_capabilities;
#ifdef CONFIG_DEVICE_IRQ_BYPASS
      ep->bypass_list = NULL;
#endif
#ifdef CONFIG_DEBUG
      ep->type = DEV_IRQ_EP_SOURCE;
#endif
    }
}

void device_irq_sink_init(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t sink_count,
                          enum dev_irq_sense_modes_e sense_capabilities)
{
  uint_fast8_t i;

  for (i = 0; i < sink_count; i++)
    {
      struct dev_irq_ep_s *ep = sinks + i;
      ep->dev = dev;
      ep->link_count = 0;
      ep->sense = sense_capabilities;
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
      assert(sources[i].type != DEV_IRQ_EP_SINK);
#endif
      device_irq_ep_unlink_all(sources + i);
#ifdef CONFIG_DEVICE_IRQ_BYPASS
      device_irq_bypass_src_cleanup(sources + i);
#endif
    }
}

void device_irq_sink_unlink(struct device_s *dev, struct dev_irq_ep_s *sinks, uint_fast8_t sink_count)
{
  uint_fast8_t i;

  for (i = 0; i < sink_count; i++)
    {
#ifdef CONFIG_DEBUG
      assert(sinks[i].type != DEV_IRQ_EP_SOURCE);
#endif
      device_irq_ep_unlink_all(sinks + i);
      device_irq_sink_fcn_set(sinks + i);
    }
}

error_t device_irq_source_link(struct device_s *dev, struct dev_irq_ep_s *srcs,
                               uint_fast8_t src_count, uint32_t enable_mask)
{
  uint_fast8_t i;
  error_t err;
  bool_t done[src_count];

  memset(done, 0, sizeof(done));

  DEVICE_RES_FOREACH(dev, r, {

      if (r->type != DEV_RES_IRQ)
        continue;

      uint_fast8_t id = r->u.irq.dev_out_id;

      if (id >= src_count)
        {
          printk("device: driver for device %p does not provide source end-point for IRQ output %u.\n",
                 dev, id);
          err = -ENOENT;
          goto error;
        }

      if (done[id])
        {
          printk("device: multiple resource entries for IRQ source end-point %u.\n", dev, id);
          err = -ENOENT;
          goto error;
        }

      struct device_s *icu_dev = dev;
      if (device_get_by_path(&icu_dev, r->u.irq.icu, &device_filter_init_done))
        {
          printk("device: no initialized interrupt controller available for %p device.\n", dev);
          err = -ENOENT;
          goto error;
        }

      struct device_icu_s icu;

      if (device_get_accessor(&icu, icu_dev, DRIVER_CLASS_ICU, 0))
        {
          printk("device: can not use %p device as an interrupt controller.\n", r->u.irq.icu);
          err = -EINVAL;
          goto error;
        }

      struct dev_irq_ep_s *src = srcs + id;
      struct dev_irq_ep_s *sink = DEVICE_OP(&icu, get_endpoint, DEV_IRQ_EP_SINK, r->u.irq.icu_in_id);

      if (!sink)
        {
          printk("device: interrupt controller %p does not have sink end-point for IRQ input %u.\n", icu_dev, r->u.irq.icu_in_id);
          err = -EINVAL;
        }
      else
        {
          err = device_irq_ep_link(src, sink);
        }

      device_put_accessor(&icu);

      if (err)
        goto error;

      if ((enable_mask >> id) & 1)
        if (!device_icu_irq_enable(src, r->u.irq.irq_id, src, src))
          {
            printk("device: Unable to enable IRQ output %u of device %p, no suitable irq path found.\n", r->u.irq.irq_id, dev);
            err = -EBUSY;
            goto error;
          }

      done[id]++;
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

bool_t device_icu_irq_enable(struct dev_irq_ep_s *local_src, uint_fast16_t target_irq_id,
                             struct dev_irq_ep_s *target_src, struct dev_irq_ep_s *dev_ep)
{
  struct dev_irq_ep_s **array;
  uint_fast8_t count = local_src->link_count;

  switch (count)
    {
    case 0:
      return 0;
    case 1:
      array = &local_src->links.single;
      break;
    default:
      array = local_src->links.array;
      break;
    }

  bool_t res = 0;
  uint_fast8_t  j;

  for (j = 0; j < count; j++)
    {
      struct dev_irq_ep_s *next_sink = array[j];
      struct device_icu_s next_icu;

#warning should we enable all source or stop on the first success? use icu priority to decide order?
      ensure(device_get_accessor(&next_icu, next_sink->dev, DRIVER_CLASS_ICU, 0) == 0);
      res |= DEVICE_OP(&next_icu, enable_irq, next_sink, target_irq_id, target_src, dev_ep);
      device_put_accessor(&next_icu);
    }

  return res;
}

error_t device_icu_irq_bind(struct dev_irq_ep_s *source, const char *icu_name,
                            uint_fast16_t sink_id, uint8_t irq_id)
{
    error_t err;
    struct device_icu_s icu;
    struct dev_irq_ep_s *sink;

    err = device_get_accessor_by_path(&icu, NULL, icu_name, DRIVER_CLASS_ICU);
    if (err) {
        printk("Error while getting icu \"%s\": %d\n", icu_name, err);
        return err;
    }

    sink = DEVICE_OP(&icu, get_endpoint, DEV_IRQ_EP_SINK, sink_id);

    if (!sink) {
        err = -ENOENT;
        goto out;
    }

    err = device_irq_ep_link(source, sink);
    if (err)
        goto out;

    if (!DEVICE_OP(&icu, enable_irq, sink, irq_id, source, source)) {
      device_irq_ep_unlink(source, sink);
      err = -EBUSY;
      goto out;
    }

    err = 0;

  out:
    device_put_accessor(&icu);

    return err;
}

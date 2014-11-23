/*
    this file is part of mutekh.

    mutekh is free software; you can redistribute it and/or modify it
    under the terms of the gnu lesser general public license as
    published by the free software foundation; version 2.1 of the
    license.

    mutekh is distributed in the hope that it will be useful, but
    without any warranty; without even the implied warranty of
    merchantability or fitness for a particular purpose.  see the gnu
    lesser general public license for more details.

    you should have received a copy of the gnu lesser general public
    license along with mutekh; if not, write to the free software
    foundation, inc., 51 franklin street, fifth floor, boston, ma
    02110-1301 usa.

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <device/device.h>
#include <device/driver.h>
#include <device/class/clock.h>

#include <mutek/printk.h>

error_t dev_clock_sink_hold(struct dev_clock_sink_ep_s *sink,
                           struct dev_clock_ready_s *ready)
{
  struct dev_clock_src_ep_s *src = sink->src;
  error_t err = 0;

  LOCK_SPIN_IRQ(&src->dev->lock);

  src->use_count++;
  if (!(src->flags & DEV_CLOCK_SRC_EP_RUNNING))
    err = src->f_use(src, ready, DEV_CLOCK_SRC_USE_HOLD);

  LOCK_RELEASE_IRQ(&src->dev->lock);

  return err;
}

void dev_clock_sink_release(struct dev_clock_sink_ep_s *sink)
{
  struct dev_clock_src_ep_s *src = sink->src;

  LOCK_SPIN_IRQ(&src->dev->lock);

  if (--src->use_count == 0)
    src->f_use(src, NULL, DEV_CLOCK_SRC_USE_RELEASE);

  LOCK_RELEASE_IRQ(&src->dev->lock);
}

error_t dev_clock_config(struct device_clock_s *accessor,
                         dev_clock_config_id_t config_id)
{
  struct device_s *dev = accessor->dev;
  bool_t done = 0;
  error_t err;

  DEVICE_RES_FOREACH(dev, r, {
      switch (r->type)
        {
        case DEV_RES_CLOCK_RTE: {
          if (!((r->u.clock_rte.config >> config_id) & 1))
            break;

          union dev_clock_config_value_u val, *v = NULL;

          if (r->u.clock_rte.denom != 0)
            {
              val.ratio.num = r->u.clock_rte.num;
              val.ratio.denom = r->u.clock_rte.denom;
              v = &val;
            }

          err = DEVICE_OP(accessor, config_node, r->u.clock_rte.node,
                          r->u.clock_rte.parent, v);
          if (err)
            goto err;

          done = 1;
          break;
        }

        case DEV_RES_CLOCK_OSC: {
          if (!((r->u.clock_osc.config >> config_id) & 1))
            break;

          union dev_clock_config_value_u val;
          val.freq.num = r->u.clock_osc.num;
          val.freq.denom = r->u.clock_osc.denom;
          val.acc = DEV_FREQ_ACC_INVALID;

          err = DEVICE_OP(accessor, config_node, r->u.clock_osc.node,
                          DEV_CLOCK_INVALID_NODE_ID, &val);
          if (err)
            goto err;

          done = 1;
          break;
        }

        default:
          continue;
        }
  });

  if (!done)
    return -ENOENT;

  return DEVICE_OP(accessor, commit);

 err:
  DEVICE_OP(accessor, rollback);
  return err;
}

void dev_clock_src_changed(struct device_clock_s *accessor,
                           struct dev_clock_src_ep_s *src,
                           const struct dev_freq_s *freq,
                           const struct dev_freq_accuracy_s *acc)
{
  struct dev_clock_sink_ep_s *s = src->sink_head;

  while (s != NULL)
    {
      if (s->f_changed != NULL)
        s->f_changed(s, freq, acc);
      s = s->next;
    }
}

error_t dev_clock_sink_link(struct device_s *dev,
                            struct dev_clock_sink_ep_s *sinks,
                            struct dev_clock_link_info_s *lkinfo,
                            dev_clock_node_id_t first_sink,
                            dev_clock_node_id_t last_sink)
{
  error_t err = 0;
  size_t count = last_sink - first_sink + 1;
  bool_t done[count];

  memset(done, 0, sizeof(done));

  DEVICE_RES_FOREACH(dev, r, {

    if (r->type != DEV_RES_CLOCK_SRC)
      continue;

    dev_clock_node_id_t id = r->u.clock_src.sink_ep;
    if (id < first_sink || id > last_sink)
      continue;

    if (done[id])
      {
        printk("device: multiple resource entries for clock sink end-point %u.\n", dev, id);
        err = -ENOENT;
        goto error;
      }

    struct dev_clock_sink_ep_s *sink = &sinks[id - first_sink];
    struct dev_clock_link_info_s *li = &lkinfo[id - first_sink];

    struct device_s *clock_dev;

    if (device_get_by_path(&clock_dev, r->u.clock_src.src, &device_filter_init_done))
      {
        printk("device: no initialized clock provider available for %p device.\n", dev);
        err = -ENOENT;
        goto error;
      }

    struct device_clock_s clock;

    if (device_get_accessor(&clock, clock_dev, DRIVER_CLASS_CLOCK, 0))
      {
        printk("device: can not use %p device as a clock provider.\n", r->u.clock_src.src);
        err = -EINVAL;
        goto error;
      }

    struct dev_clock_node_info_s info;
    enum dev_clock_node_info_e mask = DEV_CLOCK_INFO_SRC;
    if (lkinfo != NULL)
      {
        li->src_id = r->u.clock_src.src_ep;
        mask |= DEV_CLOCK_INFO_FREQ | DEV_CLOCK_INFO_ACCURACY;
      }

    if (DEVICE_OP(&clock, node_info, r->u.clock_src.src_ep, &mask, &info) ||
        !(mask & DEV_CLOCK_INFO_SRC))
      {
        printk("device: clock provider %p does not have source end-point with node id %u.\n",
               clock_dev, r->u.clock_src.src_ep);
        err = -EINVAL;
      }
    else
      {
        struct dev_clock_src_ep_s  *src = info.src;

        sink->src = src;
        sink->next = src->sink_head;
        src->sink_head = sink;
        if (sink->f_changed != NULL
            && (src->flags & DEV_CLOCK_SRC_EP_VARFREQ)
            && !(src->flags & DEV_CLOCK_SRC_EP_NOTIFY))
          {
            src->flags |= DEV_CLOCK_SRC_EP_NOTIFY;
            src->f_use(src, NULL, DEV_CLOCK_SRC_USE_NOTIFY);
          }

        if (lkinfo != NULL)
          {
            li->src_flags = src->flags;
            if (mask & DEV_CLOCK_INFO_FREQ)
              li->freq = info.freq;
            else
              li->freq = DEV_FREQ_INVALID;
            if (mask & DEV_CLOCK_INFO_ACCURACY)
              li->acc = info.acc;
            else
              li->acc = DEV_FREQ_ACC_INVALID;
          }
      }

      device_put_accessor(&clock);

      if (err)
        goto error;

      done[id]++;
  });

  dev_clock_node_id_t i;
  for (i = 0; i < count; i++)
    if (!done[i])
      {
        printk("device: Unable to link clock sink end-point %u of device %p, no clock source resource entry.\n", i, dev);
        err = -ENOENT;
        goto error;
      }

  return 0;

 error:
  dev_clock_sink_unlink(dev, sinks, count);
  return err;
}

void dev_clock_sink_unlink(struct device_s *dev,
                           struct dev_clock_sink_ep_s *sinks,
                           size_t count)
{
  uint_fast8_t i;

  for (i = 0; i < count; i++)
    {
      struct dev_clock_sink_ep_s *sink = sinks + i;
      struct dev_clock_src_ep_s *src = sink->src;

      if (src == NULL)
        continue;

      bool_t notify = 0;
      struct dev_clock_sink_ep_s **s = &src->sink_head;

      while (*s)
        {
          if (*s == sink)
            {
              *s = sink->next;
              sink->src = NULL;
            }
          else
            {
              notify |= (*s)->f_changed != NULL;
              s = &sink->next;
            }
        }

      if ((src->flags & DEV_CLOCK_SRC_EP_NOTIFY) && !notify)
        {
          src->flags ^= DEV_CLOCK_SRC_EP_NOTIFY;
          src->f_use(src, NULL, DEV_CLOCK_SRC_USE_IGNORE);
        }
    }
}

error_t dev_clock_node_info(struct device_s *dev, dev_clock_node_id_t node_id,
                            enum dev_clock_node_info_e mask,
                            struct dev_clock_node_info_s *info)
{
  struct device_clock_s accessor;

  if (device_get_accessor(&accessor, dev, DRIVER_CLASS_CLOCK, 0))
    return -EINVAL;

  enum dev_clock_node_info_e m = mask;
  error_t err = DEVICE_OP(&accessor, node_info, node_id, &m, info);

  device_put_accessor(&accessor);

  if (!err && m != mask)
    return -EINVAL;

  return err;
}


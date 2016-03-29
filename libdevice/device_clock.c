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
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014-2016

*/

#include <device/device.h>
#include <device/driver.h>
#include <device/class/cmu.h>

#include <mutek/printk.h>

#ifdef CONFIG_DEVICE_CLOCK_GATING
error_t dev_clock_sink_gate(struct dev_clock_sink_ep_s *sink,
                            enum dev_clock_ep_flags_e gates)
{
  struct dev_clock_src_ep_s *src = sink->src;
  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&src->dev->lock);

  if ((sink->flags ^ gates) & DEV_CLOCK_EP_ANY)
    {
      err = 0;
      sink->flags = gates | (sink->flags & ~DEV_CLOCK_EP_ANY);

      enum dev_clock_ep_flags_e src_gates = src->flags & DEV_CLOCK_EP_ANY;

# ifdef CONFIG_DEVICE_CLOCK_SHARING
      if (src_gates & ~gates)
        {
          /* if a gate disabling is suggested for this sink,
             also check other sinks. */
          struct dev_clock_sink_ep_s *osink = src->sink_head;
          while (osink != NULL)
            {
              gates |= osink->flags & DEV_CLOCK_EP_ANY;
              osink = osink->next;
            }
        }
# endif

      if (src_gates ^ gates)
        {
          /* initiate source end-point gates change */
          union dev_clock_setup_u setup;
          setup.flags = gates | (sink->flags & DEV_CLOCK_EP_SINK_SYNC);
          err = src->f_setup(src, DEV_CLOCK_SETUP_GATES, &setup);
        }
    }

  LOCK_RELEASE_IRQ(&src->dev->lock);

  return err;
}
#endif

void dev_cmu_src_ready(struct dev_clock_src_ep_s *src,
                         enum dev_clock_ep_flags_e gates)
{
  enum dev_clock_ep_flags_e old = src->flags;
  struct dev_clock_sink_ep_s *sink = src->sink_head;

# ifdef CONFIG_DEVICE_CLOCK_SHARING
  while (1)
    {
# endif
      if (sink == NULL)
        return;

      struct device_s *dev = sink->dev;
      const struct driver_s *drv = dev->drv;

      LOCK_SPIN_IRQ(&dev->lock);
      enum dev_clock_ep_flags_e flags = sink->flags;
      /* check that all new requirements are satisfied */
      if (((gates & flags) == flags) && ((old & flags) != flags))
        drv->f_use(sink, DEV_USE_CLOCK_GATES);
      LOCK_RELEASE_IRQ(&dev->lock);

# ifdef CONFIG_DEVICE_CLOCK_SHARING
      sink = sink->next;
    }
# endif
}

static error_t dev_cmu_config_res(struct device_cmu_s *accessor,
                                    dev_cmu_config_id_t config_id)
{
  struct device_s *dev = accessor->dev;
  bool_t done = 0;
  error_t err;

  DEVICE_RES_FOREACH(dev, r, {
      switch (r->type)
        {
        case DEV_RES_CMU_MUX: {
          if (!((r->u.cmu_mux.config >> config_id) & 1))
            break;

          struct dev_freq_ratio_s ratio;
          ratio.num = r->u.cmu_mux.num;
          ratio.denom = r->u.cmu_mux.denom;

          err = DEVICE_OP(accessor, config_mux, r->u.cmu_mux.node,
                          r->u.cmu_mux.parent, &ratio);
          if (err)
            goto err;

          done = 1;
          break;
        }

        case DEV_RES_CMU_OSC: {
          if (!((r->u.cmu_osc.config >> config_id) & 1))
            break;

          struct dev_freq_s freq;
          freq.num = r->u.cmu_osc.num;
          freq.denom = r->u.cmu_osc.denom;
          freq.acc_m = r->u.cmu_osc.acc_m;
          freq.acc_e = r->u.cmu_osc.acc_e;

          err = DEVICE_OP(accessor, config_osc, r->u.cmu_osc.node, &freq);
          if (err)
            goto err;

          done = 1;
          break;
        }

        default:
          if (done)
            goto done;
        }
  });

  if (!done)
    return -ENOENT;

 done:
  return DEVICE_OP(accessor, commit);

 err:
  DEVICE_OP(accessor, rollback);
  return err;
}

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
error_t dev_cmu_config(struct device_cmu_s *accessor,
                         dev_cmu_config_id_t config_id)
{
  struct device_s *dev = accessor->dev;
  error_t err;
  LOCK_SPIN_IRQ(&dev->lock);
  err = dev_cmu_config_res(accessor, config_id);
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}
#endif

error_t dev_cmu_init(const struct driver_s *drv, struct device_s *dev)
{
  struct device_cmu_s acc = {
    .api = (void*)drv->classes[0],
    .dev = dev
  };
  assert(acc.api->class_ == DRIVER_CLASS_CMU);
  return dev_cmu_config_res(&acc, 0);
}

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
void dev_cmu_src_notify(struct dev_clock_src_ep_s *src,
                          struct dev_clock_notify_s *param)
{
  struct dev_clock_sink_ep_s *sink = src->sink_head;

# ifdef CONFIG_DEVICE_CLOCK_SHARING
  while (1)
    {
# endif
      if (sink == NULL)
        return;
      struct device_s *dev = sink->dev;
      const struct driver_s *drv = dev->drv;

      LOCK_SPIN_IRQ(&dev->lock);
      param->sink = sink;
      drv->f_use(param, DEV_USE_CLOCK_NOTIFY);
      LOCK_RELEASE_IRQ(&dev->lock);

# ifdef CONFIG_DEVICE_CLOCK_SHARING
      sink = sink->next;
    }
# endif
}
#endif

static void dev_clock_link(struct dev_clock_src_ep_s *src,
                           struct dev_clock_sink_ep_s *sink)
{
  sink->src = src;
#ifdef CONFIG_DEVICE_CLOCK_SHARING
  sink->next = src->sink_head;
#endif
  src->sink_head = sink;
  src->dev->ref_count++;
}

error_t dev_clock_sink_link(struct dev_clock_sink_ep_s *sink,
                            dev_cmu_node_id_t id,
                            struct dev_freq_s *freq)
{
  struct device_s *dev = sink->dev;
  error_t err = -ENOENT;
  bool_t done = 0;

  DEVICE_RES_FOREACH(dev, r, {

      switch (r->type)
        {
        case DEV_RES_CLK_SRC: {

          if (r->u.clock_src.sink_ep != id)
            continue;

          if (done)
            {
              printk("device: duplicate clock source resource entry for %p device.\n", dev);
              return -EINVAL;
            }

          struct device_cmu_s cmu;

          done = 1;
          if (device_get_accessor_by_path(&cmu.base, &dev->node, r->u.clock_src.src, DRIVER_CLASS_CMU))
            {
              printk("device: no initialized clock provider available for %p device.\n", dev);
              return -ENOENT;
            }

          struct dev_cmu_node_info_s info;
          enum dev_cmu_node_info_e mask = DEV_CMU_INFO_SRC;

          if (freq != NULL)
            mask |= DEV_CMU_INFO_FREQ | DEV_CMU_INFO_ACCURACY;

          LOCK_SPIN_IRQ(&cmu.dev->lock);

          if (DEVICE_OP(&cmu, node_info, r->u.clock_src.src_ep, &mask, &info) ||
              !(mask & DEV_CMU_INFO_SRC))
            {
              printk("device: clock provider %p does not have a source end-point with node id %u.\n",
                     cmu.dev, r->u.clock_src.src_ep);
              err = -EINVAL;
              goto unlock;
            }

          if (freq != NULL)
            {
              if (mask & DEV_CMU_INFO_FREQ)
                {
                  *freq = info.freq;
                  if (!(mask & DEV_CMU_INFO_ACCURACY))
                    freq->acc_e = 0;
                }
              else if (device_get_res_freq(dev, freq, id))
                {
                  printk("device: unable to get frequency %u for device %p.\n", id, dev);
                  err = -EINVAL;
                  goto unlock;
                }
            }

          struct dev_clock_src_ep_s  *src = info.src;

          err = src->f_setup(src, DEV_CLOCK_SETUP_LINK, (void*)&sink);
          if (!err)
            {
              dev_clock_link(src, sink);

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
              sink->flags |= src->flags & DEV_CLOCK_EP_VARFREQ;

              if ((sink->flags & DEV_CLOCK_EP_SINK_NOTIFY) &&
                  src->notify_count++ == 0)
                src->f_setup(src, DEV_CLOCK_SETUP_NOTIFY, NULL);
#endif

              if (~src->flags & sink->flags & DEV_CLOCK_EP_ANY)
                {
                  enum dev_clock_ep_flags_e flags = src->flags | sink->flags;
                  err = src->f_setup(src, DEV_CLOCK_SETUP_GATES, (void*)&flags);
                }
            }

          unlock:;
          LOCK_RELEASE_IRQ(&cmu.dev->lock);

          device_put_accessor(&cmu.base);
          break;
        }

#ifdef CONFIG_DEVICE_CLOCK_THROTTLE
        case DEV_RES_CLK_MODES: {
          if (id != r->u.clock_modes.sink_ep)
            continue;

          sink->mode = sink->mode_ids = r->u.clock_modes.modes;
          break;
        }
#endif
        default:
          break;
        }
    });

  if (!done)
    printk("device: no clock source resource entry for sink %u of device %p.\n", id, dev);

  return err;
}

void dev_clock_sink_unlink(struct dev_clock_sink_ep_s *sink)
{
  struct dev_clock_src_ep_s *src = sink->src;

  if (src == NULL)
    return;

  LOCK_SPIN_IRQ(&src->dev->lock);

  struct dev_clock_sink_ep_s **s = &src->sink_head;
  enum dev_clock_ep_flags_e flags = 0;

#ifdef CONFIG_DEVICE_CLOCK_SHARING
  while (*s)
    {
      if (*s == sink)
        {
          *s = sink->next;
          sink->src = NULL;
        }
      else
        {
          flags |= sink->flags & DEV_CLOCK_EP_ANY;
          s = &sink->next;
        }
    }
#else
  *s = NULL;
  sink->src = NULL;
#endif
  src->dev->ref_count--;

  if (~flags & src->flags & DEV_CLOCK_EP_ANY)
    src->f_setup(src, DEV_CLOCK_SETUP_GATES, (void*)&flags);

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  if ((sink->flags & DEV_CLOCK_EP_SINK_NOTIFY) &&
      --src->notify_count == 0)
    src->f_setup(src, DEV_CLOCK_SETUP_NONOTIFY, NULL);
#endif

  src->f_setup(src, DEV_CLOCK_SETUP_UNLINK, (void*)&sink);

  LOCK_RELEASE_IRQ(&src->dev->lock);
}

error_t dev_cmu_node_info(struct device_cmu_s *accessor,
                            dev_cmu_node_id_t node_id,
                            enum dev_cmu_node_info_e *mask,
                            struct dev_cmu_node_info_s *info)
{
  error_t err;
  LOCK_SPIN_IRQ(&accessor->dev->lock);
  err = DEVICE_OP(accessor, node_info, node_id, mask, info);
  LOCK_RELEASE_IRQ(&accessor->dev->lock);
  return err;
}


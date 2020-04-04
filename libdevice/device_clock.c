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

#undef LOGK_MODULE_ID
#define LOGK_MODULE_ID "devc"

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
  enum dev_clock_ep_flags_e changes = (sink->flags ^ gates) & DEV_CLOCK_EP_ANY;

  if (!changes)
    return 0;

  LOCK_SPIN_IRQ(&src->dev->lock);
# if defined(CONFIG_DEVICE_CLOCK_SHARING)
  if (changes & DEV_CLOCK_EP_CLOCK) {
    if (gates & DEV_CLOCK_EP_CLOCK)
      src->clock_count++;
    else
      src->clock_count--;
  }

  if (changes & DEV_CLOCK_EP_POWER) {
    if (gates & DEV_CLOCK_EP_POWER)
      src->power_count++;
    else
      src->power_count--;
  }
# endif

  sink->flags ^= changes;

  /* initiate source endpoint gates change */
  union dev_clock_src_setup_u setup;
  setup.flags = gates | (sink->flags & DEV_CLOCK_EP_GATING_SYNC);
  err = src->f_setup(src, DEV_CLOCK_SRC_SETUP_GATES, &setup);

  LOCK_RELEASE_IRQ(&src->dev->lock);

  return err;
}
#endif

error_t dev_clock_sink_scaler_set(struct dev_clock_sink_ep_s *sink,
                              const struct dev_freq_ratio_s *scale)
{
  struct dev_clock_src_ep_s *src = sink->src;
  error_t err;

  LOCK_SPIN_IRQ(&src->dev->lock);

  err = src->f_setup(src, DEV_CLOCK_SRC_SETUP_SCALER,
                     (const union dev_clock_src_setup_u *)scale);

  LOCK_RELEASE_IRQ(&src->dev->lock);

  return err;
}

void dev_cmu_src_update_async(struct dev_clock_src_ep_s *src,
                              enum dev_clock_ep_flags_e gates)
{
  enum dev_clock_ep_flags_e old = src->flags;
  struct dev_clock_sink_ep_s *sink = src->sink_head;

  gates &= DEV_CLOCK_EP_ANY;
  src->flags = gates | (old & ~DEV_CLOCK_EP_ANY);

# ifdef CONFIG_DEVICE_CLOCK_SHARING
  while (1)
    {
# endif
      if (sink == NULL)
        return;

      struct device_s *dev = sink->dev;
      const struct driver_s *drv = dev->drv;

      LOCK_SPIN_IRQ(&dev->lock);
      if (!(sink->flags & DEV_CLOCK_EP_GATING_SYNC))
        drv->f_use(sink, DEV_USE_CLOCK_SINK_GATE_DONE);
      LOCK_RELEASE_IRQ(&dev->lock);

# ifdef CONFIG_DEVICE_CLOCK_SHARING
      sink = sink->next;
    }
# endif
}

error_t dev_cmu_configid_set(struct device_s *dev,
                             const struct device_cmu_config_ops_s *ops,
                             uint_fast8_t config_id)
{
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

          err = ops->config_mux(dev, r->u.cmu_mux.node, r->u.cmu_mux.parent, &ratio);
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

          err = ops->config_osc(dev, r->u.cmu_osc.node, &freq);
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
    return 0;

 done:
  return ops->commit(dev);

 err:
  ops->rollback(dev);
  return err;
}

error_t dev_cmu_init(struct device_s *dev,
                     const struct device_cmu_config_ops_s *ops)
{
  return dev_cmu_configid_set(dev, ops, 0);
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
      drv->f_use(param, DEV_USE_CLOCK_SINK_FREQ_CHANGED);
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
#if defined(CONFIG_DEVICE_CLOCK_THROTTLE) && defined(CONFIG_DEVICE_CLOCK_SHARING)
  sink->configid_min = 0;
  src->configid_ctr += 1 << (CONFIG_DEVICE_CLOCK_SHARING_MAX_LOG2 * sink->configid_min);
#endif
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
              logk_error("Duplicate clock source resource entry for %p device", dev);
              return -EINVAL;
            }

          struct device_cmu_s cmu;

          done = 1;
          if (device_get_accessor_by_path(&cmu.base, &dev->node, r->u.clock_src.src, DRIVER_CLASS_CMU))
            {
              logk_error("No initialized clock provider available for %p device", dev);
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
              logk_error("Clock provider %p does not have a source endpoint with node id %u",
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
                  logk_error("Unable to get frequency %u for device %p", id, dev);
                  err = -EINVAL;
                  goto unlock;
                }
            }

          struct dev_clock_src_ep_s  *src = info.src;

          err = src->f_setup(src, DEV_CLOCK_SRC_SETUP_LINK,
                             (const union dev_clock_src_setup_u*)&sink);
          if (!err)
            {
              dev_clock_link(src, sink);

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
              sink->flags |= src->flags & DEV_CLOCK_EP_VARFREQ;

              if ((sink->flags & DEV_CLOCK_EP_FREQ_NOTIFY) &&
                  src->notify_count++ == 0)
                src->f_setup(src, DEV_CLOCK_SRC_SETUP_NOTIFY, NULL);
#endif

#if defined(CONFIG_DEVICE_CLOCK_SHARING)
              // Always update the refcounts
              if (sink->flags & DEV_CLOCK_EP_CLOCK)
                src->clock_count++;

              if (sink->flags & DEV_CLOCK_EP_POWER)
                src->power_count++;
#endif

              // Setup gates if changes are requested
              if (~src->flags & sink->flags & DEV_CLOCK_EP_ANY)
                {
                  enum dev_clock_ep_flags_e flags = src->flags | sink->flags;
                  err = src->f_setup(src, DEV_CLOCK_SRC_SETUP_GATES,
                                     (const union dev_clock_src_setup_u*)&flags);
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

          sink->mode_ids = r->u.clock_modes.modes;
# ifdef CONFIG_DEVICE_CLOCK_SHARING
          sink->configid_min = 0;
# endif
          break;
        }
#endif
        default:
          break;
        }
    });

  if (!done)
    logk_error("No clock source resource entry for sink %u of device %p", id, dev);

  return err;
}

void dev_clock_sink_unlink(struct dev_clock_sink_ep_s *sink)
{
  struct dev_clock_src_ep_s *src = sink->src;

  if (src == NULL)
    return;

  // Remove requirements from endpoint, if any
  if (sink->flags & DEV_CLOCK_EP_ANY)
    dev_clock_sink_gate(sink, 0);

  LOCK_SPIN_IRQ(&src->dev->lock);

  src->dev->ref_count--;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  if ((sink->flags & DEV_CLOCK_EP_FREQ_NOTIFY) &&
      --src->notify_count == 0)
    src->f_setup(src, DEV_CLOCK_SRC_SETUP_NONOTIFY, NULL);
#endif

  src->f_setup(src, DEV_CLOCK_SRC_SETUP_UNLINK,
               (const union dev_clock_src_setup_u*)&sink);

  LOCK_RELEASE_IRQ(&src->dev->lock);
}

error_t dev_cmu_node_info_get(struct device_cmu_s *accessor,
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

#if defined(CONFIG_DEVICE_CLOCK_THROTTLE)
static
error_t dev_clock_src_throttle(struct dev_clock_src_ep_s *src,
                               uint32_t configid_min_old,
                               uint32_t configid_min_new)
{
  error_t err = 0;
  struct dev_clock_src_throttle_s throttle;

  LOCK_SPIN_IRQ(&src->dev->lock);

#if defined(CONFIG_DEVICE_CLOCK_SHARING)
  const uint32_t offs_new = configid_min_new * CONFIG_DEVICE_CLOCK_SHARING_MAX_LOG2;
  const uint32_t offs_old = configid_min_old * CONFIG_DEVICE_CLOCK_SHARING_MAX_LOG2;

  assert(src->configid_ctr);

  src->configid_ctr -= 1 << offs_old;
  src->configid_ctr += 1 << offs_new;

  throttle.configid_old = src->configid_min;

  if (configid_min_new > src->configid_min)
    {
      // Calling sink makes a new max(configid_min)
      throttle.configid_new = configid_min_new;
      src->configid_min = configid_min_new;
    }
  else if (configid_min_new < configid_min_old
           && (src->configid_ctr >> offs_old))
    {
      // Calling sink reduces its configid_min, but there are still some
      // other sinks requesting at least the same
      throttle.configid_new = src->configid_min;
    }
  else
    {
      // Calling sink changes configid_min on source
      // TODO: optimize this loop ?
      for (int8_t configid = CONFIG_DEVICE_CMU_CONFIGID_COUNT - 1;
           configid >= 0;
           --configid) {
        if (src->configid_ctr >> (configid * CONFIG_DEVICE_CLOCK_SHARING_MAX_LOG2)) {
          src->configid_min = configid;
          throttle.configid_new = __MAX(configid, configid_min_new);
          break;
        }
      }
    }
#else
  throttle.configid_old = configid_min_old;
  throttle.configid_new = configid_min_new;
  src->configid_min = configid_min_new;
#endif

  if (throttle.configid_old != throttle.configid_new)
    err = src->f_setup(src, DEV_CLOCK_SRC_SETUP_THROTTLE,
                       (const union dev_clock_src_setup_u *)&throttle);

  LOCK_RELEASE_IRQ(&src->dev->lock);

  return err;
}

error_t dev_clock_sink_throttle(struct dev_clock_sink_ep_s *sink,
                                uint_fast8_t mode_id)
{
  uint32_t configid_min_new;
  uint32_t configid_min_old;

  configid_min_new = sink->mode_ids >> (CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2 * mode_id);
  configid_min_new &= (1 << CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2) - 1;

#if defined(CONFIG_DEVICE_CLOCK_SHARING)
  configid_min_old = sink->configid_min;
#else
  configid_min_old = sink->src->configid_min;
#endif

  if (configid_min_old == configid_min_new)
    return 0;

#if defined(CONFIG_DEVICE_CLOCK_SHARING)
  sink->configid_min = configid_min_new;
#endif

  return dev_clock_src_throttle(sink->src, configid_min_old, configid_min_new);
}
#endif


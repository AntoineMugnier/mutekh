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

error_t dev_clock_ep_use(struct dev_clock_ep_s *sink, struct kroutine_s *done)
{
#ifdef CONFIG_DEBUG
  if (sink->type != DEV_CLOCK_NODE_EP_SINK)
    return -EINVAL;
#endif

  if (sink->u.sink.enabled)
    return 0;

  struct device_clock_s *clk    = NULL;
  struct dev_clock_ep_s *src_ep = sink->u.sink.src;
  if (device_get_accessor(&clk, src_ep->dev, DRIVER_CLASS_CLOCK, 0))
    return -EINVAL;

  /* it is the responsibility of the driver to go up in the clock tree
     the ensure the enabling of the whole clock path. */
  error_t err = DEVICE_OP(clk, gating, sink, 1 /* enable */);
  device_put_accessor(&clk);

  if (err)
    return err;

  sink->u.sink.enabled = 1;
  return 0;
}

error_t dev_clock_ep_release(struct dev_clock_ep_s *sink)
{
#if defined(CONFIG_DEBUG)
  if (sink->type != DEV_CLOCK_NODE_EP_SINK)
    return -EINVAL;
#endif

  if (!sink->u.sink.enabled)
    return 0;

  struct device_clock_s *clk = NULL;
  struct dev_clock_ep_s *src_ep = sink->u.sink.src;
  if (device_get_accessor(&clk, src_ep->dev, DRIVER_CLASS_CLOCK, 0))
    return -EINVAL;

  /* it is the responsibility of the driver to go up in the clock tree
     the ensure the disabling of the whole clock path if necessary. */
  error_t err = DEVICE_OP(clk, gating, sink, 0 /* disable */);
  device_put_accessor(&clk);

  if (err)
    return err;

  sink->u.sink.enabled = 0;
  return 0;
}

error_t dev_clock_config(struct device_clock_s *ckdev,
                         dev_clock_config_id_t config_id)
{
  struct device_s *dev = ckdev->dev;

  DEVICE_RES_FOREACH(dev, r, {
    if (r->type == DEV_RES_CLOCK_RTE && r->u.clock_rte.cfg == config_id)
      {
        error_t err = DEVICE_OP(
          ckdev,
          set_config,
          r->u.clock_rte.in,
          r->u.clock_rte.out,
          r->u.clock_rte.num,
          r->u.clock_rte.denum
        );

        if (err)
          return err;
      }
  });

  return 0;
}

error_t dev_clock_get_freq(struct dev_clock_ep_s   *sink,
                           struct dev_clock_freq_s *freq)
{
#if defined(CONFIG_DEBUG)
  if (sink->type != DEV_CLOCK_NODE_EP_SINK)
    return -EINVAL;
#endif

  struct dev_clock_ep_s *src = sink->u.sink.src;
  freq->integral = src->u.src.freq.integral;
  freq->num      = src->u.src.freq.num;
  freq->denum    = src->u.src.freq.denum;

  return 0;
}

void dev_clock_source_init(struct device_s *dev, struct dev_clock_ep_s *src)
{
  src->dev                 = dev;
#if defined(CONFIG_DEBUG)
  src->type                = DEV_CLOCK_NODE_EP_SOURCE;
#endif
  src->u.src.sink_head     = NULL;
  src->u.src.freq.integral = 0;
  src->u.src.freq.num      = 0;
  src->u.src.freq.denum    = 0;
}

void dev_clock_osc_init(struct device_s       *dev,
                        struct dev_clock_ep_s *src,
                        uint_fast32_t         integral,
                        dev_clock_frac_t      num,
                        dev_clock_frac_t      denum)
{
  src->dev                 = dev;
#if defined(CONFIG_DEBUG)
  src->type                = DEV_CLOCK_NODE_OSCILLATOR;
#endif
  src->u.src.sink_head     = NULL;
  src->u.src.freq.integral = integral;
  src->u.src.freq.num      = num;
  src->u.src.freq.denum    = denum;
}

error_t dev_clock_osc_init_by_id(struct device_s       *dev,
                                 struct dev_clock_ep_s *src,
                                 dev_clock_node_id_t   osc_id)
{
  src->dev             = dev;
#if defined(CONFIG_DEBUG)
  src->type            = DEV_CLOCK_NODE_OSCILLATOR;
#endif
  src->u.src.sink_head = NULL;

  /* find the frequency in the device tree. */
  struct dev_resource_s *osc_res = NULL;

  DEVICE_RES_FOREACH(dev, r, {
    if (r->type == DEV_RES_CLOCK_OSC && r->u.clock_osc.id == osc_id)
      {
        osc_res = r;
        break;
      }
  });

  if (osc_res == NULL)
    {
      printk(
        "device: cannot find a clock oscillator with id %u that is"
        " associated with device %p.\n",
        osc_id,
        dev
      );
      return -ENOENT;
    }

  /* save the frequency in the clock end-point. */
  src->u.src.freq.integral = osc_res->u.clock_osc.integral;
  src->u.src.freq.num      = osc_res->u.clock_osc.num;
  src->u.src.freq.denum    = osc_res->u.clock_osc.denum;

  return 0;
}

void dev_clock_sink_init(struct device_s       *dev,
                         struct dev_clock_ep_s *sink,
                         dev_clock_ep_config_t *config)
{
  sink->dev            = dev;
#if defined(CONFIG_DEBUG)
  sink->type           = DEV_CLOCK_NODE_EP_SINK;
#endif
  sink->u.sink.src     = NULL;
  sink->u.sink.next    = NULL;
  sink->u.sink.enabled = 0;
  sink->u.sink.config  = config;
}

static
error_t dev_clock_find_source(struct device_s       *dev,
                              const char            *path,
                              struct device_clock_s **accessor)
{
  /* look up for the device that provides the clock source. */
  struct device_s *clk_dev = dev;

  error_t err = device_get_by_path(&clk_dev, path, &device_filter_init_done);
  if (err)
    {
      printk(
        "device: cannot find intialized clock source from path `%s'"
        " available for device %p.\n",
        path, dev
      );
      *accessor = NULL;
      return -ENOENT;
    }

  /* fetch the accessor using the clock class. */
  if (device_get_accessor(&accessor, clk_dev, DRIVER_CLASS_CLOCK, 0))
    {
      printk(
        "device: cannot use device %p as an clock controller.\n",
        clk_dev
      );
      *accessor = NULL;
      return -EINVAL;
    }

  return 0;
}

static
error_t dev_clock_find_ep_source(struct device_clock_s *ckdev,
                                 dev_clock_node_id_t   src_id,
                                 struct dev_clock_ep_s **src)
{
  *src = DEVICE_OP(ckdev, get_endpoint, DEV_CLOCK_NODE_EP_SOURCE, src_id);
  if (src == NULL)
    {
      printk(
        "device: clock source %p does not have source endpoint %u.\n",
        src_id
      );
      *src = NULL;
      return -EINVAL;
    }

  return 0;
}

error_t dev_clock_sink_link(struct device_s       *dev,
                            struct dev_clock_ep_s *sink,
                            bool_t                enable)
{
  error_t err = 0;

  /* retreive the source clock from device tree resources. */
  struct dev_resource_s *src_res = NULL;

  DEVICE_RES_FOREACH(dev, r, {
    if (r->type == DEV_RES_CLOCK_SRC)
      {
        src_res = r;
        break;
      }
  });

  if (src_res == NULL)
  {
    printk("device: cannot find clock source in resource list of device %p.\n",
           dev);
    return -EINVAL;
  }

  /* find the clock source device. */
  struct device_clock_s *clk = NULL;
  err = dev_clock_find_source(dev, src_res->u.clock_src.src, &clk);
  if (err)
    return err;

  /* find the clock source endpoint that belongs to device @tt clk. */
  struct dev_clock_ep_s *ep = NULL;
  err = dev_clock_find_ep_source(clk, src_res->u.clock_src.in, &ep);
  if (err)
    return err;

  /* link source and sink. */
  assert(sink->u.sink.next == NULL);
  sink->u.sink.src     = ep;
  sink->u.sink.next    = ep->u.src.sink_head;
  sink->u.sink.enabled = enable;
  ep->u.src.sink_head  = sink;

  /* if the sink is enabled, it needs to be configured. */
  if (enable)
    sink->u.sink.config(sink);

  /* release the accessor. */
  device_put_accessor(&clk);

  return 0;
}


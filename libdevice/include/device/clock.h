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

/**
   @file
   @module {Core::Devices support library}
   @short Device clock and power graph

   This header provides functions designed to manage power and clock
   supplied to a device. This API is designed to be used from any
   device drivers where managing incoming clock and power signals is
   needed.

   Physical clock and power signal connections points are represented
   in the software by source endpoint (@ref dev_clock_src_ep_s) and
   sink endpoint (@ref dev_clock_sink_ep_s) objects allocated in
   device private data by the drivers. The former are handled by clock
   and power provider devices while the latter are handled by clock
   and power consumer devices.

   This API provides gating, throttling and frequency notification
   features only for the clock and power signals associated to the
   sink endpoints owned by the consumer device.

   @section {Endpoints setup in device drivers}

   Endpoints are linked together dynamically when the device drivers
   which own the sink endpoint calls the @ref dev_drv_clock_init on
   initialization. Endpoint links will match actual hardware
   connections as described using @ref DEV_RES_CLK_SRC device
   resources attached to the clock consumer device.

   Depending on the flags passed to the function, some clock and power
   signals may be enabled on initialization (see below). Moreover the
   device driver may learn the current frequency of the input clock
   signal.

   In order to simplify drivers code, an empty @ref dev_clock_src_ep_s
   structure along with dummy implementations of the @ref
   dev_drv_clock_init and the @ref dev_drv_clock_cleanup functions are
   also available when the @ref #CONFIG_DEVICE_CLOCK token is not
   defined.

   @csee dev_clock_sink_link
   @csee dev_clock_sink_unlink
   @end section

   @section {Enabling clock and power signals}

   Depending on the hardware, some gates may only be enabled
   asynchronously. In this case, the device driver will have to handle
   delayed gate enabling events. This also applies when signals
   are enabled at link time (on driver initialization).

   Once linked, the sink endpoint owned by the driver of the consumer
   device can be used in various ways:

   @list @item When @ref #CONFIG_DEVICE_CLOCK_GATING is defined, clock
   and power signals gating can be requested by calling the @ref
   dev_clock_sink_gate function. In the other case, gates can only be
   enabled at endpoint link time and are left enabled until the
   endpoint is unlinked.

   @item When @ref #CONFIG_DEVICE_CLOCK_THROTTLE is defined,
   predefined clock throttling modes can be advised by calling the
   @ref dev_clock_sink_throttle function.

   @item Device drivers can not trigger frequency changes which would
   impact other devices. However, when the clock manager hardware
   provide a frequency scaler for the specific signal associated to
   the endpoint, the @ref dev_clock_sink_scaler_set function may be
   used. This function is useful when the only prescaler available is
   on the clock manager side, for instance for the baud rate generator
   of an UART.
   @end list

   @end section

   @section {Source endpoints}

   Unlike sink endpoints which may belong to any kind of device,
   source endpoints are owned by clock provider devices which
   implement the @ref DRIVER_CLASS_CMU driver API defined in @ref
   {@device/class/cmu.h}.

   @end section

   @section {Device clocking modes and sink endpoints}

   Device drivers may define a set of clocking modes.  Meaning and
   count of said modes is device-specific.  With @ref
   #CONFIG_DEVICE_CLOCK_THROTTLE enabled, devices may dynamically
   change clocking mode for each sink endpoint.  A mapping from device
   clocking mode to peer CMU device configuration IDs is defined in
   the device resources, see @ref #DEV_STATIC_RES_CLOCK_MODES.

   This way, a device driver defines its behavioral modes, and
   platform designer defines the mapping between device modes and
   acceptable peer CMU device configuration.  Each sink endpoint list
   compatible clocking configurations, CMU driver takes the one
   matching all the requirements.

   Mode selection is done asynchronously and sink endpoints will be
   notified of clock changes through usual clock notification ways,
   see @ref DEV_USE_CLOCK_SINK_FREQ_CHANGED and @ref
   DEV_CLOCK_EP_FREQ_NOTIFY.

   @end section
*/

#ifndef __DEVICE_CLOCK_H__
#define __DEVICE_CLOCK_H__

#include <hexo/types.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/types.h>
#include <device/resources.h>

/* forward declaration. */
struct dev_clock_notify_s;
struct dev_clock_src_ep_s;
struct dev_clock_sink_ep_s;

/** @This specifies clock endpoint flags.
    @see dev_clock_src_ep_s
    @see dev_clock_sink_ep_s */
enum dev_clock_ep_flags_e
{
  /** For a source endpoint, this specifies if the power gate is
      currently enabled. For a sink endpoint, this specifies that
      power gate enabling is required. */
  DEV_CLOCK_EP_POWER = 0x01,
  /** For a source endpoint, this specifies if the clock gate is
      currently enabled. For a sink endpoint, this specifies that
      clock gate enabling is required. */
  DEV_CLOCK_EP_CLOCK = 0x02,

  /** This specifies both @ref DEV_CLOCK_EP_POWER and @ref
      DEV_CLOCK_EP_CLOCK */
  DEV_CLOCK_EP_POWER_CLOCK  = 0x03,
  /** This specifies both @ref DEV_CLOCK_EP_POWER and @ref
      DEV_CLOCK_EP_CLOCK as well as future gates. */
  DEV_CLOCK_EP_ANY  = 0x03,
  /** This specifies that no gate is enabled. */
  DEV_CLOCK_EP_NONE = 0x00,

  /** For a sink endpoint, this specifies that the gate enabling
      operations performed by the @ref dev_clock_src_setup_t function
      of the clock provider can not be asynchronous. The clock
      provider driver must return an error if synchronous clock
      enabling is not supported when @ref DEV_CLOCK_SRC_SETUP_LINK is
      invoked. This flag can not be changed once the endpoint has
      been linked. */
  DEV_CLOCK_EP_GATING_SYNC   = 0x04,

  /** @internal For a sink endpoint, this specifies if the clock
      frequency change notification is required. This is ignored
      unless @ref #CONFIG_DEVICE_CLOCK_VARFREQ is defined. */
  DEV_CLOCK_EP_FREQ_NOTIFY = 0x08,

  /** @This specifies if the frequency of the endpoint can
      change. This is ignored unless @ref #CONFIG_DEVICE_CLOCK_VARFREQ
      is defined. */
  DEV_CLOCK_EP_VARFREQ = 0x10,
};

/** @internal @This specifies operations for the @ref
    dev_clock_src_setup_t function. */
enum dev_clock_src_setup_op_e
{
  /** This operation is invoked from the @ref dev_clock_sink_link
      function when a new sink endpoint is about to be linked. A
      pointer to the sink is passed as parameter. The linking is
      aborted if an error is returned.

      When @ref #CONFIG_DEVICE_CLOCK_GATING is defined, this hook may
      be used by the clock provider in order to prepare its internal
      clock tree for gating the clock and power signals associated to
      the endpoint. It is implementation defined if some parts of the
      clock tree are gated at this point.

      Some checks should also be performed. If synchronous gate
      enabling operations is not supported (see @ref
      DEV_CLOCK_SRC_SETUP_GATES) and the @ref DEV_CLOCK_EP_GATING_SYNC flag
      of the sink is set, then the @tt -ENOTSUP error must be
      returned. */
  DEV_CLOCK_SRC_SETUP_LINK,

  /** This operation is used when a sink endpoint has been
      unlinked. @see DEV_CLOCK_SRC_SETUP_LINK */
  DEV_CLOCK_SRC_SETUP_UNLINK,

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  /** This operation is used when the frequency change notification
      flags of the source endpoint is set. This hook can be used by
      the clock provider if it needs to update internal data to
      support frequency change notification. */
  DEV_CLOCK_SRC_SETUP_NOTIFY,

  /** This is similar to @ref DEV_CLOCK_SRC_SETUP_NOTIFY */
  DEV_CLOCK_SRC_SETUP_NONOTIFY,
#endif

  /** This operation sets the value of the frequency scaler. This is
      initiated on a linked sink by a call to @ref
      dev_clock_sink_scaler_set. A default implementation should return
      @tt -ENOTSUP. */
  DEV_CLOCK_SRC_SETUP_SCALER,

  /** This operation requests a gates change on a clock source
      endpoint. This operation is invoked from the @ref
      dev_clock_sink_link function as well as from the @ref
      dev_clock_sink_gate function when @ref
      #CONFIG_DEVICE_CLOCK_GATING is defined.

      Requested gates are passed as parameters.  @ref
      DEV_CLOCK_EP_POWER, @ref DEV_CLOCK_EP_CLOCK and @ref
      DEV_CLOCK_EP_NONE are possible values.

      The clock provider driver must perform the gate changes then
      update the source endpoint @tt flags by calling the @ref
      dev_cmu_src_update_sync function.

      If an enabling operation can be completed immediately, the
      function returns 0. When the requested change takes time, the
      function must return @tt -EAGAIN, unless the @ref
      DEV_CLOCK_EP_GATING_SYNC flag is used. When doing so, the @ref
      dev_cmu_src_update_async function must later be called so that drivers
      associated to sink endpoints are notified that enabling is
      effective. Disabling operations can be silently delayed and do
      not require notification.

      When @ref #CONFIG_DEVICE_CLOCK_GATING is defined, this operation
      may be used multiple times even before the gate changes are
      effective. In the other case this operation is only used when
      endpoints are linked and unlinked. */
  DEV_CLOCK_SRC_SETUP_GATES,

#ifdef CONFIG_DEVICE_CLOCK_THROTTLE
  /** This operation sets the frequency throttle mode of the endpoint.
      This is initiated on a linked sink by a call to @ref
      dev_clock_sink_throttle.  Throttling may be done asynchronously. */
  DEV_CLOCK_SRC_SETUP_THROTTLE,
#endif
};

struct dev_clock_src_throttle_s
{
  /** CMU device config ID requested (at least) before the switch. */
  uint_fast8_t configid_old;
  /** CMU device config ID requested (at least) after the switch. */
  uint_fast8_t configid_new;
};

/** @internal @This contains parameters passed to the @ref
    dev_clock_src_setup_t function. */
union dev_clock_src_setup_u
{
  /** @see DEV_CLOCK_SRC_SETUP_LINK @see DEV_CLOCK_SRC_SETUP_UNLINK */
  struct dev_clock_sink_ep_s *sink;
  /** @see DEV_CLOCK_SRC_SETUP_GATES */
  enum dev_clock_ep_flags_e flags;
  /** @see DEV_CLOCK_SRC_SETUP_SCALER */
  struct dev_freq_ratio_s scale;
#ifdef CONFIG_DEVICE_CLOCK_THROTTLE
  /** @see DEV_CLOCK_SRC_SETUP_THROTTLE */
  struct dev_clock_src_throttle_s throttle;
#endif
};

/** @internal @see dev_clock_src_setup_t */
#define DEV_CLOCK_SRC_SETUP(n) error_t (n) (struct dev_clock_src_ep_s *src, \
                                            enum dev_clock_src_setup_op_e op, \
                                            const union dev_clock_src_setup_u *param)

/** @internal @This is the source endpoint operation function.
    @This is implemented by clock provider drivers and
    should not be called directly.

    The device lock must be held when calling this function.

    @see dev_clock_src_setup_op_e */
typedef DEV_CLOCK_SRC_SETUP(dev_clock_src_setup_t);

/** Clock and power signals source endpoint object. It may
    be used to notify clock frequency changes to the consumer and
    request clock and power gating from the provider.

    When @ref #CONFIG_DEVICE_CLOCK_SHARING is defined, source
    endpoints on a clock provider device can be dynamically linked to
    multiple sink endpoints on consumer devices. This way a provider
    can feed clock and/or power to multiple devices.

    On the clock provider side, a source endpoint is a type of
    internal clock tree node which can be used as a connection point
    from the provider device to an external consumer device.

    When @ref #CONFIG_DEVICE_CLOCK is not defined, this structure
    contains no fields.

    @see dev_clock_sink_ep_s */
struct dev_clock_src_ep_s
{
#ifdef CONFIG_DEVICE_CLOCK
  /** pointer to associated clock/power provider */
  struct device_s         *dev;

  /** @internal pointer to the first linked sink ep in the linked list */
  struct dev_clock_sink_ep_s *sink_head;

  /** @internal pointer to the endpoint configuration function of the clock
      signal provider device. */
  dev_clock_src_setup_t     *f_setup;

# ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  /** @internal number of sink endpoints which currently expect to have a
      frequency change notification. */
  uint8_t                notify_count;
# endif

  /** endpoint flags */
  enum dev_clock_ep_flags_e BITFIELD(flags,8);

# if defined(CONFIG_DEVICE_CLOCK_SHARING)
  uint32_t BITFIELD(power_count,8);
  uint32_t BITFIELD(clock_count,8);
#endif

# if defined(CONFIG_DEVICE_CLOCK_THROTTLE)
#  if defined(CONFIG_DEVICE_CLOCK_SHARING)
  /** @internal counter of sinks requesting a given minimal config ID,
      per config ID. */
  uint32_t BITFIELD(configid_ctr,
                    CONFIG_DEVICE_CLOCK_SHARING_MAX_LOG2
                    * CONFIG_DEVICE_CMU_CONFIGID_COUNT);
#  endif

  /** @internal current minimal config ID requested by source endpoint */
  uint32_t BITFIELD(configid_min,
                    CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2);
# endif
#endif
};

/** Clock and power signal sink endpoint structure. Sink
    endpoints can be dynamically linked to a single clock source
    endpoint.

    When belonging to a clock provider, a sink endpoint is a type of
    internal clock tree node which can be used as an external clock
    source.

    When @ref #CONFIG_DEVICE_CLOCK is not defined, this structure
    contains no fields.

    @see dev_clock_src_ep_s */
struct dev_clock_sink_ep_s
{
#ifdef CONFIG_DEVICE_CLOCK
  /** pointer to associated clock/power consumer */
  struct device_s *dev;

  /** @internal pointer to linked source ep in clock provider device */
  struct dev_clock_src_ep_s *src;

# ifdef CONFIG_DEVICE_CLOCK_SHARING
  /** @internal pointer to sibling sink ep in device sharing the same clock signal */
  struct dev_clock_sink_ep_s *next;
# endif

  /** endpoint flags */
  enum dev_clock_ep_flags_e BITFIELD(flags,8);

# ifdef CONFIG_DEVICE_CLOCK_THROTTLE
  /** @internal lookup table used to convert between device driver
      specific mode id to minimal CMU config mode ID.  This is copied
      from DEV_RES_CLK_MODES resource matching the endpoint at
      initialization */
  uint32_t BITFIELD(mode_ids,
                    CONFIG_DEVICE_CLOCK_MODE_COUNT
                    * CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2);

#  ifdef CONFIG_DEVICE_CLOCK_SHARING
  /** @internal current minimal CMU config mode requested by sink */
  uint32_t BITFIELD(configid_min, CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2);
#  endif
# endif
#endif
};


/** @This enables or disables clock and power signals associated with
    a sink endpoint. This is usually called from a clock/power
    consumer device driver.

    This function returns 0 under the following conditions:
    @list
      @item when disabling gates,
      @item when some gates are already enabled,
      @item when immediate enabling was possible.
        This is always the case when @ref DEV_CLOCK_EP_GATING_SYNC is set.
    @end list

    When enabling a gate requires time and the @ref
    DEV_CLOCK_EP_GATING_SYNC sink flag is not set, the function returns
    @tt -EAGAIN and the @ref dev_use_t function of the associated
    device driver will be called with the @ref DEV_USE_CLOCK_SINK_GATE_DONE
    operation once all requested gates are enabled. */
config_depend(CONFIG_DEVICE_CLOCK_GATING)
error_t dev_clock_sink_gate(struct dev_clock_sink_ep_s *sink,
                            enum dev_clock_ep_flags_e gates);

/** @This changes the clock throttle mode of a sink endpoint.
    This is usually called from a clock/power consumer device driver.

    The value of @tt mode_id is driver specific; a @ref
    DEV_RES_CLK_MODES resource entry for the sink endpoint must define
    minimal config ID for the attached CMU device for each device mode.

    This *may* change some clock frequencies, depending on state of
    other sink endpoints linked to the same source as well as clock
    mode expressed in the device tree. Any frequency change will be
    reported as usual if notifications are enabled on the sink. */
config_depend(CONFIG_DEVICE_CLOCK_THROTTLE)
error_t dev_clock_sink_throttle(struct dev_clock_sink_ep_s *sink,
                                uint_fast8_t mode_id);

/** @This attempts to apply an additional frequency scale factor to
    the clock signal of a source endpoint. This is only permitted
    when the source endpoint has no other linked sink. */
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_clock_sink_scaler_set(struct dev_clock_sink_ep_s *sink,
                              const struct dev_freq_ratio_s *scale);

/** @This is used as argument to the @ref dev_cmu_src_notify
    function and @ref dev_use_t function when used with the @ref
    DEV_USE_CLOCK_SINK_FREQ_CHANGED operation. */
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
struct dev_clock_notify_s
{
  struct dev_clock_sink_ep_s *sink;
  struct dev_freq_s freq;
};

/** @This is an equivalent to @ref dev_clock_sink_scaler_set when calling
    from @ref dev_use_t function of a driver while in a @ref
    #DEV_USE_CLOCK_SINK_FREQ_CHANGED. */
config_depend_alwaysinline(CONFIG_DEVICE_CLOCK,
error_t dev_clock_notify_scaler_set(struct dev_clock_notify_s *notify,
                                const struct dev_freq_ratio_s *scale)
{
  struct dev_clock_src_ep_s *src = notify->sink->src;

  return src->f_setup(src, DEV_CLOCK_SRC_SETUP_SCALER,
                      (const union dev_clock_src_setup_u *)scale);
});
#endif

/** @This initializes a clock source endpoint node. */
config_depend_alwaysinline(CONFIG_DEVICE_CLOCK,
void dev_clock_source_init(struct device_s *dev,
                           struct dev_clock_src_ep_s *src,
                           dev_clock_src_setup_t *setup),
{
  src->dev           = dev;
  src->sink_head     = NULL;
  src->f_setup       = setup;
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  src->notify_count  = 0;
#endif
  src->flags         = 0;
});

/** @This initializes a clock sink endpoint node. */
config_depend_alwaysinline(CONFIG_DEVICE_CLOCK,
void dev_clock_sink_init(struct device_s *dev,
                         struct dev_clock_sink_ep_s *sink,
                         enum dev_clock_ep_flags_e flags),
{
  sink->dev          = dev;
  sink->flags        = flags;
});

/** @This links a clock sink endpoint to the appropriate source
    endpoint of a clock provider device as described in the device
    resources. This function is typically called from a device driver
    initialization function. The endpoint must have been initialized
    previously by calling @ref dev_clock_sink_init.

    If the sink endpoint has been initialized with some initially
    enabled gate flags, the clock provider of the source will be
    required to enable the gates. In this case, when enabling a gate
    requires time and the @ref DEV_CLOCK_EP_GATING_SYNC sink flag is not
    set, the @ref dev_use_t function of the associated device driver
    will be called with the @ref DEV_USE_CLOCK_SINK_GATE_DONE operation once
    all requested gates are enabled.

    When not @tt NULL, the @tt freq parameter is set according to the
    current clock tree multiplexers and scalers configuration, no
    matter if the clock gate is currently enabled. If the clock
    provider does not support reporting frequency, a @ref DEV_RES_FREQ
    resource entry is searched. If none are found, an error is returned.

    When @ref #CONFIG_DEVICE_CLOCK_VARFREQ is defined, and the @ref
    DEV_CLOCK_EP_FREQ_NOTIFY flag is set, any subsequent clock
    frequency change will be reported to the device driver.
*/
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_clock_sink_link(struct dev_clock_sink_ep_s *sink,
                            uint_fast8_t id, struct dev_freq_s *freq);

/** @This unlink a clock sink endpoint. Some gates on the source
    endpoints will be disabled as needed. @see dev_drv_clock_cleanup */
config_depend(CONFIG_DEVICE_CLOCK)
void dev_clock_sink_unlink(struct dev_clock_sink_ep_s *sink);

/** This helper function may be called from the device driver
    initialization function. When @ref #CONFIG_DEVICE_CLOCK is defined
    It takes care of calling @ref dev_clock_sink_init and @ref
    dev_clock_sink_link. It forwards the return value of the latter.

    When @ref #CONFIG_DEVICE_CLOCK is not defined and @tt is not NULL,
    the frequency is searched in the device resources.

    @see dev_drv_clock_cleanup */
ALWAYS_INLINE error_t
dev_drv_clock_init(struct device_s *dev,
                   struct dev_clock_sink_ep_s *sink, uint_fast8_t id,
                   enum dev_clock_ep_flags_e flags, struct dev_freq_s *freq)
{
#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, sink, flags);

  return dev_clock_sink_link(sink, id, freq);
#else
  if (freq == NULL)
    return 0;
  return device_get_res_freq(dev, freq, id);
#endif
}

/** This helper function may be called from the device driver
    initialization and cleanup functions. When @ref
    #CONFIG_DEVICE_CLOCK is defined, @ref dev_clock_sink_unlink is
    called. */
ALWAYS_INLINE void
dev_drv_clock_cleanup(struct device_s *dev,
                      struct dev_clock_sink_ep_s *sink)
{
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(sink);
#endif
}

/** @This adds a clock endpoint link in the device resources.

    This entry defines a link between the source endpoint with the
    node id @tt src_id (relevant to clock provider device) and the
    sink endpoint with node id @tt sink_id (relevant to the present
    device).

    @csee DEV_RES_CLK_SRC
 */
config_depend_and2_alwaysinline(CONFIG_DEVICE_CLOCK, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_clock_src(struct device_s *dev, const char *src_name,
                                 uint_fast8_t src_id, uint_fast8_t sink_id),
{
  struct dev_resource_s *r;

  /* this setup the src pointer thanks to the union and the fact that the src
     field is positioned first. */
  error_t err =
    device_res_alloc_str(dev, DEV_RES_CLK_SRC, src_name, NULL, &r);
  if (err)
    return err;

  r->u.clock_src.src_ep  = src_id;
  r->u.clock_src.sink_ep = sink_id;

  return 0;
})

#ifdef CONFIG_DEVICE_CLOCK

/** @This specifies a clock endpoint link.
    @csee DEV_RES_CLK_SRC
    @see device_res_add_clock_src @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_CLK_SRC(__src, __src_id, __sink_id) \
  {                                                         \
    .type  = DEV_RES_CLK_SRC,                             \
    .u = { .clock_src = {                                   \
      .src = (__src),                                       \
      .src_ep  = (__src_id),                                \
      .sink_ep = (__sink_id),                               \
    } }                                                     \
  }

#else

/** @hidden */
# define DEV_STATIC_RES_CLK_SRC(__src, __src_id, __sink_id) \
  {                                                         \
    .type = DEV_RES_UNUSED,                                 \
  }

#endif

#ifdef CONFIG_DEVICE_CLOCK_THROTTLE

# define _DEV_STATIC_RES_CMU_MODE(i, a) (\
  (a) ? ((a) << (CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2 * i)) : 0)

# define _DEV_STATIC_RES_CLOCK_MODES_VA(a, b, c, d, e, f, g, h, ...)    \
  _DEV_STATIC_RES_CMU_MODE(0, a) | _DEV_STATIC_RES_CMU_MODE(1, b) |   \
  _DEV_STATIC_RES_CMU_MODE(2, c) | _DEV_STATIC_RES_CMU_MODE(3, d) |   \
  _DEV_STATIC_RES_CMU_MODE(4, e) | _DEV_STATIC_RES_CMU_MODE(5, f) |   \
  _DEV_STATIC_RES_CMU_MODE(6, g) | _DEV_STATIC_RES_CMU_MODE(7, h)

/** @experimental @This specifies the mapping between device driver
    throttling clock mode ids and CMU device config ID.  @csee
    DEV_RES_CLK_MODES @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_CLOCK_MODES(__sink_id, ...)           \
  {                                                         \
    .type  = DEV_RES_CLK_MODES,                           \
    .u = { .clock_modes = {                                 \
      .sink_ep = (__sink_id),                               \
      .modes = _DEV_STATIC_RES_CLOCK_MODES_VA(__VA_ARGS__,    \
                                0, 0, 0, 0, 0, 0, 0, 0)     \
    } }                                                     \
  }

#else

/** @hidden */
# define DEV_STATIC_RES_CLOCK_MODES(__sink_id, ...)           \
  {                                                         \
    .type = DEV_RES_UNUSED,                                 \
  }

#endif

#endif

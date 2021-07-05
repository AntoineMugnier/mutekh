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

/**
   @file
   @module {Core::Devices support library}
   @short Clock and power management driver API
   @index {Clock and power management} {Device classes}
   @csee DRIVER_CLASS_CMU

   This header defines the API implemented by clock management units
   drivers along with some related function provided by the device
   library. A clock management driver is responsible for dealing with
   other device driver clock and power management requests as well as
   applying the user requested clock configuration.

   The driver API provides functions to update the internal clock tree
   configuration. These functions are implemented by clock management
   driver and should not be called directly; the library functions
   should be used instead so that the user only has access to a viable
   set of clock configurations.

   A clock management unit device is a black box which contains a
   clock tree made of internal nodes of four different types:
   oscillators, mux, sink endpoints and source endpoints. Each node
   in the internal tree is assigned a compile time numerical id.

   The driver provides functions to specify and apply a configuration
   for some of the internal nodes of the tree. The following list
   describes the role of the various node types:

   @list
   @item Oscillator nodes are used to represent a clock source
   internal to the device. They usually represent a fixed frequency
   crystal directly managed by the device or an internal oscillator
   with an optionally variable frequency. The API allow specifying
   the frequency and accuracy of the oscillator. An oscillator node
   do not have a parent in the tree.

   @item Mux nodes are used to choose between multiple parent clock
   nodes and optionally scale the frequency of the selected
   parent. The set of usable parent nodes is usually restricted and
   may even be of size 1. Likewise the possible values for the scale
   factor is usually restricted by the hardware. They are used to
   represent clock multiplexer, divider and PLLs in the hardware tree.

   @item Sink endpoint nodes have an associated @ref
   dev_clock_sink_ep_s object. They represent an input connected to
   the output of an other clock management unit. They do not have a
   parent in the tree and do not need configuration.

   @item Source endpoint nodes have an associated @ref
   dev_clock_src_ep_s object. They represent a clock output which can
   be connected to the input of consumer device. Depending on clock
   tree, they may need configuration (selectable parent, ratio,
   etc.). They may also raise management request operations through
   the @ref dev_clock_src_setup_t function.
   @end list

   Links between endpoint may convey a clock signal, a power supply
   or both. The driver of the clock management unit must be able to
   handle clock management operations on its source endpoints needed
   by driver of linked consumer devices. This includes endpoint
   linking operations, gating operations and frequency change
   notifications. This operations are defined in @ref {@device/clock.h}.

   @section {CMU device configuration selection}

   Some sets of valid node configuration entries may be defined by
   attaching @ref DEV_RES_CMU_MUX and @ref DEV_RES_CMU_OSC resources
   to a clock management device. Each entry may be associated to one
   or more configuration id which can be activated.

   Power/Clock configuration IDs are defined with an order.  When
   multiple configurations are possible, driver will select the one
   with the lowest ID.

   The configuration 0 is loaded from the cmu driver initialization be
   calling the @ref dev_cmu_init function.

   On a running platform, CMU device configuration mode is a merge of:
   @list
   @item Application-required configuration ID,
   @item Connected sink endpoint clocking modes (when @ref
   #CONFIG_DEVICE_CLOCK_THROTTLE is defined).
   @end list

   The user may change the set of active node configuration entries at
   any time by calling @ref dev_cmu_app_configid_set_t.

   Because of configuration ID ordering, platform designer should
   order its configuration IDs from lowest power consumption to
   highest.

   @end section
*/

#ifndef __DEVICE_CMU_H__
#define __DEVICE_CMU_H__

#include <device/resources.h>
#include <device/clock.h>

#include <mutek/kroutine.h>

struct device_cmu_s;

#define CONFIG_DEVICE_CLOCK_MASKW (1 << (CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2))

/** Index of the configuration associated with a set of @ref
    DEV_RES_CMU_MUX and @ref DEV_RES_CMU_OSC resources in the
    device tree. */
typedef uint_fast8_t dev_cmu_config_id_t;

/** Mask of configuration ids @see dev_cmu_config_id_t */
# if CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2 == 6
typedef uint64_t dev_cmu_config_mask_t;
# elif CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2 == 5
typedef uint32_t dev_cmu_config_mask_t;
# elif CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2 < 5
typedef uint16_t dev_cmu_config_mask_t;
# else
#  error CONFIG_DEVICE_CMU_CONFIGID_COUNT_LOG2: unsupported value
# endif

/** Index of the clock tree node inside a device. Nodes are numbered
    as follow: source eps nodes, sink eps nodes, internal nodes,
    oscillator nodes. */
typedef uint_fast8_t dev_cmu_node_id_t;

/** @see dev_cmu_config_mux_t */
#define DEV_CMU_CONFIG_MUX(n) error_t (n) (              \
    struct device_s *dev,                         \
    dev_cmu_node_id_t   node_id,                           \
    dev_cmu_node_id_t   parent_id,                         \
    struct dev_freq_ratio_s *ratio                         \
)

/** @internal @This sets the next configuration of a clock mux node
    internal to the device.

    This function acts on endpoint nodes and internal clock signals
    nodes.  The function have to select the mux to the parent clock
    node inside the device and optionally update the clock scale
    factor associated to this mux.

    No hardware configuration actually takes place before the call to
    the @ref dev_cmu_commit_t function. The device lock must be held
    when calling this function. */
typedef DEV_CMU_CONFIG_MUX(dev_cmu_config_mux_t);



/** @see dev_cmu_config_osc_t */
#define DEV_CMU_CONFIG_OSC(n) error_t (n) (                   \
    struct device_s *dev,                            \
    dev_cmu_node_id_t   node_id,                              \
    struct dev_freq_s *freq                                   \
)

/** @internal @This sets the next configuration of a clock oscillator
    node internal to the device.

    @This sets the oscillator frequency value and the accuracy
    values.

    No hardware configuration actually takes place before the call to
    the @ref dev_cmu_commit_t function. The device lock must be held
    when calling this function. */
typedef DEV_CMU_CONFIG_OSC(dev_cmu_config_osc_t);



/** @see dev_cmu_commit_t */
#define DEV_CMU_COMMIT(n) error_t (n) (struct device_s *dev)

/** @internal @This starts the configuration of the clocks based on
    parameters passed to previous calls to the @ref
    dev_cmu_config_mux_t and @ref dev_cmu_config_osc_t
    functions.

    The driver may further delay the configuration of some clock
    signals and make it effective only when appropriate. Depending on
    the hardware, this may happen immediately or may be delayed until
    the clock signal is not used anymore. The driver may also choose
    to internally switch to an alternate clock source during the PLL
    lock period. In any case the clock provider must ensure that clock
    signals keep running smoothly if currently in use or return an
    error.

    The @ref dev_cmu_src_notify function is called by the driver
    for all impacted source endpoints once the change has occurred.
    The device lock must be held when calling this function. */
typedef DEV_CMU_COMMIT(dev_cmu_commit_t);



/** @see dev_cmu_rollback_t */
#define DEV_CMU_ROLLBACK(n) error_t (n) (struct device_s *dev)

/** @internal @This discard all configuration changes requests made by
    calling the @ref dev_cmu_config_mux_t and @ref
    dev_cmu_config_osc_t functions. This can be used to
    revert to a known state in case of error.

    The device lock must be held when calling this function. */
typedef DEV_CMU_ROLLBACK(dev_cmu_rollback_t);


/** @This specifies node information to retrieve for the @ref
    dev_cmu_node_info_t function. */
enum dev_cmu_node_info_e
{
  DEV_CMU_INFO_FREQ    = 0x01,
  DEV_CMU_INFO_NAME    = 0x02,
  DEV_CMU_INFO_PARENT  = 0x04,
  DEV_CMU_INFO_RUNNING = 0x08,
  DEV_CMU_INFO_SINK    = 0x10,
  DEV_CMU_INFO_SRC     = 0x20,
  DEV_CMU_INFO_ACCURACY = 0x40,
  /** For scale-enabled source endpoints */
  DEV_CMU_INFO_SCALE   = 0x80,
};

/** @This stores node information retrieved by the @ref
    dev_cmu_node_info_t function. */
struct dev_cmu_node_info_s
{
  /** Node current clock frequency */
  struct dev_freq_s          freq;
  /** Node name */
  const char                 *name;
  /** Node id of the currently selected parent node in the tree. */
  dev_cmu_node_id_t        parent_id;
  /** Specifies if the clock is currently running. */
  bool_t                     running;
  /** Pointer to sink endpoint object when relevant */
  struct dev_clock_sink_ep_s *sink;
  /** Pointer to source endpoint object when relevant */
  struct dev_clock_src_ep_s  *src;
  /** Source endpoint frequency scale */
  struct dev_freq_ratio_s    scale;
};

/** @see dev_cmu_node_info_t */
#define DEV_CMU_NODE_INFO(n) error_t (n) (                              \
    struct device_cmu_s *accessor,                                      \
    dev_cmu_node_id_t node_id,                                          \
    enum dev_cmu_node_info_e *mask,                                     \
    struct dev_cmu_node_info_s *info                                    \
)

/** @internal @This retrieves information about an internal clock
    node. The @tt mask parameter specifies which information are
    fetched and is updated according to what is actually available.

    Driver may set more fields than actually requested by @tt mask
    upon calling.

    The device lock must be held when calling this function.

    @see dev_cmu_node_info_s
    @see dev_cmu_node_info_e
    @see dev_cmu_node_info */
typedef DEV_CMU_NODE_INFO(dev_cmu_node_info_t);

/** @see dev_cmu_app_configid_set_t */
#define DEV_CMU_APP_CONFIGID_SET(n) error_t (n) (                              \
    struct device_cmu_s *accessor,                                      \
    uint_fast8_t config_id)

/** @internal @This informs the device of configuration id required by
    the application code.  Driver should merge it with requirements
    from source endpoints, if @ref #CONFIG_DEVICE_CLOCK_THROTTLE is
    defined.

    @see dev_cmu_app_configid_set */
typedef DEV_CMU_APP_CONFIGID_SET(dev_cmu_app_configid_set_t);

/**
   @internal @this defines an API for a CMU driver. It must be passed
   to libdevice helpers to accomplish various management tasks.
 */
struct device_cmu_config_ops_s
{
  dev_cmu_config_mux_t *config_mux;
  dev_cmu_config_osc_t *config_osc;
  dev_cmu_commit_t     *commit;
  dev_cmu_rollback_t   *rollback;
};

/**
   @this is a helper to declare a @ref device_cmu_config_ops_s
   structure.
 */
#define DRIVER_CMU_CONFIG_OPS_DECLARE(prefix)                           \
  static const struct device_cmu_config_ops_s prefix##_config_ops = {   \
    .config_mux = prefix ## _config_mux,                                \
    .config_osc = prefix ## _config_osc,                                \
    .commit = prefix ## _commit,                                        \
    .rollback = prefix ## _rollback,                                    \
  }

DRIVER_CLASS_TYPES(DRIVER_CLASS_CMU, cmu,
                   dev_cmu_node_info_t        *f_node_info;
                   dev_cmu_app_configid_set_t *f_app_configid_set;
                   );

#define DRIVER_CMU_METHODS(prefix)                                  \
  ((const struct driver_class_s*)&(const struct driver_cmu_s){      \
    .class_ = DRIVER_CLASS_CMU,                                     \
    .f_node_info = prefix ## _node_info,                            \
    .f_app_configid_set = prefix ## _app_configid_set,              \
  })

/** @This must be used to perform initial clock configuration.

    This function is similar to @ref dev_cmu_config but is designed
    to be called from the initialization function of cmu
    device drivers. It selects resource associated to config id 0. */
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_cmu_init(struct device_s *dev,
                     const struct device_cmu_config_ops_s *ops);

/**
   @internal This helper iterates on resources to configure relevant
   muxes and oscillator nodes, then commits the changes.
 */
error_t dev_cmu_configid_set(struct device_s *dev,
                             const struct device_cmu_config_ops_s *ops,
                             uint_fast8_t config_id);

/** @internal This helper function is called by the clock provider
    device driver when the frequency of a clock source endpoint
    changes.

    This function will propagate the change to all connected sink
    endpoints by calling the @ref dev_use_t function of the
    associated device driver with the @ref DEV_USE_CLOCK_SINK_FREQ_CHANGED
    operation. The @tt sink field of @tt param is set by the function.
 */
config_depend(CONFIG_DEVICE_CLOCK_VARFREQ)
void dev_cmu_src_notify(struct dev_clock_src_ep_s *src,
                        struct dev_clock_notify_s *param);

/** This helper updates the gates state of a source endpoint when
    enabled synchronously from endpoint setup handler. @This is called
    by the clock provider device driver when the state of the
    requested gates has been updated. */
config_depend_alwaysinline(CONFIG_DEVICE_CLOCK,
void dev_cmu_src_update_sync(struct dev_clock_src_ep_s *src,
                             enum dev_clock_ep_flags_e gates),
{
  enum dev_clock_ep_flags_e old = src->flags;
  src->flags = gates | (old & ~(DEV_CLOCK_EP_POWER | DEV_CLOCK_EP_CLOCK));
});

/** @This function is called by the clock provider device driver when
    the requested gates has been enabled asynchronously.

    This function will propagate the change to all connected sink
    endpoints by calling the @ref dev_use_t function of the
    associated device driver with the @ref DEV_USE_CLOCK_SINK_GATE_DONE
    operation. */
config_depend(CONFIG_DEVICE_CLOCK_GATING)
void dev_cmu_src_update_async(struct dev_clock_src_ep_s *src,
                              enum dev_clock_ep_flags_e gates);

/** @This tells whether a source endpoint currently requests power.
 */
config_depend(CONFIG_DEVICE_CLOCK)
ALWAYS_INLINE
bool_t dev_clock_src_is_power_requested(const struct dev_clock_src_ep_s *src)
{
#if !defined(CONFIG_DEVICE_CLOCK)
  return 0;
#elif defined(CONFIG_DEVICE_CLOCK_SHARING)
  return src->power_count != 0;
#else
  return src->sink_head && !!(src->sink_head->flags & DEV_CLOCK_EP_POWER);
#endif
}

/** @This tells whether a source endpoint currently requests clock.
 */
config_depend(CONFIG_DEVICE_CLOCK)
ALWAYS_INLINE
bool_t dev_clock_src_is_clock_requested(const struct dev_clock_src_ep_s *src)
{
#if !defined(CONFIG_DEVICE_CLOCK)
  return 0;
#elif defined(CONFIG_DEVICE_CLOCK_SHARING)
  return src->clock_count != 0;
#else
  return src->sink_head && !!(src->sink_head->flags & DEV_CLOCK_EP_CLOCK);
#endif
}

/** @This is a wrapper for the @ref dev_cmu_node_info_t function
    which takes care of locking the device. */
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_cmu_node_info_get(struct device_cmu_s *accessor,
                          dev_cmu_node_id_t node_id,
                          enum dev_cmu_node_info_e *mask,
                          struct dev_cmu_node_info_s *info);

/** @This adds an internal clock mux entry in the device resource list.

    This entry specifies the parent clock signal used to generate the
    clock for an other clock signal node. This entry belongs to a
    configuration set that is defined by @tt config_mask. All clock
    related resources associated with a given configuration id can be
    applied by calling @ref dev_cmu_config.

    The parent clock can be scaled up or down by a fraction defined by
    @tt fnum and @tt fdenom parameters.

    Depending on the clock device driver and clock signal, a default
    mux may be used if no resource entry is present for a given
    node.

    @b note: the validity of the mux and scaling factor is the
    responsibility of the programmer. There may be no internal
    validity check on clocking configurations.

    @csee DEV_RES_CMU_MUX */
config_depend_and2_alwaysinline(CONFIG_DEVICE_CLOCK, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_cmu_mux(struct device_s     *dev,
                                   dev_cmu_node_id_t parent_id,
                                   dev_cmu_node_id_t node_id,
                                   dev_cmu_config_mask_t config_mask,
                                   const struct dev_freq_ratio_s *ratio),
{
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_CMU_MUX);
  if (err)
    return err;

  r->u.cmu_mux.parent = parent_id;
  r->u.cmu_mux.node   = node_id;
  r->u.cmu_mux.config = config_mask;
  r->u.cmu_mux.num    = ratio->num;
  r->u.cmu_mux.denom  = ratio->denom;

  return 0;
})

#ifdef CONFIG_DEVICE_CLOCK

/** @This specifies a clock mux resource entry in a static
    device resources table declaration.
    @csee DEV_RES_CMU_MUX @see device_res_add_cmu_mux
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_CMU_MUX(__parent_id, __node_id,                 \
                                __config_mask, __num, __denom)          \
  {                                                                     \
    .type = DEV_RES_CMU_MUX,                                            \
      .u = { .cmu_mux = {                                               \
        .parent = (__parent_id),                                        \
        .node   = (__node_id),                                          \
        .config = (__config_mask),                                      \
        .num    = (__num),                                              \
        .denom  = (__denom),                                            \
      } }                                                               \
  }

/** @This specifies a clock source oscillator resource entry in a
    static device resources table declaration. A default value is used
    for frequency accuracy.
    @csee DEV_RES_CMU_OSC @see device_res_add_cmu_osc
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_CMU_OSC(__node_id, __config_id, __num, __denom) \
  {                                                               \
    .type = DEV_RES_CMU_OSC,                                    \
      .u = { .cmu_osc = {                                       \
        .node     = (__node_id),                                  \
        .config   = (__config_id),                                \
        .num      = (__num),                                      \
        .denom    = (__denom),                                    \
        .acc_m    = 7,                                            \
        .acc_e    = 31,                                           \
      } }                                                         \
  }

/** @This is similar to @cref #DEV_STATIC_RES_CMU_OSC */
# define DEV_STATIC_RES_CMU_OSC_ACC(__node_id, __config_id,             \
                                    __num, __denom, _acc_m, _acc_e)     \
  {                                                               \
    .type = DEV_RES_CMU_OSC,                                    \
      .u = { .cmu_osc = {                                       \
        .node     = (__node_id),                                  \
        .config   = (__config_id),                                \
        .num      = (__num),                                      \
        .denom    = (__denom),                                    \
        .acc_m    = (_acc_m),                                     \
        .acc_e    = (_acc_e),                                     \
      } }                                                         \
  }

#else

/** @hidden */
# define DEV_STATIC_RES_CMU_MUX(__parent_id, __node_id,                 \
                                __config_mask, __num, __denom)          \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }

/** @hidden */
# define DEV_STATIC_RES_CMU_OSC(__node_id, __config_id, __num, __denom) \
  {                                                               \
    .type = DEV_RES_UNUSED,                                       \
  }

/** @hidden */
# define DEV_STATIC_RES_CMU_OSC_ACC(__node_id, __config_id,             \
                                    __num, __denom, _acc_m, _acc_e)     \
  {                                                               \
    .type = DEV_RES_UNUSED,                                       \
  }

#endif

/** @This adds a clock frequency resource entry to a device.

    This entry specifies a frequency value for an internal oscillator
    node. This entry belongs to a configuration set that is defined by
    @tt config_mask. All clock related resources associated with a
    given configuration id can be applied by calling @ref
    dev_cmu_config.

    This may be used to specify the frequency of an external clock
    source or to configure the frequency of an internal oscillator.
    In the later case, the internal oscillator may have a default
    frequency value and the resource entry is not mandatory.

    @csee DEV_RES_CMU_OSC
 */
config_depend_and2_alwaysinline(CONFIG_DEVICE_CLOCK, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_cmu_osc(struct device_s     *dev,
                                 dev_cmu_node_id_t node_id,
                                 dev_cmu_config_mask_t config_mask,
                                 uint64_t            num,
                                 uint32_t            denom),
{
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_CMU_OSC);
  if (err)
    return err;

  r->u.cmu_osc.node     = node_id;
  r->u.cmu_osc.config   = config_mask;
  r->u.cmu_osc.num      = num;
  r->u.cmu_osc.denom    = denom;

  return 0;
})

#endif

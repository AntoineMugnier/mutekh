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

#ifndef __DEVICE_CLOCK_H__
#define __DEVICE_CLOCK_H__

#include <hexo/types.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>

#include <mutek/kroutine.h>


/** Index of the clock tree node inside a device. Nodes are numbered
    as follow: source eps nodes, sink eps nodes, internal nodes,
    oscillator nodes. */
typedef uint_fast8_t dev_clock_node_id_t;

/** @This specify a reserved invalid node id value */
#define DEV_CLOCK_INVALID_NODE_ID ((1 << CONFIG_DEVICE_CLOCK_MAX_ID) - 1)

/** @This helpers provide a numerical representation of an internal node
    signal. */
#define DEV_CLOCK_NODE_EDGE(src, dst) (((dst) << CONFIG_DEVICE_CLOCK_MAX_ID) | (src))

/** Index of the configuration associated with a set of @ref
    DEV_RES_CLOCK_RTE and @ref DEV_RES_CLOCK_OSC resources in the
    device tree. */
typedef uint_fast8_t dev_clock_config_id_t;

/** Mask of configuration ids @see dev_clock_config_id_t */
typedef uint32_t dev_clock_config_mask_t;



/* forward declaration. */
struct device_clock_s;
struct dev_clock_src_ep_s;
struct dev_clock_sink_ep_s;

/** @This is used by dev_clock_src_use_t to notify */
struct dev_clock_ready_s
{
  struct kroutine_s kr;
  void *pv;
};

/** @This specifies the action performed by the @ref
    dev_clock_src_use_t function. */
enum dev_clock_src_use_e
{
  DEV_CLOCK_SRC_USE_HOLD,
  DEV_CLOCK_SRC_USE_RELEASE,
  DEV_CLOCK_SRC_USE_NOTIFY,
  DEV_CLOCK_SRC_USE_IGNORE,
};

/** @see dev_clock_ep_use_t */
#define DEV_CLOCK_SRC_USE(n) error_t (n) (struct dev_clock_src_ep_s *src, \
                                          struct dev_clock_ready_s *ready, \
                                          enum dev_clock_src_use_e action)

/** @This tells a driver with a clock source end-point if the
    associated clock signal is enabled and has changes notification.

    This is called by @ref dev_clock_sink_hold with @ref
    DEV_CLOCK_SRC_USE_HOLD when @tt src->running is false. This is
    called by @ref dev_clock_sink_release with @ref
    DEV_CLOCK_SRC_USE_RELEASE when @tt src->use_count becomes zero.

    This function is called with the lock of the source end-point
    device held. The @tt use_count field of the end-point is
    updated before the call. The function is responsible for
    updating the @ref dev_clock_src_ep_s::running field.
*/
typedef DEV_CLOCK_SRC_USE(dev_clock_src_use_t);

/** Clock signal source end-point structure. A source end-point is a
    type of clock tree node which can be used as a connection point
    between two devices. Source end-points can be dynamically linked
    to multiple sink end-points so that a clock provider device can
    feed a clock signal to multiple clock consumer devices. */
struct dev_clock_src_ep_s
{
  struct device_s         *dev;

  /** pointer to the first linked sink ep in the linked list */
  struct dev_clock_sink_ep_s *sink_head;

  /** pointer to the use/release function of the clock signal provider */
  dev_clock_src_use_t     *f_use;

  /** indicates if the clock is currently running. This can be 0
      when the clock is not ready yet even if @tt use_count > 0.
      @see dev_clock_sink_hold @see dev_clock_sink_release
      @see dev_clock_src_use_t
  */
  bool_t                  running:1;

  /** indicates if at least one linked sink end-point has a non @tt
      NULL @ref dev_clock_sink_ep_s::f_changed function pointer. */
  bool_t                  notify:1;

  /** number of sink endpoints which currently need to have this
      clock running. This counter is protected by @tt dev->lock.
      @see dev_clock_sink_hold @see dev_clock_sink_release
      @see dev_clock_src_use_t
  */
  uint16_t                use_count;
};

/** @see dev_clock_sink_changed_t */
#define DEV_CLOCK_SINK_CHANGED(n) void (n) (struct dev_clock_sink_ep_s *ep, \
                                            const struct dev_freq_s *freq)

/** @This notifies a driver with a clock sink end-point that the
    frequency has changed. */
typedef DEV_CLOCK_SINK_CHANGED(dev_clock_sink_changed_t);


/** Clock signal sink end-point structure. A sink end-point is a
    type of clock tree node which can be used as a connection point
    between two devices. Sink end-points can be dynamically linked
    to a single clock source end-point. */
struct dev_clock_sink_ep_s
{
  struct device_s         *dev;

  /** pointer to linked source ep in clock provider device */
  struct dev_clock_src_ep_s *src;

  /** pointer to sibling sink ep in device sharing the same clock signal */
  struct dev_clock_sink_ep_s *next;

  /** pointer to the clock change notification callback, may be NULL. */
  dev_clock_sink_changed_t *f_changed;
};



/** @see dev_clock_config_node_t */
union dev_clock_config_value_u
{
  struct dev_freq_s    freq;
  struct dev_freq_ratio_s    ratio;
};

/** @see dev_clock_config_node_t */
#define DEV_CLOCK_CONFIG_NODE(n) error_t (n) (                \
    struct device_clock_s *ckdev,                            \
    dev_clock_node_id_t   node_id,                           \
    dev_clock_node_id_t   parent_id,                         \
    union dev_clock_config_value_u *value                    \
)

/** @This set the next configuration of a clock node internal to the
    device.

    For oscillator nodes, this function sets the oscillator frequency
    value. The @tt parent_id parameter is not relevant in this case.

    For other types of node (end-point and internal clock signals),
    the function have to select the route to the parent clock node
    inside the device and optionally update the clock scale factor
    associated to this route. The @tt value parameter may be @tt NULL.

    No hardware configuration actually takes place before the call to
    the @ref dev_clock_commit_t function.
*/
typedef DEV_CLOCK_CONFIG_NODE(dev_clock_config_node_t);



/** @see dev_clock_commit_t */
#define DEV_CLOCK_COMMIT(n) error_t (n) (struct device_clock_s *ckdev)

/** @This starts the configuration of the clocks based on parameters
    passed to previous calls to the @ref dev_clock_config_node_t
    function.

    The driver may further delay the configuration of some clock
    signals and make it effective only when appropriate. Depending on
    the hardware, this may happen immediately or may be delayed until
    the clock signal is not used anymore. The driver may also choose
    to internally switch to an alternate clock source during the PLL
    lock period. In any case the clock provider must ensure that
    clock signals keep running smoothly if currently in use.

    The @ref dev_clock_src_changed function is called by the driver
    for all impacted source end-points once the change has occurred.
 */
typedef DEV_CLOCK_COMMIT(dev_clock_commit_t);



/** @see dev_clock_rollback_t */
#define DEV_CLOCK_ROLLBACK(n) error_t (n) (struct device_clock_s *ckdev)

/** @This discard all configuration changes requests made by calling
    the @ref dev_clock_config_node_t function. This can be used to
    revert to a known state in case of error.
 */
typedef DEV_CLOCK_ROLLBACK(dev_clock_rollback_t);


/** @This specifies node information to retrieve for the @ref
    dev_clock_node_info_t function. */
enum dev_clock_node_info_e
{
  DEV_CLOCK_INFO_FREQ    = 0x01,
  DEV_CLOCK_INFO_NAME    = 0x02,
  DEV_CLOCK_INFO_PARENT  = 0x04,
  DEV_CLOCK_INFO_RUNNING = 0x08,
  DEV_CLOCK_INFO_SINK    = 0x10,
  DEV_CLOCK_INFO_SRC     = 0x20,
};

/** @This stores node information retrieved by the @ref
    dev_clock_node_info_t function. */
struct dev_clock_node_info_s
{
  struct dev_freq_s    freq;
  const char                 *name;
  dev_clock_node_id_t        parent_id;
  bool_t                     running;
  struct dev_clock_sink_ep_s *sink;
  struct dev_clock_src_ep_s  *src;
};

/** @This retrieves information about an internal clock node. The @tt
    mask parameter indicates which information are fetched and is
    updated according to what is actually available.
    @see dev_clock_node_info_s @see dev_clock_node_info_e */
#define DEV_CLOCK_NODE_INFO(n) error_t (n) (                             \
    struct device_clock_s *ckdev,                                       \
    dev_clock_node_id_t node_id,                                        \
    enum dev_clock_node_info_e *mask,                                   \
    struct dev_clock_node_info_s *info                                  \
)

/** @This returns the requested node information */
typedef DEV_CLOCK_NODE_INFO(dev_clock_node_info_t);



DRIVER_CLASS_TYPES(clock,
                   dev_clock_node_info_t   *f_node_info;
                   dev_clock_config_node_t *f_config_node;
                   dev_clock_commit_t      *f_commit;
                   dev_clock_rollback_t    *f_rollback;
                   );

/** @This increases the clock source use count. The kroutine inside
    @tt ready is invoked when the clock source is running. If @tt
    ready is @tt NULL, the function spins until the clock is running. */
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_clock_sink_hold(struct dev_clock_sink_ep_s *sink,
                            struct dev_clock_ready_s *ready);

/** @This decreases the clock source use count.

    Depending on the current policy, when the counter value reaches 0,
    the device stack may walks up the clock tree in order to disable
    the clock. Depending on the number of enabled shared clocks,
    multiple level of clocks in the tree might be disabled. */
config_depend(CONFIG_DEVICE_CLOCK)
void dev_clock_sink_release(struct dev_clock_sink_ep_s *sink);

/** @This is convenience wrapper for the @ref dev_clock_node_info_t
    function. An error is returned if the requested information are
    not available. */
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_clock_node_info(struct device_s *dev,
                            dev_clock_node_id_t node_id,
                            enum dev_clock_node_info_e mask,
                            struct dev_clock_node_info_s *info);

/** This function updates the configuration of the device internal
    nodes using resource entries from the device tree associated with
    a given configuration id.

    The configuration id selects all resources with the corresponding
    bit set in the config mask. */
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_clock_config(struct device_clock_s *ckdev,
                         dev_clock_config_id_t config_id);

/** @This function is called by the clock provider device driver when
    the frequency of a clock source end-point change if the @ref
    dev_clock_src_ep_s::notify field is set.

    This function will propagate the change to all connected sink
    end-points by calling the @ref dev_clock_sink_changed_t function
    of the sinks.
 */
config_depend(CONFIG_DEVICE_CLOCK)
void dev_clock_src_changed(struct device_clock_s *ckdev,
                           struct dev_clock_src_ep_s *src,
                           const struct dev_freq_s *freq);

/** @This initializes a clock source end-point node. */
config_depend(CONFIG_DEVICE_CLOCK)
ALWAYS_INLINE
void dev_clock_source_init(struct device_s *dev,
                           struct dev_clock_src_ep_s *src,
                           dev_clock_src_use_t *use)
{
  memset(src, 0, sizeof(*src));
  src->dev           = dev;
  src->f_use         = use;
}

/** @This initializes a clock sink end-point node. The @tt changed
    parameter may be @tt NULL. */
config_depend(CONFIG_DEVICE_CLOCK)
ALWAYS_INLINE
void dev_clock_sink_init(struct device_s       *dev,
                         struct dev_clock_sink_ep_s *sink,
                         dev_clock_sink_changed_t *changed)
{
  memset(sink, 0, sizeof(*sink));
  sink->dev           = dev;
  sink->f_changed     = changed;
}

/** @This links multiple clock sink end-points to the appropriate
    source end-point of a clock provider device as described in the
    device resources. An array of sink end-points must be passed along
    with the range of node ids associated to the sink end-points.

    If the @tt freqs pointer is not @tt NULL, the array is filled with
    the current frenquency values of end-points. If the @tt src_id
    pointer is not @tt NULL, the array is filled with node id of the
    source end-point relevant to the clock provider device.

    This function is typically called from device driver
    initialization function.
 */
config_depend(CONFIG_DEVICE_CLOCK)
error_t dev_clock_sink_link(struct device_s *dev,
                            struct dev_clock_sink_ep_s *sinks,
                            struct dev_freq_s *freqs,
                            dev_clock_node_id_t *src_id,
                            dev_clock_node_id_t first_sink,
                            dev_clock_node_id_t last_sink);

/** @This unlinks all clock sink end-points in the array. End-points
     which are not linked are skipped. */
config_depend(CONFIG_DEVICE_CLOCK)
void dev_clock_sink_unlink(struct device_s *dev,
                           struct dev_clock_sink_ep_s *sinks,
                           size_t count);

/** @This adds an internal clock route entry in the device resource list.

    This entry specifies the parent clock signal used to generate the
    clock for an other clock signal node. This route belongs to a
    configuration set that is defined by @tt config_mask. All clock
    related resources associated with a given configuration id can be
    applied by calling @ref dev_clock_config.

    The parent clock can be scaled up or down by a fraction defined by
    @tt fnum and @tt fdenom parameters.

    Depending on the clock device driver and clock signal, a default
    route may be used if no resource entry is present for a given
    node.

    @b note: the validity of the route and scaling factor is the
    responsibility of the programmer. There may be no internal
    validity check on clocking configurations.

    @see #DEV_STATIC_RES_CLK_RTE
 */
config_depend(CONFIG_DEVICE_CLOCK)
ALWAYS_INLINE
error_t device_add_res_clock_route(struct device_s     *dev,
                                   dev_clock_node_id_t parent_id,
                                   dev_clock_node_id_t node_id,
                                   dev_clock_config_mask_t config_mask,
                                   uint32_t            fnum,
                                   uint32_t            fdenom)
{
#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_CLOCK_RTE);
  if (err)
    return err;

  r->u.clock_rte.parent = parent_id;
  r->u.clock_rte.node   = node_id;
  r->u.clock_rte.config = config_mask;
  r->u.clock_rte.num    = fnum;
  r->u.clock_rte.denom  = fdenom;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_CLOCK

/** @This can be used to include a clock route resource entry in a static
    device resources table declaration.

    @see device_res_add_clock_rte @see #DEV_DECLARE_STATIC_RESOURCES
 */
# define DEV_STATIC_RES_CLK_RTE(__parent_id, __node_id,                 \
                                __config_mask, __num, __denom)          \
  {                                                                     \
    .type = DEV_RES_CLOCK_RTE,                                          \
      .u = { .clock_rte = {                                             \
        .parent = (__parent_id),                                        \
        .node   = (__node_id),                                          \
        .config = (__config_mask),                                      \
        .num    = (__num),                                              \
        .denom  = (__denom),                                            \
      } }                                                               \
  }

#else

# define DEV_STATIC_RES_CLK_RTE(__parent_id, __node_id,                 \
                                __config_mask, __num, __denom)          \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }

#endif

/** @This adds a clock frequency resource entry to a device.

    This entry specifies a frequency value for an internal oscillator
    node. This route belongs to a configuration set that is defined by
    @tt config_mask. All clock related resources associated with a
    given configuration id can be applied by calling @ref
    dev_clock_config.

    This may be used to specify the frequency of an external clock
    source or to configure the frequency of an internal oscillator.
    In the later case, the internal oscillator may have a default
    frequency value and the resource entry is not mandatory.

    @see #DEV_STATIC_RES_CLK_OSC
 */
config_depend(CONFIG_DEVICE_CLOCK)
ALWAYS_INLINE
error_t device_add_res_clock_osc(struct device_s     *dev,
                                 dev_clock_node_id_t node_id,
                                 dev_clock_config_mask_t config_mask,
                                 uint64_t            num,
                                 uint32_t            denom)
{
#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_CLOCK_OSC);
  if (err)
    return err;

  r->u.clock_osc.node     = node_id;
  r->u.clock_osc.config   = config_mask;
  r->u.clock_osc.num      = num;
  r->u.clock_osc.denom    = denom;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_CLOCK

/** @This can be used to include a clock source oscillator resource entry in a
    static device resources table declaration.
    @see device_res_add_clock_osc @see #DEV_DECLARE_STATIC_RESOURCES
 */
# define DEV_STATIC_RES_CLK_OSC(__node_id, __config_id, __num, __denom) \
  {                                                               \
    .type = DEV_RES_CLOCK_OSC,                                    \
      .u = { .clock_osc = {                                       \
        .node     = (__node_id),                                  \
        .config   = (__config_id),                                \
        .num      = (__num),                                      \
        .denom    = (__denom),                                    \
      } }                                                         \
  }

#else

# define DEV_STATIC_RES_CLK_OSC(__node_id, __config_id, __num, __denom) \
  {                                                               \
    .type = DEV_RES_UNUSED,                                       \
  }

#endif

/** @This adds a clock end-point link in the device resources.

    This entry defines a link between the source end-point with the
    node id @tt src_id (relevant to clock provider device) and the
    sink end-point with node id @tt sink_id (relevant to the present
    device).

    @see #DEV_STATIC_RES_CLK_SRC
 */
config_depend(CONFIG_DEVICE_CLOCK)
ALWAYS_INLINE
error_t device_add_res_clock_src(struct device_s     *dev,
                                 const char          *src_name,
                                 dev_clock_node_id_t src_id,
                                 dev_clock_node_id_t sink_id)
{
#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_resource_s *r;

  /* this setup the src pointer thanks to the union and the fact that the src
     field is positioned first. */
  error_t err =
    device_res_alloc_str(dev, DEV_RES_CLOCK_SRC, src_name, NULL, &r);
  if (err)
    return err;

  /* force dependence checking for clock source using source name. */
  r->flags |= DEVICE_RES_FLAGS_DEPEND;

  r->u.clock_src.src_ep  = src_id;
  r->u.clock_src.sink_ep = sink_id;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_CLOCK

/** @This can be used to include a clock source oscillator resource entry in a
    static device resources table declaration. The @tt src parameter must be
    statically allocated.
    @see device_res_add_clock_src @see #DEV_DECLARE_STATIC_RESOURCES
 */
# define DEV_STATIC_RES_CLK_SRC(__src, __src_id, __sink_id) \
  {                                                         \
    .type  = DEV_RES_CLOCK_SRC,                             \
    .flags = DEVICE_RES_FLAGS_DEPEND,                       \
    .u = { .clock_src = {                                   \
      .src = (__src),                                       \
      .src_ep  = (__src_id),                                \
      .sink_ep = (__sink_id),                               \
    } }                                                     \
  }

#else

# define DEV_STATIC_RES_CLK_SRC(__src, __src_id, __sink_id) \
  {                                                         \
    .type = DEV_RES_UNUSED,                                 \
  }

#endif

#endif


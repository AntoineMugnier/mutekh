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


/** Index of the config associated with a set of @ref
    DEV_RES_CLOCK_RTE resources in the device tree.
    @see dev_clock_ep_enable */
typedef uint_fast8_t dev_clock_config_id_t;

/** Representation of the numerator or denumerator of a fractional frequency.
    */
typedef uint_fast16_t dev_clock_frac_t;

/** @This specifies clock tree node types. */
enum dev_clock_node_type_e
{
  DEV_CLOCK_NODE_NONE,

  /** Device source end-point node. Source end-points represent clock
      signal output from a device. They can be dynamically linked to
      multiple sink end-points of clock receiver devices. */
  DEV_CLOCK_NODE_EP_SOURCE,

  /** Device sink end-point node. Sink end-points represent clock
      inputs of a device. One sink end-point can be dynamically linked
      to a single source end-point. */
  DEV_CLOCK_NODE_EP_SINK,

  /** Device internal clock signal node. */
  DEV_CLOCK_NODE_INTERNAL,

  /** Device internal clock signal wired to a fixed frequency oscillator. */
  DEV_CLOCK_NODE_OSCILLATOR,
};

/** freq in Hz = integral + num / denum */
struct dev_clock_freq_s
{
  uint32_t integral;
  uint16_t num;
  uint16_t denum;
};

/* forward declaration. */
struct device_clock_s;
struct dev_clock_ep_s;

#define DEV_CLOCK_EP_CONFIG(n) void (n) (struct dev_clock_ep_s *ep)

/** @This notifies the callee that the clock frequency has
    changed on an end-point. */
typedef DEV_CLOCK_EP_CONFIG(dev_clock_ep_config_t);

#define DEV_CLOCK_EP_GATING(n) void (n) (struct dev_clock_ep_s *ep, \
                                         bool_t enable)

/** @This notifies the callee that its clock must be enabled. */
typedef DEV_CLOCK_EP_GATING(dev_clock_ep_gating_t);

struct dev_clock_ep_s
{
  struct device_s            *dev;

#ifdef CONFIG_DEBUG
  enum dev_clock_node_type_e type;
#endif

  union {
    struct {
      /** pointer to linked source ep */
      struct dev_clock_ep_s *src;
      /** pointer to sibling sink ep */
      struct dev_clock_ep_s *next;
      /** specify if the sink ep currently needs a clock */
      bool_t                enabled;

      dev_clock_ep_config_t *config;
      dev_clock_ep_gating_t *gating;
    } sink;

    struct {
      /** pointer to first sink ep in linked list */
      struct dev_clock_ep_s   *sink_head;
      /** current frequency on the link, updated on configuration. */
      struct dev_clock_freq_s freq;
    } src;
  } u;
};

#define DEVCLOCK_GATING(n) error_t (n) (struct device_clock_s *ckdev, \
                                        struct dev_clock_ep_s *sink,  \
                                        bool_t                enable)

/** @This enables or disables the clock gating which drives the specified
    clock signal node. */
typedef DEVCLOCK_GATING(devclock_gating_t);


#define DEVCLOCK_SET_CONFIG(n) error_t (n) (struct device_clock_s *ckdev, \
                                            dev_clock_node_id_t   src_id, \
                                            dev_clock_node_id_t   dst_id, \
                                            dev_clock_frac_t      num,    \
                                            dev_clock_frac_t      denum)

/** @This activates an internal route between two clock signal nodes
    inside the device and sets the clock factor fraction associated to
    this route. Alternate routes to the destination node are discarded. */
typedef DEVCLOCK_SET_CONFIG(devclock_set_config_t);


#define DEVCLOCK_GET_ENDPOINT(n) struct dev_clock_ep_s * (n) ( \
  struct device_clock_s      *ckdev,                           \
  enum dev_clock_node_type_e type,                             \
  dev_clock_node_id_t        node_id                           \
)

/** @This returns a pointer to the end-point with given node id. This
    function returns @tt NULL for internal nodes.

    If the @tt type parameter is not @tt NULL, it is updated with the
    type of node. */
typedef DEVCLOCK_GET_ENDPOINT(devclock_get_endpoint_t);

DRIVER_CLASS_TYPES(clock,
                   devclock_gating_t       *f_gating;
                   devclock_set_config_t   *f_set_config;
                   devclock_get_endpoint_t *f_get_endpoint;
                   );

/** @This increases the clock source use count. If the clock can not
    be enabled immediately, @tt EAGAIN is returned and the kroutine is
    invoked when the clock source is running again. */
error_t dev_clock_ep_use(struct dev_clock_ep_s *sink, struct kroutine_s *done);

/** @This decreases the clock source use count.

    Depending on the current policy, when the counter value reaches 0,
    the device stack may walks up the clock tree in order to disable
    the clock. Depending on the number of enabled shared clocks,
    multiple level of clocks in the tree might be disabled. */
error_t dev_clock_ep_release(struct dev_clock_ep_s *sink);

/** This helper function calls the driver @ref devclock_set_config_t
    for all nodes associated to a specific configuration id in the
    device tree. */
error_t dev_clock_config(struct device_clock_s *ckdev,
                         dev_clock_config_id_t config_id);

/** This helper function provide the frequency at a given sink in the clock
    path. */
error_t dev_clock_get_freq(struct dev_clock_ep_s   *sink,
                           struct dev_clock_freq_s *freq);

/** @This initializes a clock source end-point. */
void dev_clock_source_init(struct device_s *dev, struct dev_clock_ep_s *src);

/** @This initializes a device clock oscillator with its configuration in the
    device resource list. The arguments @tt integral, @tt num and @tt denum are
    provided from information stored in the device tree.

    This function is typically called from device initialization function that
    needs to have its internal oscillator node configured.
 */
void dev_clock_osc_init(struct device_s       *dev,
                        struct dev_clock_ep_s *osc,
                        uint_fast32_t         integral,
                        dev_clock_frac_t      num,
                        dev_clock_frac_t      denum);

/** @This initializes a device clock oscillator with its configuration in the
    device resource list. The @tt osc_id is used to retreive the frequency
    information from the device tree.

    This function is typically called from device initialization function that
    needs to have its internal oscillator node configured.

    @return 0 in case of success or a negative error code.
 */
error_t dev_clock_osc_init_by_id(struct device_s       *dev,
                                 struct dev_clock_ep_s *osc,
                                 dev_clock_node_id_t   osc_id);

/** @This initializes a clock sink end-point. */
void dev_clock_sink_init(struct device_s       *dev,
                         struct dev_clock_ep_s *sink,
                         dev_clock_ep_config_t *config);

/** @This links a device clock sink end-points to the appropriate source
    end-point of a clock generator device as described in the device
    resources.

    This function is typically called from device initialization function which
    needs to have its clock input configured. This function also updates the
    end-point with the current clock frequency value.
 */
error_t dev_clock_sink_link(struct device_s       *dev,
                            struct dev_clock_ep_s *sink,
                            bool_t                enable);

/** @This adds an internal clock route entry in the device resource list.

    This entry specifies a clock route from a @tt src clock source node to
    a @tt sink clock consumer node. This route belongs to a predefined
    configuration that is identified by @tt id.

    The clock source can be scaled up or down using a fraction that is
    represented by two 16-bit integral values @tt fnum and @tt fdenum.

    IMPORTANT note: the validity of the route and scaling factor is the
    responsibility of the programmer. There is no internal validity check
    on clocking configurations.

    @see #DEV_STATIC_RES_CLK_RTE
 */
static inline
error_t device_add_res_clock_route(struct device_s       *dev,
                                   dev_clock_node_id_t   src,
                                   dev_clock_node_id_t   sink,
                                   dev_clock_config_id_t config_id,
                                   uint16_t              fnum,
                                   uint16_t              fdenum)
{
#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_CLOCK_RTE);
  if (err)
    return err;

  r->u.clock_rte.in    = src;
  r->u.clock_rte.out   = sink;
  r->u.clock_rte.cfg   = config_id;
  r->u.clock_rte.num   = fnum;
  r->u.clock_rte.denum = fdenum;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_CLOCK

/** @This can be used to include a clock route resource entry in a static
    device resources table declaration. The config parameter must be a valid
    configuration id as well as the source and sink identifiers.
    @see device_res_add_clock_rte @see #DEV_DECLARE_STATIC_RESOURCES
 */
# define DEV_STATIC_RES_CLK_RTE(__src_id, __sink_id, __cfg, __num, __denum) \
  {                                                                         \
    .type = DEV_RES_CLOCK_RTE,                                              \
    .u = { .clock_rte = {                                                   \
      .in    = (__src_id),                                                  \
      .out   = (__sink_id),                                                 \
      .cfg   = (__cfg),                                                     \
      .num   = (__num),                                                     \
      .denum = (__denum),                                                   \
    } }                                                                     \
  }

#else

# define DEV_STATIC_RES_CLK_RTE(__src_id, __sink_id, __cfg, __num, __denum) \
  {                                                                         \
    .type = DEV_RES_UNUSED,                                                 \
  }

#endif

/** @This adds a internal clock source oscillator in the device resources.

    This entry specifies a clock source oscillator with frequency information.

    The node id can be used as a clock source in clock routing (@see
    device_add_res_clock_route).

    The frequency is represented using a fraction of integral numbers and of
    the form: a + b / c. The parameters are detailed as follows:

    @list
      @item An @em integral i32-bit value, representing the integral
      part of the frequency.
      @item A @em num 16-bit value as the numerator of the fractional part.
      @item A @em denum 16-bit value as the denumerator of the fractional part.
    @end list

    @see #DEV_STATIC_RES_CLK_OSC
 */
static inline
error_t device_add_res_clock_osc(struct device_s     *dev,
                                 dev_clock_node_id_t node_id,
                                 uint32_t            integral,
                                 dev_clock_frac_t    num,
                                 dev_clock_frac_t    denum)
{
#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_CLOCK_OSC);
  if (err)
    return err;

  r->u.clock_osc.id       = node_id;
  r->u.clock_osc.integral = integral;
  r->u.clock_osc.num      = num;
  r->u.clock_osc.denum    = denum;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_CLOCK

/** @This can be used to include a clock source oscillator resource entry in a
    static device resources table declaration.
    @see device_res_add_clock_rte @see #DEV_DECLARE_STATIC_RESOURCES
 */
# define DEV_STATIC_RES_CLK_OSC(__id, __integral, __num, __denum) \
  {                                                               \
    .type = DEV_RES_CLOCK_OSC,                                    \
    .u = { .clock_osc = {                                         \
      .id       = (__id),                                         \
      .integral = (__integral),                                   \
      .num      = (__num),                                        \
      .denum    = (__denum),                                      \
    } }                                                           \
  }

#else

# define DEV_STATIC_RES_CLK_OSC(__id, __integral, __num, __denum) \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }

#endif

/** @This adds a clock source in the device resources.

    This entry specifies a clock source link from@tt src_id
    source id in the device denominated by @tt src to @tt sink_id in the
    present device.

    @see #DEV_STATIC_RES_CLK_SRC
 */
static inline
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

  r->u.clock_src.in  = src_id;
  r->u.clock_src.out = sink_id;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_CLOCK

/** @This can be used to include a clock source oscillator resource entry in a
    static device resources table declaration. The @tt src parameter must be
    statically allocated.
    @see device_res_add_clock_rte @see #DEV_DECLARE_STATIC_RESOURCES
 */
# define DEV_STATIC_RES_CLK_SRC(__src, __src_id, __sink_id) \
  {                                                         \
    .type  = DEV_RES_CLOCK_SRC,                             \
    .flags = DEVICE_RES_FLAGS_DEPEND,                       \
    .u = { .clock_src = {                                   \
      .src = (__src),                                       \
      .in  = (__src_id),                                    \
      .out = (__sink_id),                                   \
    } }                                                     \
  }

#else

# define DEV_STATIC_RES_CLK_SRC(__src, __src_id, __sink_id) \
  {                                                         \
    .type = DEV_RES_UNUSED,                                 \
  }

#endif

#endif


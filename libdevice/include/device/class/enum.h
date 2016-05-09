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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

/**
   @file
   @module {Core::Devices support library}
   @short Device enumerator driver API
   @index {Device enumerator} {Device classes}
   @csee DRIVER_CLASS_ENUM
*/

#ifndef __DEVICE_ENUM_H__
#define __DEVICE_ENUM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/request.h>

struct device_enum_s;

enum dev_enum_rq_type_e
{
  /** @This request waits for a change in the list of child devices.
      The request terminates when the @tt rev field become lower than
      the current revision counter of the enumerator. The @tt rev
      field is updated to the last value of the counter. */
  DEV_ENUM_LIST_EVENT,

  /** @This request waits for an existing child device driver to
      complete the initialization of the specified class. The @tt
      ref_count of the device is increased if the request terminates
      successfully. */
  DEV_ENUM_INIT_EVENT,
};

struct dev_enum_rq_s
{
  struct dev_request_s base;

  enum dev_enum_rq_type_e type:8;
  error_t error;

  union {
    struct {
      device_enum_rev_t                 rev;
    }                                   list;

    struct {
      struct device_s                   *dev;
      const struct driver_class_s       *api;
      enum driver_class_e               class_:8;
    }                                   init;
  };
};

STRUCT_INHERIT(dev_enum_rq_s, dev_request_s, base);

/** @see dev_enum_match_driver_t */
#define DEV_ENUM_MATCH_DRIVER(n) bool_t (n)(struct device_enum_s *accessor, \
                                            const struct dev_enum_ident_s *ident, \
                                            size_t count, struct device_s *dev)

/** @This determines if the @tt drv driver is suitable to drive the
    @tt dev device. The device must have been enumerated by the @tt
    accessor device. @This generally relies on the enumeration ids table
    provided by the driver. */
typedef DEV_ENUM_MATCH_DRIVER(dev_enum_match_driver_t);

/** @see dev_enum_request_t */
#define DEV_ENUM_REQUEST(n) void (n)(const struct device_enum_s *accessor, \
                                     struct dev_enum_rq_s *rq)

/** @This enqueues a request. */
typedef DEV_ENUM_REQUEST(dev_enum_request_t);


/** @see dev_enum_cancel_t */
#define DEV_ENUM_CANCEL(n) error_t (n)(const struct device_enum_s *accessor, \
                                       struct dev_enum_rq_s *rq)

/** @This cancel a request. */
typedef DEV_ENUM_CANCEL(dev_enum_cancel_t);



DRIVER_CLASS_TYPES(DRIVER_CLASS_ENUM, enum,
                   dev_enum_match_driver_t *f_match_driver;
                   dev_enum_request_t *f_request;
                   dev_enum_cancel_t *f_cancel;
                   );

/** @see driver_enum_s */
#define DRIVER_ENUM_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_enum_s){   \
    .class_ = DRIVER_CLASS_ENUM,                                  \
    .f_match_driver = prefix ## _match_driver,                    \
    .f_request = prefix ## _request,                              \
    .f_cancel = prefix ## _cancel,                                \
  })

config_depend_and2_inline(CONFIG_DEVICE_ENUM, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_enum_wait_request(const struct device_enum_s *accessor,
                              struct dev_enum_rq_s *rq),
{
    struct dev_request_status_s status;

    dev_request_sched_init(&rq->base, &status);
    DEVICE_OP(accessor, request, rq);
    dev_request_sched_wait(&status);

    return rq->error;
})

/** @internal @This is a generic implementation of the @ref
    DEV_ENUM_INIT_EVENT request handler suitable for most enum
    drivers. The @ref dev_drv_enum_request_generic function handle all
    type of requests. */
config_depend(CONFIG_DEVICE_ENUM)
void dev_drv_enum_init_enqueue(dev_request_queue_root_t *q,
                               struct dev_enum_rq_s *rq);

/** @internal @This is a generic implementation of the @ref
    DEV_USE_ENUM_CHILD_INIT operation handler suitable for most enum
    drivers. */
config_depend(CONFIG_DEVICE_ENUM)
void dev_drv_enum_child_init(dev_request_queue_root_t *q,
                             struct device_s *cdev);

/** @internal @This is a generic implementation of the @ref
    dev_enum_request_t function suitable for most enum drivers. The
    @ref dev_drv_enum_init_enqueue function should be used when only
    the @ref DEV_ENUM_INIT_EVENT type of request requires a generic
    implementation. */
config_depend(CONFIG_DEVICE_ENUM)
void dev_drv_enum_request_generic(dev_request_queue_root_t *q,
                                  struct device_s *dev,
                                  struct dev_enum_rq_s *rq);

/** @internal @This is a generic implementation of the @ref
    dev_enum_cancel_t function suitable for most enum driver. */
config_depend(CONFIG_DEVICE_ENUM)
error_t dev_drv_enum_cancel_generic(dev_request_queue_root_t *q,
                                    struct device_s *dev,
                                    struct dev_enum_rq_s *rq);

#endif


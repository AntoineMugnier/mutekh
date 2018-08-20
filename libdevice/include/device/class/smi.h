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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file
   @module {Core::Devices support library}
   @short System Management Interface driver class

   @section{Purpose} The @em {SMI} class gives access to Media
   Independant Interface Management bus defined in IEEE802.3.

   @end section
 */

#ifndef __DEVICE_SMI_H__
#define __DEVICE_SMI_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>

struct device_s;
struct driver_s;
struct device_smi_s;
struct driver_smi_s;

enum dev_smi_request_type_e
{
  DEV_SMI_READ,
  DEV_SMI_WRITE,
};

struct dev_smi_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
  };

  /** Destination address on bus */
  uint16_t phy:5;
  /** Register number in device */
  uint16_t reg:5;

  /** Operation */
  enum dev_smi_request_type_e BITFIELD(type,1);

  /** Value buffer. */
  uint16_t value;

};

DEV_REQUEST_INHERIT(smi); DEV_REQUEST_QUEUE_OPS(smi);

/** @see dev_smi_request_t */
#define DEV_SMI_REQUEST(n) void (n) (                              \
    const struct device_smi_s *accessor,                           \
    struct dev_smi_rq_s *rq)

/** @This enqueues a request.

   The kroutine of the request may be executed from within this
   function. Please read @xref {Nested device request completion}. 

   @xsee {Purpose} @see dev_smi_request_type_e
*/
typedef DEV_SMI_REQUEST(dev_smi_request_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_SMI, smi,
    dev_smi_request_t *f_request;
);

/** @see driver_smi_s */
#define DRIVER_SMI_METHODS(prefix)                                    \
  ((const struct driver_class_s*)&(const struct driver_smi_s){        \
    .class_ = DRIVER_CLASS_SMI,                                       \
    .f_request = prefix ## _request,                                  \
  })

/** @This is scheduler wait wrapper for the @ref dev_smi_request_t function */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_request(
    const struct device_smi_s *accessor,
    struct dev_smi_rq_s *rq),
{
  struct dev_request_status_s status;

  dev_request_sched_init(&rq->base, &status);
  DEVICE_OP(accessor, request, rq);
  dev_request_sched_wait(&status);

  return rq->error;
});

/** @This perform a @ref DEVICE_SMI_READ operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_read(
    const struct device_smi_s *accessor,
    uint8_t phy, uint8_t reg,
    uint16_t *data),
{
  struct dev_smi_rq_s rq = {
    .type = DEVICE_SMI_READ,
    .attribute = attribute,
    .phy = phy,
    .reg = reg,
  };

  error_t err = dev_smi_wait_request(accessor, &rq);

  *data = rq.data;

  return err;
});

/** @This perform a @ref DEVICE_SMI_WRITE operation and stop the
    scheduler context during the request. */
config_depend_and2_inline(CONFIG_DEVICE_SMI, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_smi_wait_write(
    const struct device_smi_s *accessor,
    uint8_t phy, uint8_t reg,
    uint16_t data),
{
  struct dev_smi_rq_s rq = {
    .type = DEVICE_SMI_WRITE,
    .attribute = attribute,
    .phy = phy,
    .reg = reg,
    .data = data,
  };

  return dev_smi_wait_request(accessor, &rq);
});

#ifdef CONFIG_DEVICE_SMI
# define DEV_STATIC_RES_DEV_SMI(path_)                          \
  DEV_STATIC_RES_DEVCLASS_PARAM("smi", path_, DRIVER_CLASS_SMI)
# define DEV_STATIC_RES_SMI_BITRATE(bitrate_)   \
  {                                             \
    .type = DEV_RES_UINT_PARAM,                 \
      .u = { .uint_param = {                    \
        .name = "bitrate",                      \
        .value = bitrate_,                      \
      } }                                       \
  }
#else
# define DEV_STATIC_RES_DEV_SMI(path_)                                  \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
# define DEV_STATIC_RES_SMI_BITRATE(bitrate_)                           \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

#endif

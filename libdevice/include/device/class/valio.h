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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

/**
   @file
   @module{Devices support library}
   @short Value IO class

   @section{Conceptual}

   Value IO class abstracts access to a set of values of
   non-normalized format and encoding.

   A Value IO device defines a list of attributes. Each has:
   @list
   @item a static identifier,
   @item a fixed size.
   @end list

   User may request reading, writing, or wait for update on any
   relevant attribute value.

   The attribute device class does not enforce any specific format on
   values, identifiers, or access capacity.  It serves as a generic
   transport API between devices and client code.  Some external
   libray may define identifiers, data encoding, and allowed subset of
   access methods for each attribute.  This is out of the scope of
   this class.

   @end section

   @section{Goals, non-goals}

   Goal of this class is to provide a driver framework for all devices
   where there is no standard class.  This is mostly useful for
   sensors that retrieve application-specific data.

   As this class is bi-directionnal, setting attribute values may be
   used to control actuators, or even set parameters for other
   attributes.

   Non-goal of the class is to provide a totally generic framework
   able to replace need for all other classes.  This class is not an
   abstraction of the class abstraction.  We may not try to fit
   everything in this class, even if it may sound cool.  In
   particular, streaming data sources (like ADC and sound devices) are
   not expected to use this class.

   Having just a data pointer with an implied size enforces this
   non-goal.

   @end section
 */

#ifndef __DEVICE_VALIO_H__
#define __DEVICE_VALIO_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>

struct device_s;
struct driver_s;
struct device_valio_s;
struct driver_valio_s;

/**
   @this defines attribute access type for a request.
 */
enum dev_valio_request_type_e {
    DEVICE_VALIO_READ,
    DEVICE_VALIO_WRITE,
    DEVICE_VALIO_WAIT_UPDATE,
};

/**
   @this defines a request on an attribute.
 */
struct dev_valio_rq_s
{
    struct dev_request_s base;

    /** Attribute index, should be accounted for in
        @ref #CONFIG_DEVICE_VALIO_ATTRIBUTE config enum token
    */
    uint16_t attribute;

    /** Request type */
    enum dev_valio_request_type_e type:2;

    /** Data buffer, either read or written.  Size is implicit for the
        attribute type. */
    void *data;

    /** Error on request */
    error_t error;
};

STRUCT_COMPOSE(dev_valio_rq_s, base);

/** @see dev_valio_request_t */
#define DEV_VALIO_REQUEST(n) void (n) (                             \
    const struct device_valio_s *accessor,                           \
    struct dev_valio_rq_s *req)

/** @This enqueues an attribute query.
*/
typedef DEV_VALIO_REQUEST(dev_valio_request_t);

DRIVER_CLASS_TYPES(valio,
    dev_valio_request_t *f_request;
);

#define DRIVER_VALIO_METHODS(prefix)                            \
  &(const struct driver_valio_s){                               \
    .class_ = DRIVER_CLASS_VALIO,                               \
    .f_request = prefix ## _request,                            \
  }

inline error_t dev_valio_spin_request(
    const struct device_valio_s *accessor,
    struct dev_valio_rq_s *req)
{
    struct dev_request_status_s status;

    dev_request_spin_init(&req->base, &status);

    DEVICE_OP(accessor, request, req);

    dev_request_spin_wait(&status);

    return req->error;
}


/** @this does the same as @tt dev_valio_wait_read but does not
    use the scheduler.  @this always spins on completion.
*/
config_depend(CONFIG_DEVICE_VALIO)
inline error_t dev_valio_spin_read(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data)
{
    struct dev_valio_rq_s req =
    {
        .type = DEVICE_VALIO_READ,
        .attribute = attribute,
        .data = data,
    };

    return dev_valio_spin_request(accessor, &req);
}

/** @this does the same as @tt dev_valio_wait_write but does not
    use the scheduler.  @this always spins on completion.
*/
config_depend(CONFIG_DEVICE_VALIO)
inline error_t dev_valio_spin_write(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    const void *data)
{
    struct dev_valio_rq_s req =
    {
        .type = DEVICE_VALIO_WRITE,
        .attribute = attribute,
        .data = (void*)data,
    };

    return dev_valio_spin_request(accessor, &req);
}

/** @this does the same as @tt dev_valio_wait_update but does not use
    the scheduler.  @this always spins on completion.
*/
config_depend(CONFIG_DEVICE_VALIO)
inline error_t dev_valio_spin_update(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data)
{
    struct dev_valio_rq_s req =
    {
        .type = DEVICE_VALIO_WAIT_UPDATE,
        .attribute = attribute,
        .data = data,
    };

    return dev_valio_spin_request(accessor, &req);
}

#if defined(CONFIG_MUTEK_SCHEDULER)

inline error_t dev_valio_wait_request(
    const struct device_valio_s *accessor,
    struct dev_valio_rq_s *req)
{
      struct dev_request_status_s status;

      dev_request_sched_init(&req->base, &status);

      DEVICE_OP(accessor, request, req);

      dev_request_sched_wait(&status);

      return req->error;
}

/** @this does a read request on an attribute.

    This places the current context on a wait queue while the request
    is processed.  Read is done without waiting for an update.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_VALIO)
inline error_t dev_valio_wait_read(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data)
{
    struct dev_valio_rq_s req =
    {
        .type = DEVICE_VALIO_READ,
        .attribute = attribute,
        .data = data,
    };

    return dev_valio_wait_request(accessor, &req);
}

/** @this does a write request on an attribute.

    This places the current context on a wait queue while the request
    is processed.  Write is done as soon as possible.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_VALIO)
inline error_t dev_valio_wait_write(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    const void *data)
{
    struct dev_valio_rq_s req =
    {
        .type = DEVICE_VALIO_WRITE,
        .attribute = attribute,
        .data = (void*)data,
    };

    return dev_valio_wait_request(accessor, &req);
}

/** @this waits for the attribute to change.

    This places the current context on a wait queue while the request
    is processed.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_VALIO)
inline error_t dev_valio_wait_update(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data)
{
    struct dev_valio_rq_s req =
    {
        .type = DEVICE_VALIO_WAIT_UPDATE,
        .attribute = attribute,
        .data = data,
    };

    return dev_valio_wait_request(accessor, &req);
}

#endif

#endif

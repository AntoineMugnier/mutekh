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
   @module {Core::Devices support library}
   @short Value IO driver class
   @index {Value IO} {Device classes}
   @csee DRIVER_CLASS_VALIO

   @section{Purpose}
   The @em {Value IO} class gives access to a set of values exposed by
   a device. The class allows getting, setting and registering for
   events on a value. However, the byte size, format, unit and
   encoding of the exposed values are not specified in the class
   itself. This class is especially relevant for sensors and
   actuators. This class should only be used to handle low bandwidth
   transfer of physical values.

   The device may expose several different values which are identified
   using attributes. A device supports a fixed set of attributes. Each
   attribute is defined by its constant identifier, its fixed byte
   size for the value and its set of supported operations.

   Several operations can be performed on an attribute by submitting
   the appropriate @ref dev_valio_rq_s {request} to the driver: @ref
   DEVICE_VALIO_READ {reading}, @ref DEVICE_VALIO_WRITE {writing} or
   @ref DEVICE_VALIO_WAIT_EVENT {waiting for an event} on a
   value. The driver is free to support requests depending on the
   operation and attribute identifier.
   @end section

   @section{Wrong use}
   This class may not be appropriate in some cases:
   @list
     @item Accessing attribute values may be used to control
     actuators, or even set configuration parameters. Yet it must not
     be used to implement a control mechanism where accessing a value
     will be interpreted as a command not related to a physical
     value. For instance, it can not be used to read a firmware
     revision or setup a low power mode of the device. Nonetheless, it
     can be used to set a motor position, set a led intensity, setup a
     value range or handle calibration of the device.

     @item This class is not appropriate for streaming samples at some
     fixed data rate. In particular, the user may query values in a
     loop but the driver is never required to provide the value in
     constant time. Moreover, only instantaneous or cumulative values
     can be reported; no fifo can be used to keep some old unread
     values. In other words, close enough read requests are likely to
     return the same value.

     @item Size of values must be fixed at compile time and can not
     vary at runtime. That's why the request does not include a
     size field.

     @item It must not be used just because no other device class is
     fitting. An new class should be created when the purpose does not
     match the scope of the @em {Value IO} class described here.
   @end list
   @end section

   @section{Defining new attributes}
   Attributes are defined globally at compile time.
   Defining an attribute implies:
   @list
     @item choosing a numerical identifier,
     @item specifying the set of supported operations,
     @item defining a C structure used to store the value,
     @item optionally defining a different C structure used to
       store the result of @ref DEVICE_VALIO_WRITE {write} operations,
     @item optionally defining a C structure used to store the
       parameters for @ref DEVICE_VALIO_READ {read} operations,
     @item optionally defining a C structure used to store the
       condition for the @ref DEVICE_VALIO_WAIT_EVENT {event}
       condition.
   @end list
   @end section

   @section{Generic attributes}

   As explained previously, the device class does not enforce any
   specific format on values, attributes or operation availability.
   It is designed as a generic value accessor API between some devices
   and the user code. However, the device driver and the user code
   need to use a common representation of the values.

   Some libraries may define their own attributes, but some common
   attribute definitions are provided by @tt libdevice.

   Some attribute identifiers are defined along with data encoding for
   some specific usages. Using these attributes allows
   interchangeability of devices of the same type. These are available
   from the following headers:

   @list
   @foreach id name type {header} {+Core::Devices support library::Valio device attributes} {0}
     @group
       @ref @id@ {@name@} : @insert {@id@} decl_short_desc
     @end group
   @end foreach
   @end list

   Any library or application developer who does not see fit in
   existing attribute definition is free to define its own attributes.

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

/** @This specifies the @em {Value IO} operation performed by a request. */
enum dev_valio_request_type_e
{
  /** This queries the value of an attribute. When this operation is
      supported, multiple requests can be queued. The driver must
      sample the value and terminates all read requests as soon as
      possible. Depending on the attribute, an instantaneous or
      cumulative value is reported. Reading a value can not change the
      value for subsequent read requests. For instance, it must not
      clear an accumulator or pop a fifo.

      The @tt data field of the request must always point to a buffer
      large enough to store the retrieved value. Depending on the
      attribute, the data pointer of the request may also be used to
      specify some query related parameters to the driver.

      The request may terminate with the @tt -ENOTSUP error if either
      the attribute or operation is not supported. */
  DEVICE_VALIO_READ,

  /** This changes the value of an attribute. When this operation is
      supported, multiple requests can be queued and will be processed
      in order. The driver must take action to update the value and
      must not terminate the request until the change is effective.

      The @tt data field of the request must always point to a buffer
      containing a properly initialized value. Depending on the
      attribute, the buffer may also be used to return a result. This
      can be the previous value, a feedback value or the actual value
      reached by the actuator for instance.

      It is implementation defined if @ref DEVICE_VALIO_READ and @ref
      DEVICE_VALIO_WRITE requests are pushed on the same queue.

      The request may terminate with the @tt -ENOTSUP error if either
      the attribute or operation is not supported. The driver may
      report the @tt -ERANGE error if the value is out of range. */
  DEVICE_VALIO_WRITE,

  /** This waits for the value of an attribute to change or satisfy a
      given condition. When this operation is supported, the driver
      may optionally support submission of more than one such
      request. When multiple requests are supported, the order is not
      relevant and any request with a satisfied condition must
      terminate immediately. This operation never interferes with read
      and write requests.

      When the @ref DEVICE_VALIO_READ operation is relevant for the
      attribute, the driver stores the new value into the buffer
      specified in @tt data field of the request. In this case, it
      must point to a buffer large enough to store the retrieved
      value.

      Depending on the attribute, the data pointer of the request may
      also be used to specify the condition. This is not relevant when
      the attribute rather relies on a change notification.

      The request may terminate with the @tt -ENOTSUP error if either
      the attribute or operation is not supported. The driver may
      report the @tt -ERANGE error if the condition is out of
      range. The driver may report @tt -EBUSY if it can not handle
      more such requests.

      This kind of request can be canceled by calling the @ref
      dev_valio_cancel_t function of the driver. */
  DEVICE_VALIO_WAIT_EVENT,
};

/** @This is the @em {Value IO} class request structure.
    @see dev_valio_request_t */
struct dev_valio_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

    /** Attribute identifier */
    uint16_t attribute;

    /** Operations */
    enum dev_valio_request_type_e BITFIELD(type,2);

    /** Data buffer used to store the value. The byte size of the
        buffer is fixed for a given attribute identifier. */
    void *data;
};

DEV_REQUEST_INHERIT(valio); DEV_REQUEST_QUEUE_OPS(valio);

/** @see dev_valio_request_t */
#define DEV_VALIO_REQUEST(n) void (n) (                             \
    const struct device_valio_s *accessor,                           \
    struct dev_valio_rq_s *rq)

/** @This enqueues a request.

   The kroutine of the request may be executed from within this
   function. Please read @xref {Nested device request completion}.

   @xsee {Purpose} @see dev_valio_request_type_e
*/
typedef DEV_VALIO_REQUEST(dev_valio_request_t);

/** @see dev_valio_request_t */
#define DEV_VALIO_CANCEL(n) error_t (n) (                             \
    const struct device_valio_s *accessor,                           \
    struct dev_valio_rq_s *rq)

/** This function is able to cancels a @ref DEVICE_VALIO_WAIT_EVENT
    request which have previously been passed to the @ref
    dev_valio_request_t function.

    The function returns 0 if the request has been canceled or @tt
    -EBUSY if the request has already ended or will terminate very
    soon. The request kroutine is not executed when this function
    returns 0.

    The content of the @tt data buffer is undefined when a request is
    canceled. */
typedef DEV_VALIO_CANCEL(dev_valio_cancel_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_VALIO, valio,
    dev_valio_request_t *f_request;
    dev_valio_cancel_t *f_cancel;
);

/** @see driver_valio_s */
#define DRIVER_VALIO_METHODS(prefix)                            \
  ((const struct driver_class_s*)&(const struct driver_valio_s){        \
    .class_ = DRIVER_CLASS_VALIO,                               \
    .f_request = prefix ## _request,                            \
    .f_cancel = prefix ## _cancel,                            \
  })

DEV_REQUEST_WAIT_FUNC(valio);

/** @This perform an operation and stop the
    scheduler context during the request.
    @csee dev_valio_wait_rq */
config_depend_and2_inline(CONFIG_DEVICE_VALIO, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_valio_wait_op(
    enum dev_valio_request_type_e type,
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data),
{
  struct dev_valio_rq_s rq;
  rq.type = type;
  rq.attribute = attribute;
  rq.data = data;
  return dev_valio_wait_rq(accessor, &rq);
});

#endif

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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2018
*/

/**
  @file
  @module {Core::Devices support library}
  @short I2C slave controller driver API
  @index {I2C slave controller} {Device classes}
  @csee DRIVER_CLASS_I2C_SLAVE

  This class handles an I2C slave device.

  @section {I2C slave driver API}

  Times where an I2C device must take action are:
  @list
  @item Start, when slave is selected by its address;
  @item Receive transfers, when slave receives data, slave acknowledges
        bytes received;
  @item Transmit transfers, when slave send data, master acknowledges
        bytes received.
  @end list

  I2C slave class provides three request types to handle these situations:
  @list
  @item Slave selection request expects a matching address
        selection. It triggers after successful (i.e. i2c START
        condition followed by matching slave address byte,
        acknowledged).

  @item Transmit request, sends data buffer to master (i.e. matches a
        read transaction). Request terminates on a STOP condition, a
        START condition or complete transfer of the buffer.

  @item Receive request, receives a data buffer from master (i.e. matches
        a write transaction). Request terminates on a STOP condition,
        a START condition or complete transfer of the buffer.
  @end list

  STOP condition is implicitly signalled through cancellation of
  pending Transmit and Receive requests.  This is because master has
  no requirement for issuing a STOP condition on a bus (STOP is a
  multi-master event for signalling release of the bus, not a
  slave-related event).

  RESTART condition is a START not preceded by a STOP.  For slave
  device user code to handle this case correctly, driver should accept
  a pending Slave selection request at the same time Transmit and
  Receive requests are served.

  After slave address is matched and slave device is active on the
  bus, device driver should clock-stretch the bus while waiting for
  subsequent Receive or Transmit requests to be pushed by calling
  code.

  For hardware able to implement it, drivers may accept
  @list
  @item Slave selection request with an address mask allowing for
        spanning the slave device on more than one address.
  @item multiple Slave selection request with different address. The
        matching one will be signalled. The others will be kept
        enqueued.  Only the calling code that is selected has ability
        to push Transfer requests.
  @end list

  @end section

  @section {Example use cases}

  @section {I2C EEPROM}

  An I2C EEPROM with 2 address bytes handles two type of transactions:

  @list
  @item Writes, where device receives 2 address bytes followed by an
        arbitrary data buffer, in one write transaction;
  @item Reads, where device receives a write transaction of 2 address
        bytes followed by a read transaction of an arbitrary amomunt
        of bytes. Between these two transactions, a STOP condition may
        or may not be issued.
  @end list

  This can be implemented the following way:
  @list
  @item Code pushes a Slave selection request;
  @item Request is satisfied with a write transaction signalled;
  @item Code pushes 3 requests:
    @list
    @item the Slave selection request again,
    @item a Data Receive request with a 2-byte buffer,
    @item a Data Receive request with a 128-byte buffer,
    @end list
  @item If master was preparing a write transaction:
    @list
    @item first Data Receive request will be satisfied after second byte of data,
    @item second Data Receive request will be satisfied after a STOP, a START, or full buffer.
    @item code would be able to double-buffer with two requests in order to handle infinite data streams in an optimal manner.
    @end list
  @item If master was preparing a read transaction:
    @list
    @item first Data Receive request will be satisfied after second byte of data,
    @item second Data Receive request will be terminated because the master issues a START condition,
    @item Slave selection request will be satisfied with a read transaction signalled,
    @item Code pushes Data Transmit request with relevant data.
    @end list
  @end list

  @end section

  @end section

*/

#ifndef __DEVICE_I2C_SLAVE_H__
#define __DEVICE_I2C_SLAVE_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>
#include <mutek/kroutine.h>
#include <gct_platform.h>
#include <gct/container_clist.h>

struct device_s;
struct driver_s;
struct device_i2c_slave_s;
struct driver_i2c_slave_s;
struct dev_i2c_slave_rq_s;

/** @This specifies the operation performed by an I2C @ref dev_i2c_slave_rq_s. */
enum dev_i2c_slave_op_e
{
  /** This waits for matching slave address to be selected by master.
      Only possibility for this request type is to complete
      successfully. */
  DEV_I2C_SLAVE_SELECTION,

  /** This sends a data buffer to master. */
  DEV_I2C_SLAVE_TRANSMIT,

  /** This receives a data buffer from master. */
  DEV_I2C_SLAVE_RECEIVE,
};

struct dev_i2c_slave_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  enum dev_i2c_slave_op_e BITFIELD(type, 2);

  union {
    struct {
      uint8_t *data;
      size_t size;

      bool_t end_ack;
    } transfer;

    struct {
      /** Use 7 lower bits. If a mask other than 0x7f is set, this
          field is updated to actual received address on completion.
          Masked-out bits (if any) may not be reset between calls, it
          is up to the driver to ignore them. */
      uint8_t saddr;

      /** Address mask, in case device spans on more than one slave
          address. Mask other than 0x7f may not be supported by all
          drivers. */
      uint8_t saddr_mask;

      /** Operation type */
      uint8_t read;
    } selection;
  };
  
};

DEV_REQUEST_INHERIT(i2c_slave); DEV_REQUEST_QUEUE_OPS(i2c_slave);

/** @see dev_i2c_slave_request_t */
#define DEV_I2C_SLAVE_REQUEST(n) \
  void (n) (const struct device_i2c_slave_s *accessor, struct dev_i2c_slave_rq_s *rq)
/**
  @This enqueues an I2C slave request.


  Possible error codes:
  @list
  @item @tt -ECANCELED Data transfer request was not served because a
        STOP or START conditon happened.
  @item @tt -EBUSY Another Slave selection request is already pending
        for this address, and multiple addresses are supported.
  @item @tt -ENOTSUP Another Slave selection request is already pending
        and mulitple addresses are not supported.
  @item @tt -EIO Master NACKed a data byte on a Transmit request.
  @item @tt -EINVAL A Transfer request may not be posted now.
  @end list

  All fields of the transfer object except @tt pvdata and @tt error
  must be properly initialized before calling this function.

  A transfer with a null @tt size has an undefined behavior.

  Pushing a Transfer request while no address selection request
  matched has a partially defined behavior. It may silently produce
  undefined results.

  The @ref kroutine_exec function will be called when the transfer
  ends. The kroutine of the request may be executed from within this
  function.  Please read @xref {Nested device request completion}.
*/
typedef DEV_I2C_SLAVE_REQUEST(dev_i2c_slave_request_t);

DRIVER_CTX_CLASS_TYPES(DRIVER_CLASS_I2C_SLAVE, i2c_slave,
    dev_i2c_slave_request_t *f_request;
);

/** @see driver_i2c_slave_s */
# define DRIVER_I2C_SLAVE_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_i2c_slave_s){    \
    .class_ = DRIVER_CLASS_I2C_SLAVE,                                   \
    .f_request = prefix ## _request,                                    \
  })

#endif

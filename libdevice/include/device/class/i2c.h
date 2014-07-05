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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

/**
 * @file
 * @module{Devices support library}
 * @short I2c bus driver API
 */

#ifndef __DEVICE_I2C_H__
#define __DEVICE_I2C_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <mutek/kroutine.h>

#if defined(CONFIG_DEVICE_I2C_REQUEST)
# include <mutek/bytecode.h>
# include <hexo/gpct_platform_hexo.h>
# include <gpct/cont_clist.h>
#endif


struct device_s;
struct driver_s;
struct device_i2c_ctrl_s;
struct driver_i2c_ctrl_s;
struct dev_i2c_ctrl_config_s;
struct dev_i2c_ctrl_transfer_s;
struct dev_i2c_ctrl_request_s;
struct dev_i2c_ctrl_sched_s;

struct device_i2c_dev_s;
struct driver_i2c_dev_s;


/***************************************** config */

enum dev_i2c_speed_e
{
  /** I2C standard mode (100kHz). */
  DEV_I2C_SPEED_STD,

  /** I2C fast mode (400kHz). */
  DEV_I2C_SPEED_FAST,

  /** I2C ultra-fast mode (1MHz). */
  DEV_I2C_SPEED_HIGH,
};

/** @This structure contains the information used to configure the I2C
    controller. **/
struct dev_i2c_ctrl_config_s
{
  /** bit rate */
  enum dev_i2c_speed_e bit_rate;
};

/** @see devi2c_ctrl_config_t */
#define DEVI2C_CTRL_CONFIG(n) error_t (n) ( \
  const struct device_i2c_ctrl_s *i2cdev,   \
  struct dev_i2c_ctrl_config_s   *cfg)      \
/**/

/**
   @This changes the configuration of the controller. If the controller does
   not support requested configuration, this function return @tt -ENOTSUP.

   @This function returns @tt -EBUSY if a transfer is currently being
   processed.
*/
typedef DEVI2C_CTRL_CONFIG(devi2c_ctrl_config_t);


/***************************************** transfer */

enum dev_i2c_ctrl_transfer_op_e
{
  DEV_I2C_OP_NONE       = 0,
  DEV_I2C_OP_START      = 1,
  DEV_I2C_OP_STOP       = 2,
  DEV_I2C_OP_START_STOP = DEV_I2C_OP_START | DEV_I2C_OP_STOP,
};

enum dev_i2c_ctrl_transfer_dir_e
{
  DEV_I2C_TR_WRITE,
  DEV_I2C_TR_READ
};

struct dev_i2c_ctrl_transfer_s
{
  /** The @ref kroutine_exec function is called on this kroutine when a
      transfer ends. When this happens, either the @tt count field of the
      transfer is zero or the @tt err field is set. */
  struct kroutine_s                 kr;

  /** Enable start/stop phase */
  enum dev_i2c_ctrl_transfer_op_e   op:2;

  /** Direction of the I2C transfer. */
  enum dev_i2c_ctrl_transfer_dir_e  dir:1;

  /** Address of the I2C slave device. */
  uint8_t                           saddr;

  /** Number of bytes to transfer. This field will be updated during the
      transfer. If 0 no data phase*/
  size_t                            count;

  /** Pointer to I2C buffer data */
  uint8_t                           *data;

  /** Callback private data. */
  void                              *pvdata;

  /** Transfer completion error. */
  error_t                           error;

  /** Associated I2C controller device. */
  struct device_i2c_ctrl_s          *i2cdev;

  /** This flag indicates the transfer is ended. */
  bool_t                            ended:1;
};

/** @see devi2c_ctrl_transfer_t */
#define DEVI2C_CTRL_TRANSFER(n) void (n) ( \
  const struct device_i2c_ctrl_s *i2cdev,  \
  struct dev_i2c_ctrl_transfer_s *tr       \
)                                          \
/**/

/** @This starts an I2C transfer. A single I2C data transfer can be started at
    the same time. This is the low level transfer function of the I2C device
    class.

    All fields of the transfer object except @tt pvdata, @tt err and @i2cdev
    must be properly initialized before calling this function. The @tt count
    field may be zero. The transfer will fail with @tt -EBUSY if an another
    transfer is currently being processed.

    The @ref kroutine_exec function will be called on @tt tr->kr when the
    transfer ends. This can happen before this function returns. It is ok to
    start a new transfer from the kroutine. This @tt tr->err value indicates
    the error status of the transfer.
*/
typedef DEVI2C_CTRL_TRANSFER(devi2c_ctrl_transfer_t);


/***************************************** queue getter */

#define DEVI2C_CTRL_SCHED(n) struct dev_i2c_ctrl_sched_s * (n) ( \
  struct device_i2c_ctrl_s *i2cdev                               \
)                                                                \
/**/

/** @This retuens the I2C request queue allocated in the I2C controller
    device.
 */
typedef DEVI2C_CTRL_SCHED(devi2c_ctrl_sched_t);


/***************************************** device class */

DRIVER_CLASS_TYPES(i2c_ctrl,
  devi2c_ctrl_config_t      *f_config;
  devi2c_ctrl_transfer_t    *f_transfer;
#if defined(CONFIG_DEVICE_I2C_REQUEST)
  devi2c_ctrl_sched_t       *f_sched;
#endif
);


#if defined(CONFIG_DEVICE_I2C_REQUEST)

/***************************************** device request */

struct dev_i2c_dev_request_s
{
  /** Routine called when the request to device is completed. */
  struct kroutine_s                kr;

  /** Transfer operation. */
  enum dev_i2c_ctrl_transfer_op_e  op;

  /** Transfer direction. */
  enum dev_i2c_ctrl_transfer_dir_e dir;

  /** Data buffer. */
  uint8_t                          *data;

  /** Data size. */
  size_t                           size;

  /** Routine private data. */
  void                             *pvdata;

  /** Error of request if any. */
  error_t                          error;
};

#define DEVI2C_DEV_REQUEST(n) error_t (n) ( \
  struct device_i2c_dev_s      *sdev,       \
  struct dev_i2c_dev_request_s *devrq       \
)                                           \
/**/

/** @This send a request to a I2C slave device through the I2C controlled,
    which the slave is attached to. */
typedef DEVI2C_DEV_REQUEST(devi2c_dev_request_t);


/***************************************** slave device class */

DRIVER_CLASS_TYPES(i2c_dev,
  devi2c_dev_request_t *f_request;
);


/***************************************** request */

enum dev_i2c_ctrl_request_type_e
{
  DEV_I2C_REQ_VM,
  DEV_I2C_REQ_INFO,
};

/** @This structure describes actions to perform on a I2C slave device. */
struct dev_i2c_ctrl_request_s
{
  /** The @ref kroutine_exec function is called on this kroutine when the
      processing is over. */
  struct kroutine_s                    kr;

  /** Queue entry that is used by the driver to enqueue requests. */
  CONTAINER_ENTRY_TYPE(CLIST)          queue_entry;

  union {
    /** The bytecode vm if request is used with bytecode. */
    struct bc_context_s                vm;

    /** Request info if request is called directely. */
    struct {
      uint_fast8_t                     saddr;
      struct dev_i2c_dev_request_s     *devrq;
      bool_t                           done:1;
    }                                  info;
  } u;

  /** This indicates the type of request. */
  enum dev_i2c_ctrl_request_type_e     type;

  /** Request associated configuration. */
  struct dev_i2c_ctrl_config_s         config;

  /** Request error status. */
  error_t                              error;

  /** Accessor to the I2C controller. */
  struct device_i2c_ctrl_s             i2cdev;

  /** Parent scheduler of this request. */
  struct dev_i2c_ctrl_sched_s          *sched;

  /** Callback private data. */
  void                                 *pvdata;

  /** This flag indicates that the request has not ended yet. */
  bool_t                               enqueued:1;

  /** This flag indicates that the request has a high priority. */
  bool_t                               priority:1;
};

CONTAINER_TYPE(dev_i2c_ctrl_queue, CLIST, struct dev_i2c_ctrl_request_s, queue_entry);
CONTAINER_FUNC(dev_i2c_ctrl_queue, CLIST, static inline, dev_i2c_ctrl_queue);

/** @This structure defines a scheduler that is associated with a I2C
    controller. */
struct dev_i2c_ctrl_sched_s
{
  /** The transfer associated with the scheduler. */
  struct dev_i2c_ctrl_transfer_s transfer;

  /** This keeps track of the last used configuration. */
  struct dev_i2c_ctrl_config_s   *config;

  /** This keeps track of the current request. */
  struct dev_i2c_ctrl_request_s  *curr_rq;

  /** The scheduler queue. */
  dev_i2c_ctrl_queue_root_t      queue;

  /** Lock on the scheduler. */
  lock_irq_t                     lock;

  /** This flag indicates the scheduler is running. */
  bool_t                         running;
};


/** @This helper function initializes an I2C request queue structure for use
    in a I2C controller device driver. It is usually called from the
    controller device initialization function so as to setup a queue stored
    in the driver private data.
 */
error_t dev_i2c_ctrl_sched_init(struct device_s             *dev,
                                struct dev_i2c_ctrl_sched_s *sched);

/** This helper function releases the queue associated with the I2C device
    controller.
 */
void dev_i2c_ctrl_sched_destroy(struct dev_i2c_ctrl_sched_s *sched);


/** @This helper function initializes a I2C request for use in a I2C slave
    device driver. It is usually called from the slave driver initialization
    function so as to setup the request stored in the driver private data.

    The @ref dev_i2c_ctrl_request::i2cdev accessor is initialized using the
    device pointed by the @tt{'i2c'} device resource entry of the slave.
 */
error_t dev_i2c_request_init(struct device_s               *slave,
                             struct dev_i2c_ctrl_request_s *rq);

/** @This helper function releases the device accessors that were associated
    with the I2C slave request. @see dev_i2c_request_init. */
void dev_i2c_request_destroy(struct dev_i2c_ctrl_request_s *rq);

/** @This helper function schedules an I2C request on the associated slave
    device. */
void dev_i2c_request_sched(struct dev_i2c_ctrl_request_s *rq);

#endif


/***************************************** helpers */

/** Config helper to change the bit rate of the I2C controller.

    @returns 0 if the configuration succeeded or a negative error code.
*/
config_depend(CONFIG_DEVICE_I2C)
error_t dev_i2c_config(struct device_i2c_ctrl_s     *i2cdev,
                       struct dev_i2c_ctrl_config_s *cfg);

/** Synchronous helper read function. This function uses the scheduler
    api to put current context in wait state if no data is currently
    available. This function spins in a loop waiting for read
    operation to complete when scheduler is disabled.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code. If the slave
    device cannot provide data at the moment, the error code is EAGAIN.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_read(struct device_i2c_dev_s         *slave,
                     enum dev_i2c_ctrl_transfer_op_e op,
                     uint8_t                         *data,
                     size_t                          size);

/** Synchronous helper write function. This function uses the scheduler
    api to put current context in wait state if some data is currently
    pending. This function spins in a loop waiting for write
    operation to complete when scheduler is disabled.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code. If the return
    value is positive but less than the given buffer size, the slave
    was not able to handle all the data (i.e. NACK the last sent data).
    If the slave device does not accept writes, the error code is EAGAIN.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_write(struct device_i2c_dev_s         *slave,
                      enum dev_i2c_ctrl_transfer_op_e op,
                      uint8_t const                   *data,
                      size_t                          size);


/***************************************** very low-level helpers */

/** Synchronous helper scan function. This function spins in a loop
    waiting for read operation to complete.

    The slave address is given in @tt saddr argument.

    @returns 0 if the address matches with a slave device or negative
    error code. If the selected address does not correspond to a device
    the error code is EADDRNOTAVAIL.
*/
config_depend(CONFIG_DEVICE_I2C)
error_t dev_i2c_wait_scan(struct device_i2c_ctrl_s *i2cdev,
                          uint_fast16_t            saddr,
                          bool_t                   saddr_10_bits);

/** Synchronous helper scan function. This function spins in a loop
    waiting for read operation to complete.

    The slave address is given in @tt saddr argument.

    @returns 0 if the address matches with a slave device or negative
    error code. If the selected address does not correspond to a device
    the error code is EADDRNOTAVAIL.
*/
config_depend(CONFIG_DEVICE_I2C)
error_t dev_i2c_spin_scan(struct device_i2c_ctrl_s *i2cdev,
                          uint_fast16_t            saddr,
                          bool_t                   saddr_10_bits);

/** Synchronous helper read function. This function uses the scheduler
    api to put current context in wait state if no data is currently
    available. This function spins in a loop waiting for read
    operation to complete when scheduler is disabled.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code. If the slave
    device cannot provide data at the moment, the error code is EAGAIN.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_wait_read(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_op_e   op,
                          uint8_t                           saddr,
                          uint8_t                           *data,
                          size_t                            size);

/** Synchronous helper read function. This function spins in a loop
    waiting for read operation to complete.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code. If the slave
    device cannot provide data at the moment, the error code is EAGAIN.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_spin_read(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_op_e   op,
                          uint8_t                           saddr,
                          uint8_t                           *data,
                          size_t                            size);

/** Synchronous helper write function. This function uses the scheduler
    api to put current context in wait state if some data is currently
    pending. This function spins in a loop waiting for write
    operation to complete when scheduler is disabled.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code. If the return
    value is positive but less than the given buffer size, the slave
    was not able to handle all the data (i.e. NACK the last sent data).
    If the slave device does not accept writes, the error code is EAGAIN.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_wait_write(const struct device_i2c_ctrl_s    *i2cdev,
                           enum dev_i2c_ctrl_transfer_op_e   op,
                           uint8_t                           saddr,
                           const uint8_t                     *data,
                           size_t                            size);

/** Synchronous helper write function. This function spins in a loop
    waiting for write operation to complete.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code. If the return
    value is positive but less than the given buffer size, the slave
    was not able to handle all the data (i.e. NACK the last sent data).
    If the slave device does not accept writes, the error code is EAGAIN.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_spin_write(const struct device_i2c_ctrl_s    *i2cdev,
                           enum dev_i2c_ctrl_transfer_op_e   op,
                           uint8_t                           saddr,
                           const uint8_t                     *data,
                           size_t                            size);


/******************************************* I2C bytecode */

#if defined(CONFIG_DEVICE_I2C_REQUEST)

/*
   @section {I2C request bytecode instructions}
   @code R
    instruction         params        opcode                  format
 -------------------------------------------------------------------

    generic instructions              0--- ---- ---- ----

    brate               v             1100 0000 ---- --vv

    yield                             1101 0000 ---- ----

    read                o,l,a,r       10oo 1lll aaaa rrrr
    write               o,l,a,r       10oo 0lll aaaa rrrr

   @end code
   @end section
*/

/**
   This instruction configures the bit rate of the I2C controller. The value
   must be chosen from DEV_I2C_SPEED_STD, DEV_I2C_SPEED_HIGH or
   DEV_I2C_SPEED_FAST.
 */
#define BC_I2C_BRATE(value)         \
  BC_CUSTOM(0x4000 | (value & 0x3)) \
/**/

/**
   This instruction allows other requests targeting slave on the same
   I2C bus to be processed.
 */
#define BC_I2C_YIELD() \
  BC_CUSTOM(0x5000)    \
/**/

/**
   This instruction reads up to 8 bytes from an I2C device at address @tt
   saddr.  The operation (START, STOP, START AND STOP) is defined by the @tt op
   argument. The destination buffer address is stored in register @tt baddr.
   The total amount of read bytes is defined in the @tt size argument.

   Note: @tt baddr must point to a valid buffer and @tt size must be greater
   than zero.
 */
#define BC_I2C_READ(op, saddr, baddr, size)                              \
  BC_CUSTOM(((op & 0x3) << 12) | (0x1 << 11) | (((size-1) & 0x7) << 8) | \
    ((saddr & 0xf) << 4) | (baddr & 0xf))                                \
/**/

/**
   This instruction writes up to 8 bytes from an I2C device at address @tt
   saddr. The operation (START, STOP, START AND STOP) is defined by the @tt op
   argument. The source buffer address is stored in register @tt baddr.
   The total amount of written bytes is defined in the @tt size argument.

   Note: @tt baddr must point to a valid buffer and @tt size must be greater
   than zero.
 */
#define BC_I2C_WRITE(op, saddr, baddr, size)                             \
  BC_CUSTOM(((op & 0x3) << 12) | (0x0 << 11) | (((size-1) & 0x7) << 8) | \
    ((saddr & 0xf) << 4) | (baddr & 0xf))                                \
/**/

#endif // CONFIG_DEVICE_I2C_REQUEST

#endif


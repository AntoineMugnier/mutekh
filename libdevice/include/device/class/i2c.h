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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009,2014
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Vincent Defilippi <vincentdefilippi@gmail.com> (c) 2016
*/

/**
  @file
  @module {Core::Devices support library}
  @short I2C bus controller driver API
  @index {I2C bus controller} {Device classes}
  @csee DRIVER_CLASS_I2C_CTRL

  This class enables driving I2C bus controllers. I2C controllers may
  be used from the application code but are most often used from
  device drivers of slave I2C devices.

  A generic I2C requests scheduler is provided which allows sharing a
  single I2C bus by multiple slave devices. This has been designed to
  support concurrence of access from multiple slave drivers.

  @section {I2C controller driver API}

  Unlike most driver classes, I2C bus controller drivers are not able
  to queue more than one request at the same time. They provide the
  @ref dev_i2c_ctrl_transfer_t function which starts a single I2C
  transfer on the BUS, if not already busy. A deferred @ref
  kroutine_s is scheduled when the transfer is over.

  Device drivers of I2C slaves must not use this driver low level API
  directly as it does not allow sharing the bus with other
  slaves. The scheduler API described below is built on top of the
  present driver API. It requires the controller driver to store an
  @ref dev_i2c_ctrl_context_s {I2C scheduler context} object in the
  device private data.

  @end section

  @section {I2C request scheduler}

  This API allows scheduling multiple I2C requests which will be
  processed once the bus becomes idle. This allows sharing a I2C bus
  between multiple slave device drivers and the application.

  Two types of requests can be scheduled concurrently:
  transaction requests and bytecode based requests. Both are covered
  in the following sections.

  @section {I2C transaction request}

  An I2C transaction is composed of multiple elementary transfers which
  perform multiple read or write operations atomically on the bus.
  A transaction begins implicitly with a START condition
  and ends with a STOP condition. A RESTART condition is also implicitly
  generated when the type of successive transfers are different. Successive
  transfers with the same type are seen as a single transfer on the bus.

  A transaction may be started on the I2C bus by calling the @ref
  dev_i2c_transaction_start function. A deferred @ref kroutine_s
  is scheduled when the transaction is over.

  When the controller is used from the device driver of a slave device,
  the @ref dev_drv_i2c_transaction_init helper function may be used to
  initialize a request from the appropriate @xref {Device
  resources}{resource entries} of the slave device.

  @csee #CONFIG_DEVICE_I2C_TRANSACTION
  @end section

  @section {I2C bytecode request}

  Driving a I2C slave device often requires many small transactions
  with some control depending on values read from the slave. Due to
  the asynchronous nature of the transaction requests API, this may
  be cumbersome to write a driver which use deferred execution of
  many small functions between transactions. A blocking C API would
  be easier to use but is not option as this would imply allocation
  of a thread stack for each slave device driver.

  Moreover some slave may have some specific requirements, either
  yielding or locking the bus for some time between transfers. Sharing
  the bus between such slaves without relying on specific code
  requires expressing timing and locking constraints to the scheduler
  for each transfer.

  These issues are addressed by providing an I2C specific bytecode
  based on the Mutekh @xref {generic bytecode}. A bytecode request
  comes with a bytecode program containing some synchronous I2C
  transfer instructions as well as time delay and gpio instructions.
  Generic bytecode instructions can be used for control. The bytecode
  can then be used to write some I2C macros operations specific to a
  given slave. A slave device driver may use several different bytecode
  routines to perform its task.

  A bytecode request may be started on the I2C bus by calling the
  @ref dev_i2c_bytecode_start function. The I2C request scheduler is
  designed to handle time delays and switching between scheduled
  bytecode programs as appropriate. A bytecode requests terminates
  when its associated bytecode program terminates. A deferred @ref
  kroutine_s is scheduled when this occurs.

  When the controller is used from a slave device driver, the @ref
  dev_drv_i2c_bytecode_init helper function may be used to
  initialize a request from the appropriate @xref {Device
  resources}{resource entries} of the slave device.

  @csee #CONFIG_DEVICE_I2C_BYTECODE
  @end section

  @end section

  @section {I2C bytecode instructions}

  This section describes I2C specific bytecode instructions.

  @table 3
  @item instruction              @item operands     @item opcode

  @item generic instructions     @item              @item @tt{0--- ---- ---- ----}

  @item i2c_nodelay              @item              @item @tt{1000 0000 0--0 ----}
  @item i2c_delay                @item r            @item @tt{1000 0000 0--1 rrrr}

  @item i2c_wait                 @item              @item @tt{1000 0000 1--0 ----}
  @item i2c_wait_delay           @item r            @item @tt{1000 0000 1--1 rrrr}

  @item i2c_yield                @item              @item @tt{1000 0001 0--0 ----}
  @item i2c_yield_delay          @item r            @item @tt{1000 0001 0--1 rrrr}
  @item i2c_yieldc               @item              @item @tt{1000 0001 1--0 ----}
  @item i2c_yieldc_delay         @item r            @item @tt{1000 0001 1--1 rrrr}

  @item i2c_addr_get             @item r            @item @tt{1000 0111 ---0 rrrr}
  @item i2c_addr_set             @item r            @item @tt{1000 0111 ---1 rrrr}

  @item i2c_gpiomode             @item i, m         @item @tt{1000 100i iiim mmmm}
  @item i2c_gpioget              @item i, r         @item @tt{1000 101i iii- rrrr}
  @item i2c_gpioset              @item i, r         @item @tt{1000 110i iii- rrrr}

  @item i2c_rdm                  @item ra, rl, e    @item @tt{1100 eee1 aaaa llll}
  @item i2c_wrm                  @item ra, rl, e    @item @tt{1100 eee0 aaaa llll}
  @item i2c_rdr                  @item r,  l,  e    @item @tt{1110 eee1 rrrr llll}
  @item i2c_wrr                  @item r,  l,  e    @item @tt{1110 eee0 rrrr llll}

  @item i2c_rdmc                 @item ra, rl, e    @item @tt{1101 eee1 aaaa llll}
  @item i2c_wrmc                 @item ra, rl, e    @item @tt{1101 eee0 aaaa llll}
  @item i2c_rdrc                 @item r,  l,  e    @item @tt{1111 eee1 rrrr llll}
  @item i2c_wrrc                 @item r,  l,  e    @item @tt{1111 eee0 rrrr llll}

  @end table

  @section {i2c_delay}
  This instruction setup a delay starting when the instruction is executed. The
  execution of the bytecode will not be suspended. When a @xref {i2c_yield}
  instruction is encountered, the execution is suspended if the delay has not
  elapsed at that time. The @cref #CONFIG_DEVICE_I2C_BYTECODE_TIMER must be
  defined in order to use this instruction.

  The delay given in the register is expressed timer unit.
  @end section

  @section {i2c_nodelay}
  This instruction reset the current delay so that the @xref {i2c_yield}
  instruction will not suspend the execution of the bytecode if there are no
  other request to process.
  @end section

  @section {i2c_yield}
  This instruction allows other requests targeting slave on the same i2c bus to
  be processed. This cannot be used if a transaction has been started until a
  STOP condition is generated.
  If a @xref {i2c_delay} instruction has been executed previously, the bytecode
  execution will not resume until the delay has elapsed.
  @end section

  @section {i2c_yieldc}
  This works like @xref {i2c_yield} but the delay can be canceled by @ref
  device_i2c_bytecode_wakeup. When the delay is canceled, the next instruction
  is skipped.
  @end section

  @section {i2c_yield_delay}
  This instruction acts as @xref {i2c_delay} followed by @xref {i2c_yield}.
  @end section

  @section {i2c_yieldc_delay}
  This instruction acts as @xref {i2c_delay} followed by @xref {i2c_yieldc}.
  @end section

  @section {i2c_rd* and i2c_wr*}
    @label{i2c_rw_instructions}
  This schedules a transfer on the i2c bus. The third argument is of
  type @ref dev_i2c_bc_completion_e which indicates how this transfer
  ends. In the case of the first transfer of the transaction, a start
  condition is generated on the bus.

  When the transaction is split in multiple transfer instructions, the
  transfer may actually not be performed before the @ref
  DEV_I2C_BC_STOP instruction. In case of a write instruction, the
  memory buffer or registers containing the output data must remain
  valid. In case of a read instruction, the memory buffer or registers
  used to store the input data may not be updated immediately.

  Successive @ref DEV_I2C_BC_CONTINUOUS transfers in different
  directions are forbidden. If a transfer error is reported by the i2c
  controller, the bytecode is terminated and the request ends with an
  error.
  @end section

  @section {i2c_rd*c and i2c_wr*c}
  These conditional instructions are similar to other read and write
  instructions. If a NAK condition is detected on the bus, the next
  instruction is not skipped. When the transfer is successful, the
  next instruction is skipped. These instructions are usually used
  with the @ref DEV_I2C_BC_STOP value as third argument. @ref
  DEV_I2C_BC_CONTINUOUS and @ref DEV_I2C_BC_RESTART are
  forbidden. @ref DEV_I2C_BC_CONTINUOUS_SYNC and @ref
  DEV_I2C_BC_RESTART_SYNC are not supported by all controllers.
  @end section

  @section {i2c_rdm* and i2c_wrm*}
  These instuctions read data to memory and write data from memory. The buffer
  address and buffer byte length are passed via registers.
  @end section

  @section {i2c_rdr* and i2c_wrr*}
  These instuctions respectively read and write the specified amount of bytes
  to virtual registers. The format of data in registers is hardware dependent
  and needs to be converted by using the @tt pack* and @tt unpack* @xref {Generic
  instruction set} {generic instructions}. The transfered bytes are stored in contiguous
  registers, using at most one register for each group of 4 bytes. The index of
  the first register used to store the data and the number of bytes are expected
  as operands.
  @end section

  @section {i2c_gpioset}
  This instruction sets the value of a gpio pin. The @cref
  #CONFIG_DEVICE_I2C_BYTECODE_GPIO token must be defined.
  @end section

  @section {i2c_gpioget}
  This instruction gets the value of a gpio pin.
  @end section

  @section {i2c_gpiomode}
  This instruction mode the value of a gpio pin.
  @end section

  @end section
*/

#ifndef __DEVICE_I2C_H__
#define __DEVICE_I2C_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>
#include <mutek/kroutine.h>

#include <device/class/gpio.h>

#ifdef CONFIG_DEVICE_I2C_BYTECODE
# include <mutek/bytecode.h>
#endif

# include <device/class/timer.h>

#ifdef CONFIG_DEVICE_I2C_REQUEST
# include <gct_platform.h>
# include <gct/container_clist.h>
#endif

struct device_s;
struct driver_s;
struct device_i2c_ctrl_s;
struct driver_i2c_ctrl_s;
struct dev_i2c_ctrl_transfer_s;
struct dev_i2c_ctrl_rq_s;
struct dev_i2c_ctrl_bytecode_rq_s;
struct dev_i2c_ctrl_transaction_rq_s;
struct dev_i2c_ctrl_context_s;
struct bc_descriptor_s;

/*----------------------------------------------------------------------------*/

#define _DEV_I2C_RESET                0
#define _DEV_I2C_READ_OP              (1 << 0)
#define _DEV_I2C_ENDING_MASK          (3 << 1)
#define _DEV_I2C_CONTINUOUS           (1 << 1)
#define _DEV_I2C_STOP                 (2 << 1)
#define _DEV_I2C_RESTART              (3 << 1)
#define _DEV_I2C_SYNC                 (1 << 3)

/** @This specifies the operation performed by an I2C @ref dev_i2c_ctrl_transfer_s. */
enum dev_i2c_op_e
{
  /** This schedules a read transfer on the i2c bus. The actual
      transfer may not be performed before the next stop operation; in
      this case the driver keep a reference to the buffer until
      done. This operation can only be followed by another read
      operation. A continuous transfer and the following transfer is
      seen as a single transfer on the i2c bus. */
  DEV_I2C_READ_CONTINUOUS = (_DEV_I2C_READ_OP | _DEV_I2C_CONTINUOUS),
  /** This is similar to @ref DEV_I2C_READ_CONTINUOUS.  The sync flag
      requires the transfer to be performed before the @ref
      dev_i2c_ctrl_transfer_s terminates. @b{Not all controller may support this}. */
  DEV_I2C_READ_CONTINUOUS_SYNC = (_DEV_I2C_READ_OP | _DEV_I2C_CONTINUOUS | _DEV_I2C_SYNC),
  /** This schedules a write transfer on the i2c bus. The actual
      transfer may not be performed before the next stop operation; in
      this case the driver keep a reference to the buffer until
      done. This operation can only be followed by another write
      operation. A continuous transfer and the following transfer is
      seen as a single transfer on the i2c bus. */
  DEV_I2C_WRITE_CONTINUOUS = _DEV_I2C_CONTINUOUS,
  /** This is similar to @ref DEV_I2C_WRITE_CONTINUOUS.  The sync flag
      requires the transfer to be performed before the @ref
      dev_i2c_ctrl_transfer_s terminates. @b{Not all controller may support this}. */
  DEV_I2C_WRITE_CONTINUOUS_SYNC = (_DEV_I2C_CONTINUOUS | _DEV_I2C_SYNC),

  /** This schedules a read transfer on the i2c bus. The actual
      transfer may not be performed before the next stop operation; in
      this case the driver keep a reference to the buffer until
      done. This generates a restart on the bus at the end of this
      transfer or at the beginning of the next one. */
  DEV_I2C_READ_RESTART = (_DEV_I2C_READ_OP | _DEV_I2C_RESTART),
  /** This is similar to @ref DEV_I2C_READ_RESTART.  The sync flag
      requires the transfer to be performed before the @ref
      dev_i2c_ctrl_transfer_s terminates. @b{Not all controller may support this}. */
  DEV_I2C_READ_RESTART_SYNC = (_DEV_I2C_READ_OP | _DEV_I2C_RESTART | _DEV_I2C_SYNC),
  /** This schedules a write transfer on the i2c bus. The actual
      transfer may not be performed before the next stop operation; in
      this case the driver keep a reference to the buffer until
      done. This generates a restart on the bus at the end of this
      transfer or at the beginning of the next one. */
  DEV_I2C_WRITE_RESTART = _DEV_I2C_RESTART,
  /** This is similar to @ref DEV_I2C_WRITE_RESTART.  The sync flag
      requires the transfer to be performed before the @ref
      dev_i2c_ctrl_transfer_s terminates. @b{Not all controller may support this}. */
  DEV_I2C_WRITE_RESTART_SYNC = (_DEV_I2C_RESTART | _DEV_I2C_SYNC),

  /** This performs a read transfer on the i2c bus and generates a
      stop on the bus. */
  DEV_I2C_READ_STOP = (_DEV_I2C_READ_OP | _DEV_I2C_STOP | _DEV_I2C_SYNC),
  /** This performs a write transfer on the i2c bus and generates a
      stop on the bus. */
  DEV_I2C_WRITE_STOP = (_DEV_I2C_STOP | _DEV_I2C_SYNC),

  /** This reset the state of the controller when in the middle of a
      transaction. The bus become idle and a new transfer can start
      properly. The way the current transfer is terminated on the bus
      is undefined. */
  DEV_I2C_RESET = _DEV_I2C_RESET,
};

/** @This contains the I2C transfer request which may be started by
    calling the @ref dev_i2c_ctrl_transfer_t function of the driver.
    @xsee {I2C controller driver API} */
struct dev_i2c_ctrl_transfer_s
{
    /** The @ref kroutine_exec function is called on this kroutine when a
        transfer ends.
        If an error has occured during the transfer @tt error
        will be set accordingly by the driver. */
    struct kroutine_s kr;

    /** User private data */
    void *pvdata;

    /** Data buffer to transfer (either read or write).
        When read operation, this field will be updated during the transaction.
        When write operation, this field must be initialized by the caller.
        This field may be @tt NULL if the @tt size equals 0. */
    uint8_t *data;

    /** Size of @tt data buffer.
        This field must be initialized by the caller and may be equals to 0. */
    uint16_t size;

    /** Address of the I2C slave device (may be 7 or 10 bits wide).
        If this field is superior to 127, it will be considered as an 10 bits
        wide address. */
    uint16_t saddr;

    /** Type of transfer */
    enum dev_i2c_op_e BITFIELD(type, 4);

    /** Transfer completion error */
    error_t err;
};

/*----------------------------------------------------------------------------*/

/** @see dev_i2c_ctrl_transfer_t */
#define DEV_I2C_CTRL_TRANSFER(n) \
  void (n) (const struct device_i2c_ctrl_s *accessor, struct dev_i2c_ctrl_transfer_s *tr)
/**
  @This starts an I2C transfer. A single I2C transfer can be started at a time.
  This is a low level transfer function of the I2C device class.

  The @ref dev_i2c_ctrl_transfer_s::type field defines the kind of the
  transfer (read or write) and indicates how it must end. The I2C
  controller generates a START condition at the beginning of every
  transaction.

  There are three ways of ending a transfer: continuous, stop and
  restart. An I2c @em transaction is defined as a transfer ending with
  a stop condition optionally preceded by some other continuous or
  restart transfers. Consecutive continuous transfers of different
  types are not supported.

  In the case where the transfer generates an error, the transaction
  is aborted, the bus is released and the internal state of the driver
  is reset. The following error codes may be reported:

  @list
  @item @tt -EBUSY Another transfer is currently being processed.
  @item @tt -EHOSTUNREACH Got a NACK after a slave address.
  @item @tt -EAGAIN Got a NACK after some data byte.
  @item @tt -EIO Unexpected I/O error.
  @item @tt -ENOTSUP The read restart operation is not supported by the controller.
  @end list

  All fields of the transfer object except @tt pvdata and @tt error
  must be properly initialized before calling this function.

  A transfer with a null @tt size has an undefined behavior.

  The @ref kroutine_exec function will be called when the transfer
  ends. The kroutine of the request may be executed from within this
  function.  Please read @xref {Nested device request completion}.
*/
typedef DEV_I2C_CTRL_TRANSFER(dev_i2c_ctrl_transfer_t);

/*----------------------------------------------------------------------------*/

DRIVER_CTX_CLASS_TYPES(DRIVER_CLASS_I2C_CTRL, i2c_ctrl,
    dev_i2c_ctrl_transfer_t *f_transfer;
);

/** @see driver_i2c_ctrl_s */
# define DRIVER_I2C_CTRL_METHODS(prefix)                                \
  ((const struct driver_class_s*)&(const struct driver_i2c_ctrl_s){     \
    .ctx_offset = offsetof(driver_pv_t , i2c_ctrl_ctx),                 \
    .class_ = DRIVER_CLASS_I2C_CTRL,                                    \
    .f_transfer = prefix ## _transfer,                                  \
  })

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_DEVICE_I2C_REQUEST

/** @This is the I2C scheduler request base structure. */
struct dev_i2c_ctrl_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** @internal */
  struct device_i2c_ctrl_s          *ctrl;

  /** Address of the I2C slave device (may be 7 or 10 bits wide). If this field
      is greater than 127, it will be considered as a 10 bits wide address. */
  uint16_t saddr;

  /** @internal This flag indicates that the request has not ended yet. */
  bool_t BITFIELD(enqueued,1);

  /** @internal This flag indicates we have a bytecode request. */
  bool_t BITFIELD(bytecode,1);
};

DEV_REQUEST_INHERIT(i2c_ctrl);

#endif

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_DEVICE_I2C_TRANSACTION

/** @This specifies the type of I2C @ref dev_i2c_ctrl_transaction_rq_s {transaction}. */
enum dev_i2c_ctrl_transaction_op_e
{
    /** Receive data from slave. */
    DEV_I2C_CTRL_TRANSACTION_READ,
    /** Transmit data to slave. */
    DEV_I2C_CTRL_TRANSACTION_WRITE,
};

/** @This describes the transfer data used by
    @ref dev_i2c_ctrl_transaction_rq_s requests. */
struct dev_i2c_ctrl_transaction_data_s {

    /** Data buffer to transfer (either read or write) */
    uint8_t *data;

    /** Byte size of the @tt data buffer */
    uint16_t size;

    /** Transfer type */
    enum dev_i2c_ctrl_transaction_op_e BITFIELD(type,1);
};

/** @This is the @xcref {I2C transaction request} structure. */
struct dev_i2c_ctrl_transaction_rq_s
{
  union {
    struct dev_i2c_ctrl_rq_s base;
    FIELD_USING(struct dev_i2c_ctrl_rq_s, error);
    FIELD_USING(struct dev_i2c_ctrl_rq_s, pvdata);
  };

    /** Array of transfers to perform */
    struct dev_i2c_ctrl_transaction_data_s *transfer;

    /** Number of transfers in the array */
    uint8_t transfer_count;

    /** Index of transfer currently in progress or pending.
        When an error occurs during the transfer, this field equals the index
        of the faultly transfer. At the end of the transaction, when no error
        occured, this field equals to @tt {transfer_count - 1}. */
    uint8_t transfer_index;
};

STRUCT_INHERIT(dev_i2c_ctrl_transaction_rq_s, dev_i2c_ctrl_rq_s, base);

#endif

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_DEVICE_I2C_BYTECODE

/** @This is the @xcref {I2C bytecode request} structure. */
struct dev_i2c_ctrl_bytecode_rq_s
{
  union {
    struct dev_i2c_ctrl_rq_s base;
    FIELD_USING(struct dev_i2c_ctrl_rq_s, error);
    FIELD_USING(struct dev_i2c_ctrl_rq_s, pvdata);
  };

  /** bytecode virtual machine context */
  struct bc_context_s vm;

#ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  /** @internal */
  dev_timer_value_t sleep_before;
#endif

#ifdef CONFIG_DEVICE_I2C_BYTECODE_GPIO
  struct device_gpio_s gpio;
  /** When the @tt gpio device accessor is valid, this table
      give the index of gpio pin to use when a @tt i2c_gpio*
      instruction is encountered. */
  const gpio_id_t     *gpio_map;
  /** @csee gpio_map */
  const gpio_width_t  *gpio_wmap;
#endif

  /** @internal */
  bool_t BITFIELD(wakeup,1);
  /** @internal */
  bool_t BITFIELD(wakeup_able,1);
};

STRUCT_INHERIT(dev_i2c_ctrl_bytecode_rq_s, dev_i2c_ctrl_rq_s, base);

#endif

/*----------------------------------------------------------------------------*/


/** @internal @This is the I2C scheduler context contained in
    private data of the I2C bus controller device. */
struct dev_i2c_ctrl_context_s
{
#ifdef CONFIG_DEVICE_I2C_REQUEST
  /** @internal */
  dev_request_queue_root_t          queue;
  /** @internal */
  struct dev_i2c_ctrl_rq_s     *current;

#ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
  /** @internal */
  struct dev_i2c_ctrl_bytecode_rq_s *timeout;
  /** @internal This device accessor is used to execute the bytecode
      time delay instructions. It may not be valid, in this case any
      delay instruction with a delay greater than zero will make the
      request fail. @see dev_i2c_timer */
  struct device_timer_s timer;
#endif

  union {
      /** used to schedule bytecode resume/execution */
    struct kroutine_s kr;
#ifdef CONFIG_DEVICE_I2C_BYTECODE_TIMER
    struct dev_timer_rq_s timer_rq;
#endif
    struct {
      struct dev_i2c_ctrl_transfer_s  transfer;
#ifdef CONFIG_DEVICE_I2C_BYTECODE
      /** @internal */
      uint16_t                        op;
#endif
    };
  };
#ifdef CONFIG_DEVICE_I2C_BYTECODE
  /** @internal */
  bool_t                            tr_in_progress:1;
  /** @internal */
  enum dev_i2c_op_e                 BITFIELD(last_type, 4);
#endif

  lock_irq_t                        lock;
#endif
};


/*----------------------------------------------------------------------------*/

/** @internal @see dev_drv_i2c_ctrl_context_init */
config_depend(CONFIG_DEVICE_I2C)
error_t dev_drv_i2c_ctrl_context_init_(struct device_s *dev,
                                       struct dev_i2c_ctrl_context_s *q);

/** This helper function initializes a I2C request context struct for
    use in a I2C controller device driver. It is usually called from
    the controller driver initialization function to initialize a
    context stored in the driver private context.

    The @ref dev_i2c_ctrl_context_s::timer accessor is initialized using
    the device pointed to by the @tt{'timer'} device resource
    entry of the controller, if available.
*/
config_depend_alwaysinline(CONFIG_DEVICE_I2C,
error_t dev_drv_i2c_ctrl_context_init(struct device_s *dev,
                                      struct dev_i2c_ctrl_context_s *q),
{
#ifdef CONFIG_DEVICE_I2C_REQUEST
  return dev_drv_i2c_ctrl_context_init_(dev, q);
#else
  return 0;
#endif
});

/** @internal @see dev_drv_i2c_ctrl_context_cleanup */
config_depend(CONFIG_DEVICE_I2C)
void dev_drv_i2c_ctrl_context_cleanup_(struct dev_i2c_ctrl_context_s *q);

/** This helper function release the device accessor associated with
    the I2C request context. @see dev_drv_i2c_ctrl_context_init */
config_depend_alwaysinline(CONFIG_DEVICE_I2C,
void dev_drv_i2c_ctrl_context_cleanup(struct dev_i2c_ctrl_context_s *q),
{
#ifdef CONFIG_DEVICE_I2C_REQUEST
  dev_drv_i2c_ctrl_context_cleanup_(q);
#endif
});

/** @This schedules a I2C single transaction request for
    execution. The kroutine of the request will be called when the
    transaction is over.

    The kroutine of the request may be executed from within this
    function. Please read @xref {Nested device request completion}. */
config_depend(CONFIG_DEVICE_I2C_TRANSACTION)
void dev_i2c_transaction_start(struct device_i2c_ctrl_s *ctrl,
                               struct dev_i2c_ctrl_transaction_rq_s *rq);

/** @This schedules a I2C bytecode request for execution. The kroutine
    of the request will be called when the bytecode terminates.

    This function returns an error only if the bytecode request is
    already running. All other errors are reported through @tt err
    field of request by executing the associated kroutine.

    If the @tt pc parameter is not @tt NULL, the @ref bc_set_pc
    function is called before starting the bytecode, unless already
    running.

    The kroutine of the request may be executed from within this
    function. Please read @xref {Nested device request completion}. */
config_depend(CONFIG_DEVICE_I2C_BYTECODE)
error_t dev_i2c_bytecode_start(struct device_i2c_ctrl_s *ctrl,
                               struct dev_i2c_ctrl_bytecode_rq_s *rq,
                               const void *pc, uint16_t mask, ...);

config_depend(CONFIG_DEVICE_I2C_BYTECODE)
error_t dev_i2c_bytecode_start_va(struct device_i2c_ctrl_s *ctrl,
                               struct dev_i2c_ctrl_bytecode_rq_s *rq,
                               const void *pc, uint16_t mask, va_list ap);

/** @This initializes a I2C bytecode request. */
config_depend_alwaysinline(CONFIG_DEVICE_I2C_BYTECODE,
void dev_i2c_bytecode_init(struct dev_i2c_ctrl_bytecode_rq_s *rq),
{
  memset(rq, 0, sizeof(*rq));
})

/** This helper function initializes a @xref{I2C bytecode request} for
    use in a I2C slave device driver. It is usually called from the
    slave driver initialization function to initialize a request
    stored in the driver private data.

    The pointer to the I2C controller @tt ctrl will be initialized
    according to the @tt i2c device resource entry of the slave.

    In order to use the gpio bytecode instructions, the @ref
    #CONFIG_DEVICE_I2C_BYTECODE_GPIO token must be defined and the @tt
    gpio parameter must be non-NULL. The pointer to a gpio accessor
    will be initialized and can then be used to setup the @tt
    gpio_map and @tt gpio_wmap fields of the request
    before starting the bytecode. The accessor can later be retrieved
    again using @ref dev_i2c_request_gpio.

    In order to use delay related bytecode instructions, the @ref
    #CONFIG_DEVICE_I2C_BYTECODE_TIMER token must be defined and the
    @tt timer parameter must be non-NULL. The pointer to a timer
    accessor will be initialized. The accessor can later be retrieved
    again using @ref dev_i2c_timer.

    @see dev_drv_i2c_bytecode_cleanup */
config_depend(CONFIG_DEVICE_I2C_BYTECODE)
error_t dev_drv_i2c_bytecode_init(struct device_s *dev,
                                  struct dev_i2c_ctrl_bytecode_rq_s *rq,
                                  const struct bc_descriptor_s *desc,
                                  struct device_i2c_ctrl_s *ctrl,
                                  struct device_gpio_s **gpio,
                                  struct device_timer_s **timer);

/** @This initializes a I2C transaction request. */
config_depend_alwaysinline(CONFIG_DEVICE_I2C_TRANSACTION,
void dev_i2c_transaction_init(struct dev_i2c_ctrl_transaction_rq_s *rq),
{
  memset(rq, 0, sizeof(*rq));
})

/** This helper function initializes a @xref{I2C transaction request}
    for use in a I2C slave device driver. It is usually called from
    the slave driver initialization function to initialize a request
    stored in the driver private data.

    The pointer to the I2C controller @tt ctrl will be initialized
    according to the @tt i2c device resource entry of the slave.

    @see dev_drv_i2c_transaction_cleanup */
config_depend(CONFIG_DEVICE_I2C_TRANSACTION)
error_t dev_drv_i2c_transaction_init(struct device_s *dev,
                                     struct dev_i2c_ctrl_transaction_rq_s *rq,
                                     struct device_i2c_ctrl_s *ctrl);

/** This function returns an accessor to the timer associated with the
    i2c controller of the request. */
config_depend_alwaysinline(CONFIG_DEVICE_I2C_BYTECODE_TIMER,
struct device_timer_s * dev_i2c_timer(struct device_i2c_ctrl_s *ctrl),
{
  struct dev_i2c_ctrl_context_s *q = device_i2c_ctrl_context(ctrl);
  return &q->timer;
})

/** This function returns an accessor to the gpio device of the request. */
config_depend_alwaysinline(CONFIG_DEVICE_I2C_BYTECODE_GPIO,
struct device_gpio_s *dev_i2c_request_gpio(struct dev_i2c_ctrl_bytecode_rq_s *rq),
{
  return &rq->gpio;
})
/** This helper function releases the device accessors associated with
    the I2C slave request. @see dev_drv_i2c_bytecode_init */
config_depend(CONFIG_DEVICE_I2C_BYTECODE)
void dev_drv_i2c_bytecode_cleanup(struct device_i2c_ctrl_s *ctrl,
                                  struct dev_i2c_ctrl_bytecode_rq_s *rq);

config_depend(CONFIG_DEVICE_I2C_TRANSACTION)
void dev_drv_i2c_transaction_cleanup(struct device_i2c_ctrl_s *ctrl,
                                     struct dev_i2c_ctrl_transaction_rq_s *rq);

/** This function cancels the delay of the current or next @xref
    {i2c_yieldc} instruction in the bytecode. If this function is
    called before the next cancelable yield instruction, the
    instruction will be skipped and no yield will be performed.

    This is reset when either a delay is canceled or the request is
    restarted. This returns an error if the request is not currently
    running. */
config_depend(CONFIG_DEVICE_I2C_BYTECODE)
error_t device_i2c_bytecode_wakeup(struct device_i2c_ctrl_s *ctrl,
                                   struct dev_i2c_ctrl_bytecode_rq_s *rq);

/** Synchronous i2c wait function. @This uses the scheduler API to
    put the current context in wait state waiting for the given
    transaction request to terminate. */
config_depend_and2(CONFIG_MUTEK_CONTEXT_SCHED, CONFIG_DEVICE_I2C_TRANSACTION)
void dev_i2c_wait_transaction(struct device_i2c_ctrl_s *ctrl,
                              struct dev_i2c_ctrl_transaction_rq_s *rq);

/** Synchronous i2c wait function. @This uses the scheduler API to
    put the current context in wait state waiting for the given
    bytecode request to terminate. */
config_depend_and2(CONFIG_MUTEK_CONTEXT_SCHED, CONFIG_DEVICE_I2C_BYTECODE)
error_t dev_i2c_wait_bytecode(struct device_i2c_ctrl_s *ctrl,
                              struct dev_i2c_ctrl_bytecode_rq_s *rq,
                              const void *pc, uint16_t mask, ...);

/*----------------------------------------------------------------------------*/

config_depend_and2_alwaysinline(CONFIG_DEVICE_I2C, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_i2c_addr(struct device_s *dev, const char *ctrl, uint16_t addr),
{
  struct dev_resource_s *r;

  error_t err =
    device_res_alloc_str(dev, DEV_RES_I2C_ADDR, ctrl, NULL, &r);
  if (err)
    return err;

  r->u.i2c_addr.addr = addr;

  return 0;
})

config_depend_and2_alwaysinline(CONFIG_DEVICE_I2C, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_i2c_bitrate(struct device_s *dev, uint32_t bitrate),
{
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_I2C_BITRATE);
  if (err)
    return err;

  r->u.i2c_bitrate.bitrate = bitrate;

  return 0;
})

#ifdef CONFIG_DEVICE_I2C

/** @This specifies a @ref #DEV_STATIC_RES_DEVCLASS_PARAM @em i2c
    entry. This is used in slave device resources as a link
    to the associated I2C bus controller with the associated slave address. */
#define DEV_STATIC_RES_I2C_ADDR(ctrl_, addr_)   \
  {                                             \
    .type = DEV_RES_I2C_ADDR,                   \
      .u = { .i2c_addr = {                      \
        .ctrl = (ctrl_),                        \
        .addr = (addr_),                        \
      } }                                       \
  }

/** @This specifies a @ref #DEV_STATIC_RES_DEVCLASS_PARAM @em i2c
    entry. This is used to specified the bitrate of the i2c controller. */
#define DEV_STATIC_RES_I2C_BITRATE(bitrate_)    \
  {                                             \
    .type = DEV_RES_I2C_BITRATE,                \
      .u = { .i2c_bitrate = {                   \
        .bitrate = (bitrate_),                  \
      } }                                       \
  }

#else

# define DEV_STATIC_RES_I2C_ADDR(ctrl_, addr_)  \
  {                                             \
    .type = DEV_RES_UNUSED,                     \
  }

# define DEV_STATIC_RES_I2C_BITRATE(bitrate_)   \
  {                                             \
    .type = DEV_RES_UNUSED,                     \
  }

#endif

#endif

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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013

*/

/**
   @file
   @module {Core::Devices support library}
   @short SPI bus controller driver API
   @index {SPI bus controller} {Device classes}
   @csee DRIVER_CLASS_SPI_CTRL

   This class enables driving SPI bus controllers. SPI controllers may
   be used from the application code but are most often used from
   device drivers of slave SPI devices.

   A generic SPI requests scheduler is provided which allows sharing a
   single SPI bus by multiple slave devices. This has been designed to
   support concurrence of access from multiple slave drivers.

   @section {SPI driver API}

   Unlike most driver classes, SPI bus controller drivers are not able
   to queue more than one request at the same time. They provide the
   @ref dev_spi_ctrl_transfer_t function which starts a single SPI
   transfer on the BUS, if not already busy. A deferred @ref
   kroutine_s is scheduled when the transfer is over.

   The @ref dev_spi_ctrl_config_t function allows setting the bus
   speed and modes for the next transfer.

   Device drivers of SPI slaves must not use this driver low level API
   directly as it does not allow sharing the bus with other
   slaves. The scheduler API described below is builtin on top of the
   present driver API. It requires the controller driver to store a
   @ref dev_spi_ctrl_context_s {spi scheduler context} object in the
   device private data for use by libdevice.

   @end section

   @section {SPI request scheduler}

   This API allows scheduling multiple SPI requests which will be
   processed once the bus becomes idle. This allows sharing a SPI bus
   between multiple slave device drivers and the application.

   Two types of requests can be scheduled concurrently: single
   transaction requests and bytecode based requests. Both are covered
   in the following sections.

   @section {SPI transaction request}

   A transaction is a single SPI transfer processed by the controller
   with an optional chip selection during the transfer.  The chip
   select operation may rely either on the bus controller or on a gpio
   controller. The scheduler will take care of driving the chip select
   signal when the transfer actually takes place.

   A transaction may be started on the SPI bus by calling the @ref
   dev_spi_transaction_start function. A deferred @ref kroutine_s
   is scheduled when the transaction is over.

   When the controller is used from a slave device driver, the @ref
   dev_drv_spi_transaction_init helper function may be used to
   initialize a request from the appropriate @xref {Device
   resources}{resource entries} of the slave device.

   @csee #CONFIG_DEVICE_SPI_TRANSACTION
   @end section

   @section {SPI bytecode request}

   Driving a SPI slave device often requires many small transactions
   with some control depending on values read from the slave. Due to
   the asynchronous nature of the transaction requests API, this may
   be cumbersome to write a driver which use deferred execution of
   many small functions between transactions. A blocking C API would
   be easier to use but is not option as this would imply allocation
   of a thread stack for each slave device driver.

   Moreover, the SPI bus interface being a de facto standard, some
   slave may have some specific requirements, either yielding or
   locking the bus for some time between transfers. Sharing the bus
   between such slaves without relying on specific code requires
   expressing timing and locking constraints to the scheduler for each
   transfer.

   These issues are addressed by providing a SPI specific bytecode
   based on the Mutekh @xref {generic bytecode}{}. A bytecode request
   come with a bytecode program containing some synchronous SPI
   transfer instructions as well as time delay, chip selection and
   gpio instructions. Generic bytecode instructions can be used for
   control. The bytecode can then be used to write some SPI macros
   operations specific to a given slave. A slave device driver may use
   several different bytecode routines to perform its task.

   A bytecode request may be started on the SPI bus by calling the
   @ref dev_spi_bytecode_start function. The SPI request scheduler
   designed to handle time delays and switching between scheduled
   bytecode programs as appropriate.  A bytecode requests terminates
   when its associated bytecode program terminates. A deferred @ref
   kroutine_s is scheduled when this occurs.

   When the controller is used from a slave device driver, the @ref
   dev_drv_spi_bytecode_init helper function may be used to
   initialize a request from the appropriate @xref {Device
   resources}{resource entries} of the slave device.

   @csee #CONFIG_DEVICE_SPI_BYTECODE
   @end section

   @end section

   @section {SPI bytecode instructions}

   This section describes SPI specific bytecode instructions.

   @table 3
    @item instruction              @item operands      @item opcode

    @item generic instructions     @item               @item @tt{0--- ---- ---- ----}

    @item spi_nodelay              @item               @item @tt{1000 0011 0000 ----}
    @item spi_deadline             @item r             @item @tt{1000 0011 01-- rrrr}
    @item spi_delay                @item r             @item @tt{1000 0011 10-- rrrr}
    @item spi_timestamp            @item r             @item @tt{1000 0011 1100 rrrr}
    @item spi_elapsed              @item               @item @tt{1000 0011 1110 ----}
    @item spi_elapsed_r            @item r             @item @tt{1000 0011 1111 rrrr}

    @item spi_yield                @item               @item @tt{1000 0000 0000 ----}
    @item spi_yield_delay          @item r             @item @tt{1000 0000 1000 rrrr}
    @item spi_yield_deadline       @item r             @item @tt{1000 0000 0100 rrrr}
    @item spi_yieldc               @item               @item @tt{1000 0000 0001 ----}
    @item spi_yieldc_delay         @item r             @item @tt{1000 0000 1001 rrrr}
    @item spi_yieldc_deadline      @item r             @item @tt{1000 0000 0101 rrrr}
    @item spi_sleep                @item               @item @tt{1000 0000 0010 ----}

    @item spi_wait                 @item               @item @tt{1000 0010 00-- ----}
    @item spi_wait_delay           @item r             @item @tt{1000 0010 10-- rrrr}
    @item spi_wait_deadline        @item r             @item @tt{1000 0010 01-- rrrr}

    @item spi_width                @item w, o          @item @tt{1000 0100 00ow wwww}
    @item spi_brate                @item r             @item @tt{1000 0100 10-- rrrr}

    @item spi_rd                   @item rd, l, e      @item @tt{1e01 llll rrrr ----}
    @item spi_wr                   @item wr, l, e      @item @tt{1e10 llll ---- rrrr}
    @item spi_swp                  @item wr, rd, l, e  @item @tt{1e11 llll rrrr rrrr}

    @item spi_cs                   @item e             @item @tt{1100 00ee 1--- ----}
    @item spi_pad                  @item p, rl, e      @item @tt{1100 00ee 0ppp llll}
    @item spi_rdm                  @item ra, rl, e     @item @tt{1100 01ee aaaa llll}
    @item spi_wrm                  @item ra, rl, e     @item @tt{1100 10ee aaaa llll}
    @item spi_swpm                 @item rar, raw, rl, e @item @tt{1100 11ee aaaa llll}

    @item spi_gpiomode             @item i, mode       @item @tt{1000 100i iiim mmmm}
    @item spi_gpioget              @item i, r          @item @tt{1000 101i iii- rrrr}
    @item spi_gpioset              @item i, r          @item @tt{1000 110i iii- rrrr}
   @end table

   @section {spi_nodelay}
   This instruction reset the current delay so that the
   @xref {spi_yield} instruction will not suspend the execution of the
   bytecode if there are no other request to process.
   @end section

   @section {spi_deadline}
   This instruction setup a deadline deadline using the @ref dev_timer_value_t
   pointed at the address in the provided register. The execution of next
   instructions follows the same semantics as @xref{spi_delay}.

   The deadline is expressed in timer units.

   The @cref #CONFIG_DEVICE_SPI_BYTECODE_TIMER token must be defined in order
   to use this instruction.
   @end section

   @section {spi_delay}
   This instruction setup a delay starting when the instruction is
   executed. The execution of the bytecode will not be suspended. When
   a @xref {spi_yield} or @xref {spi_wait} instruction is
   encountered, the execution is suspended if the delay has not
   elapsed at that time. The @cref #CONFIG_DEVICE_SPI_BYTECODE_TIMER token
   must be defined in order to use this instruction.

   The delay given in the register is expressed timer unit.
   @end section

   @section {spi_timestamp}
   This instruction fetches the current value of the timer associated with the
   bytecode virtual machine. The timer value is copied in the
   @ref dev_timer_value_t pointed at the address in the provided register.

   The @cref #CONFIG_DEVICE_SPI_BYTECODE_TIMER token must be defined in order
   to use this instruction.
   @end section

   @section {spi_elapsed}
   This conditional instruction skips the next instruction if the timer
   deadline setup by @xref {spi_delay} or @xref {spi_deadline} has not been reached.
   @end section

   @section {spi_elapsed_r}
   This conditional instruction skips the next instruction if the timer
   deadline pointed by the specified register has not been reached.
   @end section

   @section {spi_yield}
   This instruction allows other requests targeting slave on the same
   SPI bus to be processed. The chip select must be in released state
   when this instruction is executed.

   If a @xref {spi_delay} instruction has been executed previously,
   the bytecode execution will not resume until the delay has elapsed.
   @end section

   @section {spi_yieldc}
   This works like @xref {spi_yield} but the delay can be canceled by
   @ref dev_spi_bytecode_wakeup. When the delay is canceled, the
   next instruction is skipped.
   @end section

   @section {spi_sleep}
   This instruction allows other requests targeting slave on the same
   SPI bus to be processed. The @ref dev_spi_bytecode_wakeup function
   must be called in order to resume execution of the bytecode.
   @end section

   @section {spi_yield_delay}
   This instruction acts as @xref {spi_delay} followed by @xref {spi_yield}.
   @end section

   @section {spi_yield_deadline}
   This instruction acts as @xref {spi_deadline} followed by @xref {spi_yield}.
   @end section

   @section {spi_yieldc_delay}
   This works like @xref {spi_yield_delay} but the delay can be
   canceled by @ref dev_spi_bytecode_wakeup.
   @end section

   @section {spi_yieldc_deadline}
   This works like @xref {spi_yield_deadline} but the delay can be
   canceled by @ref dev_spi_bytecode_wakeup.
   @end section

   @section {spi_wait}
   This instruction instructs the controller to wait before resuming
   execution of the bytecode. No other request can be serviced on the
   same bus during the delay. Use @xref {spi_yield} instead if access
   of the bus by other devices is allowed.

   A previous @xref {spi_delay} instruction must be used to setup the delay.
   @end section

   @section {spi_wait_delay}
   This instruction acts as @xref {spi_delay} followed by @xref {spi_wait}.
   @end section

   @section {spi_wait_deadline}
   This instruction acts as @xref {spi_deadline} followed by @xref {spi_wait}.
   @end section

   @section {spi_width}
   This instruction set the SPI word width and the bit
   order in the @ref dev_spi_ctrl_config_s object associated to
   the request. The width value is a constant between 1 and 32. The order
   is MSB first when @tt order is 1.
   @end section

   @section {spi_brate}
   This instruction sets the bit transfer rate in the @ref
   dev_spi_ctrl_config_s object associated to the request. The register must
   contain the new bitrate value in bits per second.
   @end section

   @section {spi_swp}
   This instructions transfer up to 16 bytes between some virtual machine
   registers and a SPI slave. The operands are the source register,
   the destination register, the transfer byte length and the chip
   select operation.

   Available chip select operations are:
   @list
     @item @tt CS_START, @tt CS_CONTINUE: see @ref DEV_SPI_CS_SET_NOP
     @item @tt CS_END, @tt CS_PULSE: see @ref DEV_SPI_CS_SET_CLR
   @end list

   The format of data in registers is hardware dependent and needs to
   be converted by using the @tt pack* and @tt unpack* @xref {Generic
   instruction set} {generic instructions}. The transfered bytes are
   stored in contiguous registers, using at most one register for each
   group of 4 bytes. The index of the first register used to store the
   data and the number of bytes are expected as operands.
   @end section

   @section {spi_wr}
   The @tt spi_wr instruction is similar to @xref{spi_swp} but discards
   data read from the slave.
   @end section

   @section {spi_rd}
   The @tt spi_rd instruction is similar to @xref{spi_swp} but send
   padding bytes with the @tt 0xff value.
   @end section

   @section {spi_swpm}
   This instruction reads and writes multiple SPI words using two
   buffers of bytes. The address of the read buffer is given by the
   value of the @tt rar register operand and the address of the write
   buffer is given by the value of the @tt raw register operand. These
   two registers must be contiguous. The amount of SPI words to
   transfer is given by the @tt rl register operand.

   The last operand specifies the chip select operation:
   @list
     @item @tt CS_START, @tt CS_CONTINUE: see @ref DEV_SPI_CS_SET_NOP
     @item @tt CS_END, @tt CS_PULSE: see @ref DEV_SPI_CS_SET_CLR
     @item @tt CS_NOP: see @ref DEV_SPI_CS_NOP_NOP
     @item @tt CS_DESELECT: see @ref DEV_SPI_CS_CLR_NOP
   @end list
   @end section

   @section {spi_wrm}
   The @tt spi_wrm instruction is similar to @xref{spi_swpm} but discards
   data read from the slave.
   @end section

   @section {spi_rdm}
   The @tt spi_rdm instruction is similar to @xref{spi_swpm} but send
   padding bytes with the @tt 0xff value.
   @end section

   @section {spi_pad}
   The @tt spi_pad instruction is similar to @xref{spi_swpm} but
   discards data read from the slave and send padding bytes. Four packed
   bytes are used as padding pattern.
   @end section

   @section {spi_cs}
   The @tt spi_cs instruction performs a zero length transfer,
   only updating the chip select signal as specified.
   @end section

   @section {spi_gpioset}
   This instruction sets the value of a gpio pin. The @cref
   #CONFIG_DEVICE_SPI_BYTECODE_GPIO token must be defined.
   The first operand selects the pin as an index of the @ref
   dev_spi_ctrl_bytecode_rq_s::gpio_map array.
   @end section

   @section {spi_gpioget}
   This instruction gets the value of a gpio pin.
   The first operand selects the pin as an index of the @ref
   dev_spi_ctrl_bytecode_rq_s::gpio_map array.
   @end section

   @section {spi_gpiomode}
   This instruction mode the value of a gpio pin.
   The first operand selects the pin as an index of the @ref
   dev_spi_ctrl_bytecode_rq_s::gpio_map array.
   @end section

   @end section
*/

#ifndef __DEVICE_SPI_H__
#define __DEVICE_SPI_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/request.h>
#include <device/driver.h>
#include <mutek/kroutine.h>
#include <hexo/enum.h>

#include <device/class/gpio.h>

#ifdef CONFIG_DEVICE_SPI_BYTECODE
# include <mutek/bytecode.h>
#endif

#ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
# include <device/class/timer.h>
#endif

#ifdef CONFIG_DEVICE_SPI_REQUEST
# include <gct_platform.h>
# include <gct/container_clist.h>
#endif

struct device_s;
struct driver_s;
struct device_timer_s;
struct device_gpio_s;
struct device_spi_ctrl_s;
struct driver_spi_ctrl_s;
struct dev_spi_ctrl_transfer_s;
struct dev_spi_ctrl_config_s;
struct dev_spi_ctrl_rq_s;
struct dev_spi_ctrl_bytecode_rq_s;
struct dev_spi_ctrl_transaction_rq_s;
struct dev_spi_ctrl_context_s;
struct bc_descriptor_s;

/***************************************** config */

ENUM_DESCRIPTOR(dev_spi_polarity_e, strip:DEV_SPI_, upper);
ENUM_DESCRIPTOR(dev_spi_ckmode_e, strip:DEV_SPI_CK_, upper);
ENUM_DESCRIPTOR(dev_spi_bit_order_e, strip:DEV_SPI_, upper);
ENUM_DESCRIPTOR(dev_spi_cs_op_e, strip:DEV_SPI_CS_, upper);

enum dev_spi_bit_order_e
{
  DEV_SPI_MSB_FIRST,
  DEV_SPI_LSB_FIRST,
};

enum dev_spi_polarity_e
{
  DEV_SPI_ACTIVE_LOW  = 0,
  DEV_SPI_ACTIVE_HIGH = 1,
};

enum dev_spi_ckmode_e
{
  /** CPOL = 0, CPHA = 0 */
  DEV_SPI_CK_MODE_0,
  /** CPOL = 0, CPHA = 1 */
  DEV_SPI_CK_MODE_1,
  /** CPOL = 1, CPHA = 0 */
  DEV_SPI_CK_MODE_2,
  /** CPOL = 1, CPHA = 1 */
  DEV_SPI_CK_MODE_3,
};

/** @This specifies how the chip select line associated to a SPI slave
    is driven on a transfer. @see dev_spi_cs_config_s */
enum dev_spi_cs_op_e
{
  /** Do not update chip select. */
  DEV_SPI_CS_NOP_NOP = 0,
  /** Deselect chip before transfer, do not change state at the end. */
  DEV_SPI_CS_CLR_NOP = 1,
  /** Select chip before transfer, deselect at the end. */
  DEV_SPI_CS_SET_CLR = 2,
  /** Select chip before transfer, do not change state at the end. */
  DEV_SPI_CS_SET_NOP = 3,
};

/** @This specifies the id and configuration of chip select line
    associated to a SPI slave. */
struct dev_spi_cs_config_s
{
  enum dev_spi_polarity_e  polarity:1;
  /** This is either the index of the spi controller chip select
      output or the pin id of the gpio device used as chip select. */
  uint8_t                  id:7;
};

struct dev_spi_ctrl_config_s
{
  bool_t                   BITFIELD(dirty,1);

  enum dev_spi_ckmode_e    BITFIELD(ck_mode,2);

  enum dev_spi_bit_order_e BITFIELD(bit_order,1);
  enum dev_spi_polarity_e  BITFIELD(miso_pol,1);
  enum dev_spi_polarity_e  BITFIELD(mosi_pol,1);
  enum dev_spi_polarity_e  BITFIELD(cs_pol,1);

  /** Width of the data words on the SPI bus in bits */
  uint8_t                  BITFIELD(word_width,6);

  /** This field gives the bitrate in 1024 bits per second. */
  uint16_t                 bit_rate1k;
};

/** @csee dev_spi_ctrl_config_t */
#define DEV_SPI_CTRL_CONFIG(n) error_t (n) (struct device_spi_ctrl_s *accessor, \
                                            const struct dev_spi_ctrl_config_s *cfg)
/**
   @This changes the configuration of the controller. If the
   controller doesn't support the requested configuration, this
   function returns @tt -ENOTSUP.

   This function ignores the @tt cs_pol field of the configuration.

   @This function returns @tt -EBUSY if a transfer is currently being
   processed.
*/
typedef DEV_SPI_CTRL_CONFIG(dev_spi_ctrl_config_t);

/***************************************** transfer */

/** @This describes the data buffers used during both @ref
    dev_spi_ctrl_transfer_s and @ref dev_spi_ctrl_transaction_rq_s
    requests. */
struct dev_spi_ctrl_data_s
{
  /** Number of SPI words to transfer. This field will be updated
      during the transfer. */
  size_t                   count;

  /** Pointer to input buffer data, the data type used to store SPI
      words in memory is given by @ref in_width. This field will be
      updated during the transfer. This field may be @tt NULL. */
  void                     *in;
  /** Pointer to output buffer data, the data type used to load SPI
      words in memory is given by @ref out_width. This field will be
      updated during the transfer. */
  const void               *out;

  /** Width in bytes of the data type used to store a single input SPI
      word. */
  uint_fast8_t             BITFIELD(in_width,3);
  /** Width in bytes of the data type used to load a single output SPI
      word. A value of 0 means @ref uint32_t without increment of the
      output pointer during transfer. */
  uint_fast8_t             BITFIELD(out_width,3);
};

/** @This contains the SPI transfer request which may be started by
    calling the @ref dev_spi_ctrl_transfer_t function of the driver.
    @xsee {SPI driver API} */
struct dev_spi_ctrl_transfer_s
{
  /** The @ref kroutine_exec function is called on this kroutine when
      a transfer ends. When this happens, either the @tt count field of
      the transfer is zero or the @tt err field is set. */
  struct kroutine_s        kr;

  /** Transfer data buffer */
  struct dev_spi_ctrl_data_s data;

  /** User private data */
  void                     *pvdata;

  /** Chip select operation to perform for this transfer. */
  enum dev_spi_cs_op_e     cs_op:2;

  /** Chip select line configuration */
  struct dev_spi_cs_config_s cs_cfg;

  /** Transfer completion error */
  error_t                  err;
};

/** @see dev_spi_ctrl_transfer_t */
#define DEV_SPI_CTRL_TRANSFER(n) void (n) (struct device_spi_ctrl_s *accessor, \
                                          struct dev_spi_ctrl_transfer_s *tr)

/**
   @This starts an SPI transfer. A single spi data transfer can be
   started at the same time. This is the low level transfer function
   of the SPI device class. The @ref dev_spi_transaction_start and
   @ref dev_spi_bytecode_start functions are able to schedule
   requests for several SPI slaves on the same bus.

   All fields of the transfer object except @tt pvdata, @tt err and
   @tt accessor must be properly initialized before calling this
   function. The transfer will fail with @tt -EBUSY if an other
   transfer is currently being processed.

   The @tt count field may be 0 if only the chip select operation
   needs to be performed.

   The @ref kroutine_exec function will be called on @tt tr->kr when
   the transfer ends. This can happen before this function
   returns. The @tt err field indicates the error status of the transfer.

   The kroutine of the transfer may be executed from within this
   function. Please read @xref {Nested device request completion}.
*/
typedef DEV_SPI_CTRL_TRANSFER(dev_spi_ctrl_transfer_t);


/** This helper function performs a SPI transfert as defined in @tt tr
    and waits for end of transfert. */
config_depend_and2(CONFIG_DEVICE_SPI, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_spi_wait_transfer(struct device_spi_ctrl_s *accessor,
                              struct dev_spi_ctrl_transfer_s * tr);

/** Synchronous spi wait function. @This uses the scheduler API to
    put the current context in wait state waiting for the given
    bytecode request to terminate. */
config_depend_and2(CONFIG_MUTEK_CONTEXT_SCHED, CONFIG_DEVICE_SPI_BYTECODE)
error_t dev_spi_wait_bytecode(struct device_spi_ctrl_s *ctrl,
                              struct dev_spi_ctrl_bytecode_rq_s *rq,
                              const void *pc, uint16_t mask, ...);

/** Synchronous spi wait function. @This uses the scheduler API to
    put the current context in wait state waiting for the given
    transaction request to terminate. */
config_depend_and2(CONFIG_MUTEK_CONTEXT_SCHED, CONFIG_DEVICE_SPI_TRANSACTION)
error_t dev_spi_wait_transaction(struct device_spi_ctrl_s *ctrl,
                                 struct dev_spi_ctrl_transaction_rq_s *rq);

/** @see dev_spi_ctrl_cscfg_t */
#define DEV_SPI_CTRL_CSCFG(n) error_t (n) (struct device_spi_ctrl_s *accessor, \
                                           struct dev_spi_cs_config_s *cs_cfg)

/** @This sets the polarity of the specifed chip select line and
    drive the line so that the chip is not selected. */
typedef DEV_SPI_CTRL_CSCFG(dev_spi_ctrl_cscfg_t);

/***************************************** device class */

DRIVER_CTX_CLASS_TYPES(DRIVER_CLASS_SPI_CTRL, spi_ctrl,
		   dev_spi_ctrl_config_t         *f_config;
		   dev_spi_ctrl_transfer_t       *f_transfer;
		   dev_spi_ctrl_cscfg_t          *f_cscfg;
		   );

#define DRIVER_SPI_CTRL_METHODS(prefix)                     \
  ((const struct driver_class_s*)&(const struct driver_spi_ctrl_s){     \
    .ctx_offset = offsetof(driver_pv_t , spi_ctrl_ctx),                 \
    .class_ = DRIVER_CLASS_SPI_CTRL,                          \
    .f_config = prefix ## _config,                            \
    .f_transfer = prefix ## _transfer,                        \
    .f_cscfg = prefix ## _cscfg,                              \
  })

struct dev_spi_ctrl_context_s;
struct dev_spi_ctrl_bytecode_rq_s;

#ifdef CONFIG_DEVICE_SPI_REQUEST

/***************************************** request */

/** @This is the SPI scheduler request base structure. */
struct dev_spi_ctrl_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** @internal */
  struct device_spi_ctrl_s *ctrl;

  /** This configuration is applied during the execution of the request.
      No configuration takes place when @tt NULL. */
  struct dev_spi_ctrl_config_s *config;

#if defined(CONFIG_DEVICE_SPI_BYTECODE_GPIO) || defined(CONFIG_DEVICE_SPI_GPIO_CS)
  /** @internal If this device accessor refers to a gpio device, it
      will be used to drive the chip select pin and aux pins for this
      SPI slave. If it's not valid, the controller chip select
      mechanism will be used if available. @see dev_spi_request_gpio */
  struct device_gpio_s    gpio;
#endif

  /** Chip select configuration of the slave */
  struct dev_spi_cs_config_s cs_cfg;

  /** Reflect current chip select status, used for checking only. */
  bool_t                  BITFIELD(cs_state,1);

#ifdef CONFIG_DEVICE_SPI_GPIO_CS
  /** Use a gpio device to drive the chip select pin of the slave */
  bool_t                  BITFIELD(cs_gpio,1);
#endif

#ifdef CONFIG_DEVICE_SPI_CTRL_CS
  /** Use the controller to drive the chip select pin of the slave */
  bool_t                  BITFIELD(cs_ctrl,1);
#endif

  /** @internal This flag indicates that the request has not ended yet. */
  bool_t                  BITFIELD(enqueued,1);

  /** @internal This flag indicates we have a bytecode request. */
  bool_t                  BITFIELD(bytecode,1);
};

DEV_REQUEST_INHERIT(spi_ctrl);

#endif

#ifdef CONFIG_DEVICE_SPI_TRANSACTION

/** @This is the @xcref {SPI transaction request} structure.

    Fields common to all types of requests are inherited from @ref
    dev_spi_ctrl_rq_s. */
struct dev_spi_ctrl_transaction_rq_s
{
  union {
    struct dev_spi_ctrl_rq_s base;
    FIELD_USING(struct dev_spi_ctrl_rq_s, error);
    FIELD_USING(struct dev_spi_ctrl_rq_s, pvdata);
  };

  /** Transfer data buffer */
  struct dev_spi_ctrl_data_s data;

  enum dev_spi_cs_op_e     cs_op:2;
};

STRUCT_INHERIT(dev_spi_ctrl_transaction_rq_s, dev_spi_ctrl_rq_s, base);

static ALWAYS_INLINE struct dev_spi_ctrl_transaction_rq_s *
dev_spi_ctrl_transaction_rq_from_kr(struct kroutine_s *kr)
{
  return dev_spi_ctrl_transaction_rq_s_cast(dev_spi_ctrl_rq_from_kr(kr));
}

#endif

#ifdef CONFIG_DEVICE_SPI_BYTECODE

/** @This is the @xcref {SPI bytecode request} structure.

    Fields common to all types of requests are inherited from @cref
    dev_spi_ctrl_rq_s. */
struct dev_spi_ctrl_bytecode_rq_s
{
  union {
    struct dev_spi_ctrl_rq_s base;
    FIELD_USING(struct dev_spi_ctrl_rq_s, error);
    FIELD_USING(struct dev_spi_ctrl_rq_s, pvdata);
  };

  /** bytecode virtual machine context */
  struct bc_context_s      vm;

#ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  /** @internal */
  dev_timer_value_t       sleep_before;
#endif

#ifdef CONFIG_DEVICE_SPI_BYTECODE_GPIO
  /** When the @tt base.gpio device accessor is valid, this table
      give the index of gpio pin to use when a @tt spi_gpio*
      instruction is encountered. */
  const gpio_id_t         *gpio_map;
  /** @csee gpio_map */
  const gpio_width_t      *gpio_wmap;
#endif

  /** @internal */
  bool_t                  BITFIELD(wakeup,1);
  /** @internal */
  bool_t                  BITFIELD(wakeup_able,2);
};

STRUCT_INHERIT(dev_spi_ctrl_bytecode_rq_s, dev_spi_ctrl_rq_s, base);

static ALWAYS_INLINE struct dev_spi_ctrl_bytecode_rq_s *
dev_spi_ctrl_bytecode_rq_from_kr(struct kroutine_s *kr)
{
  return dev_spi_ctrl_bytecode_rq_s_cast(dev_spi_ctrl_rq_from_kr(kr));
}

#endif

/** @internal @This is the SPI scheduler queue contained in
    private data of the SPI bus controller device. */
struct dev_spi_ctrl_context_s
{
#ifdef CONFIG_DEVICE_SPI_REQUEST
  dev_request_queue_root_t      queue;
  struct dev_spi_ctrl_rq_s *current;

# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
  struct dev_spi_ctrl_bytecode_rq_s *timeout;
  /** This device accessor is used to execute the bytecode time delay
      instructions. It may not be valid, in this case any delay
      instruction with a delay greater than zero will make the request
      fail. @ref dev_spi_timer */
  struct device_timer_s         timer;
# endif

  union {
    /** used to schedule bytecode resume/execution */
    struct kroutine_s             kr;
# ifdef CONFIG_DEVICE_SPI_BYTECODE_TIMER
    struct dev_timer_rq_s         timer_rq;
# endif
    struct dev_spi_ctrl_transfer_s transfer;
  };

  /** keeps track of the last used configuration. */
  const struct dev_spi_ctrl_config_s *config;

  lock_irq_t                    lock;
#endif
};

/** This helper function initializes a SPI request queue struct for
    use in a SPI controller device driver. It is usually called from
    the controller driver initialization function to initialize a
    queue stored in the driver private context.

    The @ref dev_spi_ctrl_context_s::timer accessor is initialized using
    the device pointed to by the @tt{'timer'} device resource
    entry of the controller, if available.
*/
config_depend(CONFIG_DEVICE_SPI_REQUEST)
error_t dev_spi_context_init(struct device_s *dev, struct dev_spi_ctrl_context_s *q);

/** This helper function release the device accessor associated with
    the SPI request queue. @see dev_spi_context_init */
config_depend(CONFIG_DEVICE_SPI_REQUEST)
void dev_spi_context_cleanup(struct dev_spi_ctrl_context_s *q);

/** @This schedules a SPI single transaction request for
    execution. The kroutine of the request will be called when the
    transaction is over.

    The kroutine of the request may be executed from within this
    function. Please read @xref {Nested device request completion}. */
config_depend(CONFIG_DEVICE_SPI_TRANSACTION)
void dev_spi_transaction_start(struct device_spi_ctrl_s *ctrl,
                               struct dev_spi_ctrl_transaction_rq_s *rq);

/** @This schedules a SPI bytecode request for execution. The kroutine
    of the request will be called when the bytecode terminates.

    This function returns an error only if the bytecode request is
    already running. All other errors are reported through @tt err
    field of request by executing the associated kroutine.

    When not busy, this function may also set the registers of the
    virtual machine before starting execution of the bytecode as if
    the @ref bc_set_pc and @ref bc_set_regs function were used. If the
    @tt pc parameter is @tt NULL, the pc is not changed.

    The kroutine of the request may be executed from within this
    function. Please read @xref {Nested device request completion}. */
config_depend(CONFIG_DEVICE_SPI_BYTECODE)
error_t dev_spi_bytecode_start(struct device_spi_ctrl_s *ctrl,
                               struct dev_spi_ctrl_bytecode_rq_s *rq,
                               const void *pc, uint16_t mask, ...);

config_depend(CONFIG_DEVICE_SPI_BYTECODE)
error_t dev_spi_bytecode_start_va(struct device_spi_ctrl_s *ctrl,
                                  struct dev_spi_ctrl_bytecode_rq_s *rq,
                                  const void *pc, uint16_t mask, va_list ap);

/** @This initializes a SPI bytecode  request. */
config_depend_alwaysinline(CONFIG_DEVICE_SPI_BYTECODE,
void dev_spi_bytecode_init(struct dev_spi_ctrl_bytecode_rq_s *rq),
{
  memset(rq, 0, sizeof(*rq));
})

/** This helper function initializes a @xref{SPI bytecode request} for
    use in a SPI slave device driver. It is usually called from the
    slave driver initialization function to initialize a request
    stored in the driver private data. The @ref bc_init function is
    called with the @tt desc parameter.

    The pointer to the SPI controller @tt ctrl will be initialized
    according to the @tt spi device resource entry of the slave.

    If a @tt{'spi-cs-id'} entry is present in the device tree, the
    request is configured to use the chip select feature of the SPI
    controller. If a @tt{'gpio-cs-id'} and a @tt{'gpio'} entries are
    present, the request is configured to use a gpio pin as chip
    select instead.

    In order to use the gpio bytecode instructions, the @ref
    #CONFIG_DEVICE_SPI_BYTECODE_GPIO token must be defined and the @tt
    gpio parameter must be non-NULL. The pointer to a gpio accessor
    will be initialized and can then be used to setup the @tt
    gpio_map and @tt gpio_wmap fields of the request
    before starting the bytecode. The accessor can later be retrieved
    again using @ref dev_spi_request_gpio.

    When gpios are used either for chip select or from the bytecode,
    the @tt gpio resource entry of the device must point to a valid
    gpio device.

    In order to use delay related bytecode instructions, the @ref
    #CONFIG_DEVICE_SPI_BYTECODE_TIMER token must be defined and the
    @tt timer parameter must be non-NULL. The pointer to a timer
    accessor will be initialized. The accessor can later be retrieved
    again using @ref dev_spi_timer.

    @see dev_drv_spi_bytecode_cleanup */
config_depend(CONFIG_DEVICE_SPI_BYTECODE)
error_t dev_drv_spi_bytecode_init(struct device_s *dev,
                                  struct dev_spi_ctrl_bytecode_rq_s *rq,
                                  const struct bc_descriptor_s *desc,
                                  const struct dev_spi_ctrl_config_s *cfg,
                                  struct device_spi_ctrl_s *ctrl,
                                  struct device_gpio_s **gpio,
                                  struct device_timer_s **timer);

/** @This initializes a SPI transaction request. */
config_depend_alwaysinline(CONFIG_DEVICE_SPI_TRANSACTION,
void dev_spi_transaction_init(struct dev_spi_ctrl_transaction_rq_s *rq),
{
  memset(rq, 0, sizeof(*rq));
  rq->cs_op = DEV_SPI_CS_SET_CLR;
})

/** This helper function initializes a @xref{SPI transaction request}
    for use in a SPI slave device driver. It is usually called from
    the slave driver initialization function to initialize a request
    stored in the driver private data.

    The pointer to the SPI controller @tt ctrl will be initialized
    according to the @tt spi device resource entry of the slave.

    If a @tt{'spi-cs-id'} entry is present in the device tree, the
    request is configured to use the chip select feature of the SPI
    controller. If a @tt{'gpio-cs-id'} and a @tt{'gpio'} entries are
    present, the request is configured to use a gpio pin as chip
    select instead.

    If the @tt gpio parameter is not @tt NULL, a pointer to a gpio
    accessor will be initialized as well.

    @see dev_drv_spi_transaction_cleanup */
config_depend(CONFIG_DEVICE_SPI_TRANSACTION)
error_t dev_drv_spi_transaction_init(struct device_s *dev,
                                     struct dev_spi_ctrl_transaction_rq_s *rq,
                                     const struct dev_spi_ctrl_config_s *cfg,
                                     struct device_spi_ctrl_s *ctrl,
                                     struct device_gpio_s **gpio);

/** This function returns an accessor to the timer associated with the
    spi controller of the request. */
config_depend_alwaysinline(CONFIG_DEVICE_SPI_BYTECODE_TIMER,
struct device_timer_s *
dev_spi_timer(struct device_spi_ctrl_s *ctrl),
{
  struct dev_spi_ctrl_context_s *q = device_spi_ctrl_context(ctrl);
  return &q->timer;
})

/** This function returns an accessor to the gpio device of the request. */
config_depend_alwaysinline(CONFIG_DEVICE_SPI_BYTECODE_GPIO,
struct device_gpio_s *
dev_spi_request_gpio(struct dev_spi_ctrl_rq_s *rq),
{
  return &rq->gpio;
})

/** This helper function releases the device accessors associated with
    the SPI slave request. @see dev_drv_spi_bytecode_init */
config_depend(CONFIG_DEVICE_SPI_BYTECODE)
void dev_drv_spi_bytecode_cleanup(struct device_spi_ctrl_s *ctrl,
                                  struct dev_spi_ctrl_bytecode_rq_s *rq);

config_depend(CONFIG_DEVICE_SPI_TRANSACTION)
void dev_drv_spi_transaction_cleanup(struct device_spi_ctrl_s *ctrl,
                                     struct dev_spi_ctrl_transaction_rq_s *rq);

/** This function cancels the delay of the current or next @xref
    {spi_yieldc} instruction in the bytecode. If this function is
    called before the next cancelable yield instruction, the
    instruction will be skipped and no yield will be performed.

    This is reset when either a delay is canceled or the request is
    restarted. This returns an error if the request is not currently
    running. */
config_depend(CONFIG_DEVICE_SPI_BYTECODE)
error_t dev_spi_bytecode_wakeup(struct device_spi_ctrl_s *ctrl,
                                struct dev_spi_ctrl_bytecode_rq_s *rq);

#ifdef CONFIG_DEVICE_SPI
/** @This specifies a @ref #DEV_STATIC_RES_DEVCLASS_PARAM @em spi
    entry. This is typically used in slave device resources as a link
    to the associated SPI bus controller. */
# define DEV_STATIC_RES_DEV_SPI(path_)                          \
  DEV_STATIC_RES_DEVCLASS_PARAM("spi", path_, DRIVER_CLASS_SPI_CTRL)
#else
# define DEV_STATIC_RES_DEV_SPI(path_)                                  \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

#endif



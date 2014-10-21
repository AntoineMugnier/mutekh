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
 * @file
 * @module{Devices support library}
 * @short SPI controller driver API
 */

#ifndef __DEVICE_SPI_H__
#define __DEVICE_SPI_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <mutek/kroutine.h>

#ifdef CONFIG_DEVICE_SPI_REQUEST
# include <mutek/bytecode.h>
# include <device/class/gpio.h>
# include <device/class/timer.h>
# include <gct_platform.h>
# include <gct/container_clist.h>
#endif

struct device_s;
struct driver_s;
struct device_spi_ctrl_s;
struct driver_spi_ctrl_s;
struct dev_spi_ctrl_transfer_s;
struct dev_spi_ctrl_config_s;
struct dev_spi_ctrl_request_s;
struct dev_spi_ctrl_queue_s;

/**
   @file
*/

/***************************************** config */

enum dev_spi_bit_order_e
{
  DEV_SPI_MSB_FIRST,
  DEV_SPI_LSB_FIRST,
};

enum dev_spi_polarity_e
{
  DEV_SPI_CS_ACTIVE_LOW,
  DEV_SPI_CS_ACTIVE_HIGH,
};

enum dev_spi_ckmode_e
{
  DEV_SPI_CK_LOW_LEADING,
  DEV_SPI_CK_LOW_TRAILING,
  DEV_SPI_CK_HIGH_LEADING,
  DEV_SPI_CK_HIGH_TRAILING,
};

enum dev_spi_cs_policy_e
{
  /** The chip select is asserted during the SPI transfer and will be
      deasserted at the end of the transfer. Some buggy controllers
      are not able to hold the chip select between two words of the
      same transfer. */
  DEV_SPI_CS_TRANSFER  = 0,
  /** The chip select remains asserted. Not all controller support
      asserting the chip select when there is no ongoing transfer. An
      error will be reported in this case. */
  DEV_SPI_CS_ASSERT    = 1,
  /** The chip select is deasserted. Not all controller support
      deasserting the chip select during a transfer. An
      error will be reported in this case. */
  DEV_SPI_CS_DEASSERT  = 2,
  /** The chip select pin is not used/driven. */
  DEV_SPI_CS_RELEASE   = 3,
};

struct dev_spi_ctrl_config_s
{
  enum dev_spi_ckmode_e    ck_mode:2;

  enum dev_spi_bit_order_e bit_order:1;
  enum dev_spi_polarity_e  miso_pol:1;
  enum dev_spi_polarity_e  mosi_pol:1;

  /** This field gives the bitrate in bits per second. */
  uint32_t                 bit_rate;

  /** Width of the data words on the SPI bus in bits */
  uint_fast8_t             word_width;
};

/** @csee devspi_ctrl_config_t */
#define DEVSPI_CTRL_CONFIG(n) error_t (n) (struct device_spi_ctrl_s *scdev, \
                                           struct dev_spi_ctrl_config_s *cfg)
/**
   @This changes the configuration of the controller. If the
   controller doesn't support the requested configuration, this
   function returns @tt -ENOTSUP.

   @This function returns @tt -EBUSY if a transfer is currently being
   processed.
*/
typedef DEVSPI_CTRL_CONFIG(devspi_ctrl_config_t);

/***************************************** select */

#define DEVSPI_CTRL_SELECT(n) error_t (n) (struct device_spi_ctrl_s *scdev, \
                                           enum dev_spi_cs_policy_e pc, \
                                           enum dev_spi_polarity_e pt,  \
                                           uint_fast8_t cs_id)
/**
   @This changes the chip select value.

   This function may return @tt -ENOTSUP depending on hardware capabilities.
*/
typedef DEVSPI_CTRL_SELECT(devspi_ctrl_select_t);


/***************************************** transfer */

struct dev_spi_ctrl_transfer_s
{
  /** The @ref kroutine_exec function is called on this kroutine when
      a transfer ends. When this happens, either the @tt count field of
      the transfer is zero or the @tt err field is set. */
  struct kroutine_s        kr;

  /** Number of SPI words to transfer. If this value is 0, the @ref in
      and @ref out pointers may be @tt NULL and the callback function
      will not be invoked. This field will be updated during the
      transfer. */
  size_t                   count;

  /** Pointer to input buffer data, the data type used to store SPI
      words in memory is given by @ref in_width. This field will be
      updated during the transfer. This field may be @tt NULL. */
  void                     *in;
  /** Pointer to output buffer data, the data type used to load SPI
      words in memory is given by @ref out_width. This field will be
      updated during the transfer. */
  const void               *out;

  /** Callback private data */
  void                     *pvdata;

  /** Transfer completion error */
  error_t                  err;

  /** Associated SPI controller device */
  struct device_spi_ctrl_s *scdev;

  /** Width in bytes of the data type used to store a single input SPI
      word. */
  uint_fast8_t             in_width:3;
  /** Width in bytes of the data type used to load a single output SPI
      word. A value of 0 means @ref uint32_t without increment of the
      output pointer during transfer. */
  uint_fast8_t             out_width:3;
};

/** @see devspi_ctrl_transfer_t */
#define DEVSPI_CTRL_TRANSFER(n) void (n) (struct device_spi_ctrl_s *scdev, \
                                          struct dev_spi_ctrl_transfer_s *tr)

/**
   @This starts an SPI transfer. A single spi data transfer can be
   started at the same time. This is the low level transfer function
   of the SPI device class. The @ref devspi_ctrl_request_t function is
   able to schedule complex requests for several SPI slaves on the
   same bus.

   All fields of the transfer object except @tt pvdata, @tt err and
   @tt scdev must be properly initialized before calling this
   function. The @tt count field can not be 0. The transfer will fail
   with @tt -EBUSY if an other transfer is currently being processed.

   The @ref kroutine_exec function will be called on @tt tr->kr when
   the transfer ends. This can happen before this function
   returns. It's ok to start a new transfer from the kroutine. The @tt
   tr->err value indicates the error status of the transfer.
*/
typedef DEVSPI_CTRL_TRANSFER(devspi_ctrl_transfer_t);


/***************************************** queue getter */

#define DEVSPI_CTRL_QUEUE(n) struct dev_spi_ctrl_queue_s * (n)(struct device_spi_ctrl_s *scdev)

/**
   @This returns SPI request queue allocated in the SPI controller
   device private data.
 */
typedef DEVSPI_CTRL_QUEUE(devspi_ctrl_queue_t);

/***************************************** device class */

DRIVER_CLASS_TYPES(spi_ctrl,
		   devspi_ctrl_config_t         *f_config;
		   devspi_ctrl_select_t         *f_select;
		   devspi_ctrl_transfer_t       *f_transfer;
#ifdef CONFIG_DEVICE_SPI_REQUEST
		   devspi_ctrl_queue_t        *f_queue;
#endif
		   );

#ifdef CONFIG_DEVICE_SPI_REQUEST

/***************************************** request */

#define GCT_CONTAINER_ALGO_dev_spi_ctrl_queue CLIST

/** @This structure describes actions to perform on a SPI slave device. */
struct dev_spi_ctrl_request_s
{
  /** The @ref kroutine_exec function is called on this kroutine when
      the bytecode execution ends. */
  struct kroutine_s        kr;

  /** used by driver to enqueue requests */
  GCT_CONTAINER_ENTRY(dev_spi_ctrl_queue, queue_entry);

  /** bytecode virtual machine context */
  struct bc_context_s      vm;

  struct dev_spi_ctrl_config_s config;

  /** request end callback */
  error_t                  err;

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  dev_timer_value_t       sleep_before;
#endif

  struct device_spi_ctrl_s scdev;
  struct dev_spi_ctrl_queue_s *queue;

  /** Callback private data */
  void                     *pvdata;

  /** If this device accessor refers to a gpio device, it will be used
      to drive the chip select pin and aux pins for this SPI slave. If
      it's not valid, the controller chip select mechanism will be
      used if available. */
  struct device_gpio_s    gpio;

  /** If the @ref gpio device accessor is valid, these tables give the
      index of gpio pin to use when a @tt BC_SPI_GPIO* instruction is
      encountered. If the @ref cs_gpio field is set, the first entry
      of the table is used to drive the chip select signal. */
  const gpio_id_t         *gpio_map;
  const gpio_width_t      *gpio_wmap;

  /** If the @ref cs_ctrl field is set, this value is used by the SPI
      controller to select the chip select output. */
  uint8_t                 cs_id;

  /** Current cs policy */
  enum dev_spi_cs_policy_e cs_policy:2;

  /** Chip select polarity of the slave device */
  enum dev_spi_polarity_e cs_polarity:1;

  /** Use a gpio device to drive the chip select pin of the slave */
  bool_t                  cs_gpio:1;
  /** Use the controller to driver the chip select pin of the slave */
  bool_t                  cs_ctrl:1;

  /** This flag indicates that the request has not ended yet. */
  bool_t                  enqueued:1;

  bool_t                  wakeup:1;
  bool_t                  wakeup_able:1;

  bool_t                  priority:1;
};

GCT_CONTAINER_TYPES(dev_spi_ctrl_queue, struct dev_spi_ctrl_request_s *, queue_entry);
GCT_CONTAINER_FCNS(dev_spi_ctrl_queue, inline, dev_spi_ctrl_queue,
                   init, destroy, remove, push, pushback, pop, isempty);

struct dev_spi_ctrl_queue_s
{
  /** This device accessor is used to execute the delay bytecode
      instructions. It may not be valid, in this case any delay
      instruction with a delay greater than zero will make the request
      fail. */
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  struct device_timer_s         timer;
#endif

  union {
#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
    struct dev_timer_rq_s         timer_rq;
#endif
    struct dev_spi_ctrl_transfer_s transfer;
  };

  /** This keep track of the last used configuration. */
  struct dev_spi_ctrl_config_s *config;

  struct dev_spi_ctrl_request_s *current;
  struct dev_spi_ctrl_request_s *timeout;
  dev_spi_ctrl_queue_root_t     queue;

  lock_irq_t                    lock;
  bool_t                        running;

#ifdef CONFIG_DEVICE_SPI_REQUEST_TIMER
  /** 1us delay shift, computed by @ref dev_timer_shift_sec @multiple */
  int8_t                        delay_shift_a;
  int8_t                        delay_shift_b;
#endif
};

/** This helper function initializes a SPI request queue struct for
    use in a SPI controller device driver. It is usually called from
    the controller driver initialization function to initialize a
    queue stored in the driver private context.

    The @ref dev_spi_ctrl_queue_s::timer accessor is initialized using
    the device pointed to by the @tt{'spi-timer'} device resource
    entry of the controller, if available.
*/
error_t dev_spi_queue_init(struct device_s *dev, struct dev_spi_ctrl_queue_s *q);

/** This helper function release the device accessor associated with
    the SPI request queue. @see dev_spi_queue_init
 */
void dev_spi_queue_cleanup(struct dev_spi_ctrl_queue_s *q);

/**
   SPI controller processing function type.

   When this function is called, the bytecode associated with the
   endpoint is executed. The endpoint callback function will be called
   when the processing is over.

   The controller may process bytecode from multiple endpoints at the
   same time.

   This function will fail and return @tt -EBUSY if an other endpoint
   with the same @ref dev_spi_endpoint_s::ep_id field is currently
   being processed.

   @param scdev pointer to controller device accessor
   @param ep pointer to the SPI endpoint.
*/
void dev_spi_request_start(struct dev_spi_ctrl_request_s *rq);

/** This helper function initializes a SPI request structure for use
    in a SPI slave device driver. It is usually called from the slave
    driver initialization function to initialize a request stored in
    the driver private context.

    The @ref dev_spi_ctrl_request_s::scdev accessor is initialized
    using the device pointed to by the @tt{'spi'} device resource
    entry of the slave.

    If a @tt{'spi-cs-id'} entry is present in the device tree, the request
    is configured to use the chip select feature of the SPI
    controller.  In the other case, the @ref dev_spi_ctrl_request_s::cs_gpio
    field can still be used to drive the chip select using a GPIO pin.
*/
error_t dev_spi_request_init(struct device_s *slave,
                             struct dev_spi_ctrl_request_s *rq);

/** This helper function release the device accessors associated with
    the SPI slave request. @see dev_spi_request_init */
void dev_spi_request_cleanup(struct dev_spi_ctrl_request_s *rq);

/** This function cancels the delay of the current or next @ref
    #BC_SPI_YIELDC instruction in the bytecode. If this function is
    called before the next cancelable yield instruction, the
    instruction will be skipped and no yield will be performed.

    This is reset when either a delay is canceled or the request is
    restarted. This returns an error if the request is not currently
    running.
 */
error_t device_spi_request_wakeup(struct dev_spi_ctrl_request_s *rq);

#endif

/*************************************************************** SPI bytecode */

#ifdef CONFIG_DEVICE_SPI_REQUEST

/*
   @section {SPI request bytecode instructions}
   @code R
    instruction         params        opcode                  format
 -------------------------------------------------------------------

    generic instructions              0--- ---- ---- ----

    delay               r             1000 0011 10-- rrrr
    nodelay                           1000 0011 00-- ----

    yield                             1000 0000 001- ----
    yield_delay         r             1000 0000 101- rrrr
    yieldc                            1000 0000 000- ----
    yieldc_delay        r             1000 0000 100- rrrr

    wait                cs            1000 0010 00cc ----
    wait_delay          r, cs         1000 0010 10cc rrrr

    setcs               cs            1000 0010 01cc ----

    width               w, o          1000 0100 00ow wwww
    brate               r             1000 0100 10-- rrrr

    swp                 r, r          1000 1000 rrrr rrrr
    swpl                r, r, l       1000 1lll rrrr rrrr

    pad                 r             1001 0000 ---- rrrr

    rdm[8,16,32]        ra, r         1001 01ss aaaa rrrr
    wrm[8,16,32]        ra, r         1001 10ss aaaa rrrr
    swpm[8,16,32]       ra, r         1001 11ss aaaa rrrr

    gpioset             i, r          1010 iiii iiii rrrr
    gpioget             i, r          1011 iiii iiii rrrr
    gpiomode            i, r          1100 iiii iiii mmmm

   @end code
   @end section
*/

/**
   This instruction setup a delay starting when the instruction is
   executed. The execution of the bytecode will not be suspended. When
   a @ref #BC_SPI_YIELD or @ref #BC_SPI_WAIT instruction is
   encountered, the execution is suspended if the delay has not
   elapsed at that time. The @ref #CONFIG_DEVICE_SPI_REQUEST_TIMER
   must be defined in order to use this instruction.

   The delay given in the register is expressed microsecond unit.
 */
#define BC_SPI_DELAY(r) BC_CUSTOM(0x0380 | (r & 0xf))

/**
   This instruction reset the current delay so that the @ref
   #BC_SPI_YIELD instruction will not suspend the execution of the
   bytecode if there are no other request to process.
 */
#define BC_SPI_NODELAY() BC_CUSTOM(0x0300)

/**
   This instruction allows other requests targeting slave on the same
   SPI bus to be processed. The chip select will be deasserted in order
   to address other devices on the same SPI bus.

   If a @ref #BC_SPI_DELAY instruction has been executed previously,
   the bytecode execution will not resume until the delay has elapsed.
 */
#define BC_SPI_YIELD() BC_CUSTOM(0x0020)

/**
   This works like @ref #BC_SPI_YIELD but the delay can be canceled by
   @ref device_spi_request_wakeup. When the delay is canceled, the
   next instruction is skipped.
 */
#define BC_SPI_YIELDC() BC_CUSTOM(0x0000)

/**
   This instruction acts as @ref #BC_SPI_DELAY followed by @ref #BC_SPI_YIELD.
 */
#define BC_SPI_YIELD_DELAY(r)  BC_CUSTOM(0x00a0 | (r & 0xf))

/**
   This works like @ref #BC_SPI_YIELD_DELAY but the delay can be
   canceled by @ref device_spi_request_wakeup.
 */
#define BC_SPI_YIELDC_DELAY(r)  BC_CUSTOM(0x0080 | (r & 0xf))

/**
   This instruction instructs the controller to wait before resuming
   execution of the bytecode. No other request can be serviced on the
   same bus during the delay. Use @ref #BC_SPI_YIELD instead if access
   of the bus by other devices is allowed.

   A previous @ref #BC_SPI_DELAY instruction must be used to setup the delay.

   The chip select policy to apply during the delay must be specified
   by using a value defined in @ref dev_spi_cs_policy_e.
 */
#define BC_SPI_WAIT(cs) BC_CUSTOM(0x0200 | ((cs & 3) << 4))

/**
   This instruction acts as @ref #BC_SPI_WAIT_DELAY followed by @ref #BC_SPI_WAIT.
 */
#define BC_SPI_WAIT_DELAY(r, cs) BC_CUSTOM(0x0280 | (r & 0xf) | ((cs & 3) << 4))

/**
   This instruction set the current chip select policy.

   Some controllers may not be able to support all chip select
   policies.
 */
#define BC_SPI_SETCS(cs) BC_CUSTOM(0x0240 | ((cs & 3) << 4))

/**
   This instruction set the SPI word width in bits and the bit
   order. The width value is a constant between 1 and 32. The order is
   MSB first when @tt order is 1.
 */
#define BC_SPI_WIDTH(width, order) BC_CUSTOM(0x0400 | ((order & 1) << 5) (width & 0x1f))

/**
   This instruction sets the bit transfer rate. The register must
   contain the new bitrate value in bits per second. The old value is
   stored in the register.
 */
#define BC_SPI_BRATE(r) BC_CUSTOM(0x0480 | (w & 0xf))

/**
   This instruction transfers a single word on the SPI bus. The word
   value of the @tt wr register is transmitted. The @tt rd register is
   used to store the received word value, unless @tt rd is 15.
 */
#define BC_SPI_SWP(wr, rd) BC_CUSTOM(0x0800 | ((wr & 0xf) << 4) | (rd & 0xf))

/**
   This instruction transfers up to 8 words on the SPI bus. Word
   values are loaded and stored in contiguous registers. The word
   values of the registers starting at @tt wr are transmitted. The
   registers starting at @tt rd are used to store the received word
   values.

   If the index of the last register to transmit is greater than 14,
   the content of the register 14 is used as padding value for all
   transmitted words. If the index of the last destination register is
   greater than 14, incoming data are discarded and no register is
   modified.
 */
#define BC_SPI_SWPL(wr, rd, count) BC_CUSTOM(0x0800 | ((wr & 0xf) << 4) | (rd & 0xf) | (((count) - 1) << 8))

/**
   This instruction sets the value of a gpio pin.
 */
#define BC_SPI_GPIOSET(index, reg) BC_CUSTOM(0x2000 | (reg & 0xf) | ((index & 0xff) << 4))

/**
   This instruction gets the value of a gpio pin.
 */
#define BC_SPI_GPIOGET(reg, index) BC_CUSTOM(0x3000 | (reg & 0xf) | ((index & 0xff) << 4))

/**
   This instruction mode the value of a gpio pin.
 */
#define BC_SPI_GPIOMODE(index, mode) BC_CUSTOM(0x4000 | (mode & 0xf) | ((index & 0xff) << 4))

/**
   This instruction transfers multiple words on the SPI bus. The
   number of SPI words to transfer is given in the @tt rcnt
   register. The value of the r14 register is used as the padding
   write value.
 */
#define BC_SPI_PAD(rcnt) BC_CUSTOM(0x1000 | (rcnt & 0xf))

/**
   This instruction reads multiple SPI words and store them in a
   buffer of byte values. The value of the r14 register is used as
   the padding write value.
 */
#define BC_SPI_RDM8(raddr, rcnt) BC_CUSTOM(0x1400 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   Same instruction as @ref #BC_SPI_RDM8, using a buffer of
   16 bits words.
 */
#define BC_SPI_RDM16(raddr, rcnt) BC_CUSTOM(0x1500 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   Same instruction as @ref #BC_SPI_RDM8, using a buffer of
   32 bits words.
 */
#define BC_SPI_RDM32(raddr, rcnt) BC_CUSTOM(0x1700 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   This instruction writes multiple SPI words from a buffer of
   bytes. Input values are discarded.
 */
#define BC_SPI_WRM8(raddr, rcnt) BC_CUSTOM(0x1800 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   Same instruction as @ref #BC_SPI_WRM8, using a buffer of
   16 bits words.
 */
#define BC_SPI_WRM16(raddr, rcnt) BC_CUSTOM(0x1900 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   Same instruction as @ref #BC_SPI_WRM8, using a buffer of
   32 bits words.
 */
#define BC_SPI_WRM32(raddr, rcnt) BC_CUSTOM(0x1b00 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   This instruction reads and writes multiple SPI words using two
   buffer of bytes. The address of the read buffer is given by the
   value of the @tt raddr register and the address of the write buffer
   is given by the value of the neighbor register (@tt raddr ^ 1).
 */
#define BC_SPI_SWPM8(raddr, rcnt) BC_CUSTOM(0x1c00 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   Same instruction as @ref #BC_SPI_SWP8, using a buffer of
   16 bits words.
 */
#define BC_SPI_SWPM16(raddr, rcnt) BC_CUSTOM(0x1d00 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

/**
   Same instruction as @ref #BC_SPI_SWP8, using a buffer of
   32 bits words.
 */
#define BC_SPI_SWPM32(raddr, rcnt) BC_CUSTOM(0x1f00 | ((raddr & 0xf) << 4) | (rcnt & 0xf))

#endif /* CONFIG_DEVICE_SPI_REQUEST */

#endif


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

#ifdef CONFIG_DEVICE_SPI_REQUEST
# include <mutek/bytecode.h>
# include <device/class/gpio.h>
# include <device/class/timer.h>
# include <hexo/gpct_platform_hexo.h>
# include <gpct/cont_clist.h>
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
  DEV_SPI_LSB_FIRST,
  DEV_SPI_MSB_FIRST,
};

enum dev_spi_polarity_e
{
  DEV_SPI_CS_ACTIVE_HIGH,
  DEV_SPI_CS_ACTIVE_LOW,
};

enum dev_spi_ckmode_e
{
  DEV_SPI_CK_LOW_LEADING,
  DEV_SPI_CK_LOW_TRAILING,
  DEV_SPI_CK_HIGH_LEADING,
  DEV_SPI_CK_HIGH_TRAILING,
};

struct dev_spi_ctrl_config_s
{
  enum dev_spi_ckmode_e    ck_mode:2;

  enum dev_spi_bit_order_e bit_order:1;
  enum dev_spi_polarity_e  cs_pol:1;
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

enum dev_spi_cs_policy_e
{
  DEV_SPI_CS_SELECT_ON_NEXT_TRANSFER = 0,
  DEV_SPI_CS_SELECT_NOW              = 1,
  DEV_SPI_CS_DESELECT_NOW            = 2,
  DEV_SPI_CS_SELECT_NONE             = 3,
};

#define DEVSPI_CTRL_SELECT(n) error_t (n) (struct device_spi_ctrl_s *scdev, \
                                           enum dev_spi_cs_policy_e p, \
                                           uint_fast8_t cs)
/**
   @This changes the chip select value.

   This function may return @tt -ENOTSUP depending on hardware capabilities.
*/
typedef DEVSPI_CTRL_SELECT(devspi_ctrl_select_t);


/***************************************** transfer */

/** SPI controller device transfer callback */
#define DEVSPI_CTRL_TRANSFER_CALLBACK(n) void (n) (struct dev_spi_ctrl_transfer_s *tr, \
                                                   bool_t nested)

/**
   The SPI controller transfer callback function is called when a
   transfer ends. When this function is called, either the @tt count
   field of the transfer is zero or the @tt err field is set.
 */
typedef DEVSPI_CTRL_TRANSFER_CALLBACK(devspi_ctrl_transfer_callback_t);
struct dev_spi_ctrl_transfer_s
{
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

  /** Function to call when the transfer ends. */
  devspi_ctrl_transfer_callback_t *callback;
  /** Callback private data */
  void                     *pvdata;
  error_t                  err;

  /** Associated SPI controller device */
  struct device_spi_ctrl_s *scdev;

  /** Width in bytes of the data type used to store input SPI words. */
  uint_fast8_t             in_width:3;
  /** Width in bytes of the data type used to load output SPI words. A
      value of 0 means @ref uint32_t without increment of the output
      pointer during transfer. */
  uint_fast8_t             out_width:3;
};
#define DEVSPI_CTRL_TRANSFER(n) error_t (n) (struct device_spi_ctrl_s *scdev, \
                                             struct dev_spi_ctrl_transfer_s *tr, \
                                             bool_t polling)


/**
   @This starts an SPI transfer.

   All fields of the transfer object except @tt pvdata, @tt err and
   @tt scdev must be properly initialized. The @tt count field can not
   be 0. This function will return @tt -EBUSY if a transfer is
   currently being processed.

   Depending on the size of the transfer, the size of the device FIFOs,
   DMA capabilities, the current bit rate and other factors, the driver
   may choose to perform the transfer using polling instead of relying
   on interrupts. In this case the transfer is performed using polling
   mechanisms with the current processor interrupt enable state left
   unchanged during the transfer. If the @tt polling parameter is 0,
   polling is disallowed and the function simply returns @tt -EAGAIN.
   The @tt polling parameter has no effect if the @ref
   #CONFIG_DEVICE_IRQ configuration token is not defined.

   This function returns 0 if the transfer has been started
   asynchronously or 1 if the transfer has been processed
   synchronously. In the later case, the callback function has been
   called with the @tt nested parameter set. In either cases, the @tt
   err field of the transfer object must be checked for errors.
*/
typedef DEVSPI_CTRL_TRANSFER(devspi_ctrl_transfer_t);


/***************************************** queue getter */

#ifdef CONFIG_DEVICE_SPI_REQUEST
#define DEVSPI_CTRL_QUEUE(n) struct dev_spi_ctrl_queue_s * (n)(struct device_spi_ctrl_s *scdev)
typedef DEVSPI_CTRL_QUEUE(devspi_ctrl_queue_t);
#endif

/***************************************** request */

#ifdef CONFIG_DEVICE_SPI_REQUEST

/** @This structure describes actions to perform on a SPI slave device. */
struct dev_spi_ctrl_request_s
{
  /** bytecode virtual machine context */
  struct bc_context_s      vm;

  struct dev_spi_ctrl_config_s config;

  /** request end callback */
#ifdef CONFIG_MUTEK_SCHEDULER
  struct sched_context_s   *sched_ctx;
#endif
  error_t                  err;

  /** This field gives the base unit in timer ticks count used by the
      delay bytecode instructions for this device. */
  dev_timer_delay_t       delay_unit;
  dev_timer_value_t       sleep_before;

  struct device_spi_ctrl_s *scdev;
  struct dev_spi_ctrl_queue_s *queue;
  /** This field indicates the id of the slave on the bus. */
  uint_fast8_t            slave_id;

  /** If this device accessor refers to a gpio device, it will be used
      to driver the chip select pin and aux pins for this SPI slave. If
      it's not valid, the controller chip select mechanism will be
      used if available. */
  struct device_gpio_s    gpio;
  /** If the @ref gpio device accessor is valid and this field is
      not all ones, it indicates the gpio pin id to use to select this
      SPI slave. In the other case, this value may be used by the SPI
      controller driver directly. */
  uint_fast8_t            cs_id;
  /** If the @ref gpio device accessor is valid, this gives the index
      of the first gpio pin to drive when a @ref #BC_SPI_GPIO
      instruction is encountered. */
  uint_fast8_t            gpio_id;

  bool_t                  noyield;

  CONTAINER_ENTRY_TYPE(CLIST)	queue_entry;    //< used by driver to enqueue requests
};

CONTAINER_TYPE(dev_spi_ctrl_queue, CLIST, struct dev_spi_ctrl_request_s, queue_entry);
CONTAINER_FUNC(dev_spi_ctrl_queue, CLIST, static inline, dev_spi_ctrl_queue);

struct dev_spi_ctrl_queue_s
{
  /** This device accessor is used to execute the delay bytecode
      instructions. It may not be valid, in this case any delay
      instruction with a delay greater than zero will make the request
      fail. */
  struct device_timer_s         timer;

  union {
    struct dev_timer_rq_s         timer_rq;
    struct dev_spi_ctrl_transfer_s transfer;
  };

  /** This keep track of the last used configuration. */
  struct dev_spi_ctrl_config_s *config;

  struct dev_spi_ctrl_request_s *current;
  struct dev_spi_ctrl_request_s *timeout;
  dev_spi_ctrl_queue_root_t     queue;

  uint32_t                      slaves_mask;
  lock_t                        lock;
};

static inline error_t
dev_spi_queue_init(struct dev_spi_ctrl_queue_s *q)
{
  device_init_accessor(&q->timer);
  q->config = NULL;
  q->current = NULL;
  q->timeout = NULL;
  dev_spi_ctrl_queue_init(&q->queue);
  q->slaves_mask = 0;
  lock_init(&q->lock);
  return 0;
}

static inline void
dev_spi_queue_cleanup(struct dev_spi_ctrl_queue_s *q)
{
}

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
error_t
dev_spi_request_start(struct device_spi_ctrl_s *scdev,
                      struct dev_spi_ctrl_request_s *rq);
# if 0
/** @This initializes the SPI endpoint structure. It should be called
    from the slave driver initialization function. The @ref cs_gpio
    parameter may be @tt NULL. */
static inline error_t dev_spi_request_init(struct dev_spi_ctrl_request_s *rq,
                                           struct device_s *ctrl, struct device_s *gpio, uint_fast16_t cs_id,
                                           enum dev_spi_polarity_e cs_pol, enum dev_spi_ckmode_e ck_mode)
{
  error_t err;
  if ((err = device_get_accessor(&rq->ctrl, ctrl, DRIVER_CLASS_SPI_CTRL, 0)))
    return err;


  if (gpio == NULL)
    device_init_accessor(&rq->gpio);
  else if ((err = device_get_accessor(&rq->gpio, gpio, DRIVER_CLASS_GPIO, 0)))
    {
      device_put_accessor(&rq->ctrl);
      return err;
    }

  rq->cs_pol = cs_pol;
  rq->ck_mode = ck_mode;
  rq->cs_id = cs_id;
  return err;
}

/** @This cleanups the SPI endpoint structure. It should be called
    from the slave driver cleanup function. */
static inline void dev_spi_request_cleanup(struct dev_spi_ctrl_request_s *rq)
{
  device_put_accessor(&rq->ctrl);
  device_put_accessor(&rq->gpio);
  device_put_accessor(&rq->timer);
}
# endif /* 0 */

#endif

#endif

/***************************************** device class */

DRIVER_CLASS_TYPES(spi_ctrl,
		   devspi_ctrl_config_t         *f_config;
		   devspi_ctrl_select_t         *f_select;
		   devspi_ctrl_transfer_t       *f_transfer;
#ifdef CONFIG_DEVICE_SPI_REQUEST
		   devspi_ctrl_queue_t          *f_queue;
#endif
		   );

/*************************************************************** SPI bytecode */

#ifdef CONFIG_DEVICE_SPI_REQUEST

/*
   @section {SPI request bytecode instructions}
   @code R
    instruction         params        opcode                  format
 -------------------------------------------------------------------

    generic instructions              0--- ---- ---- ----

    yield               r             1000 0000 ---- rrrr
    delay               r             1000 0010 --00 rrrr
    delays              r             1000 0010 --01 rrrr
    delayd              r             1000 0010 --10 rrrr

    width               w, o          1000 0100 --ow wwww
    brate               r             1000 0101 ---- rrrr

    swp                 r, r          1000 0110 rrrr rrrr

    gpio                i, v          1000 0111 viii iiii

    pad                 r             1001 0000 --00 rrrr
    padd                r             1001 0000 --10 rrrr

    rdm[8,16,32]        ra, r         1001 01ss aaaa rrrr
    wrm[8,16,32]        ra, r         1001 10ss aaaa rrrr
    swpm[8,16,32]       ra, r         1001 11ss aaaa rrrr
   @end code
   @end section
*/

/**
   This instruction allows other requests targeting slave on the same
   SPI bus to be processed. The register gives a delay to wait before
   resuming execution of the bytecode for the current slave.

   The chip select may be deasserted.

   The delay given in the register is expressed in timer units as
   defined in the @ref dev_spi_ctrl_request_s structure.
*/
#define BC_SPI_YIELD(r)  BC_CUSTOM(0x0000 | (r & 0xf))

/**
   This instruction instructs the controller to wait before resuming
   execution of the bytecode. The chip select may change but
   no other request can be serviced on the same bus. Use @ref
   #BC_SPI_YIELD instead if access of the bus by other devices is
   allowed.

   The delay given in the register is expressed in timer units as
   defined in the @ref dev_spi_ctrl_request_s structure.
*/
#define BC_SPI_DELAY(r) BC_CUSTOM(0x0200 | (r & 0xf))

/**
   This instruction instructs the controller to wait before resuming
   execution of the bytecode. The chip select remains asserted and
   no other request can be serviced on the same bus.

   Use @ref #BC_SPI_DELAY instead if the state of the chip select
   doesn't matters as some controller might not support keeping the
   chip select asserted at all time. This instruction can be used with
   a delay value of zero in order to set the chip select and check the
   controller ability to keep the chip selected between transfers.
*/
#define BC_SPI_DELAYS(r) BC_CUSTOM(0x0210 | (r & 0xf))

/**
   This instruction instructs the controller to wait before resuming
   execution of the bytecode. The chip select is deasserted and no
   other request can be serviced on the same bus.

   Use @ref #BC_SPI_DELAY instead if the state of the chip select
   doesn't matters as some controller might not support keeping the
   chip select deasserted.
*/
#define BC_SPI_DELAYD(r) BC_CUSTOM(0x0220 | (r & 0xf))

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
#define BC_SPI_BRATE(r) BC_CUSTOM(0x0500 | (w & 0xf))

/**
   This instruction transfers a single word on the SPI bus. The word
   value of the @tt wr register is transmitted. The @tt rd register is
   used to store the received word value.
 */
#define BC_SPI_SWP(wr, rd) BC_CUSTOM(0x0600 | ((wr & 0xf) << 4) | (rd & 0xf))

/**
   This instruction sets the value of a gpio pin.
 */
#define BC_SPI_GPIO(index, value) BC_CUSTOM(0x0700 | ((value & 0x1) << 7) | (index & 0x7f))

/**
   This instruction transfers multiple words on the SPI bus. The value
   of the r14 register is used as the padding write value.
 */
#define BC_SPI_PAD(r) BC_CUSTOM(0x1000 | (r & 0xf))

/**
   This instruction transfers multiple words on the SPI bus with the
   chip select deasserted. The value of the r14 register is used as
   the padding write value.
 */
#define BC_SPI_PADD(r) BC_CUSTOM(0x1020 | (r & 0xf))

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
   bytes. input values are discarded.
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
   value of the @tt raddr register and the value of the write buffer
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


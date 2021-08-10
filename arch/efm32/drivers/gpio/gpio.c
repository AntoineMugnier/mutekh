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

    Copyright (c) 2013 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/devaddr.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/iomux.h>
#include <device/class/icu.h>
#include <device/clock.h>

#include <string.h>

/*
  There is an indefinite number of banks (#bank).
  There are 16 pins (#pin) per bank.
  There are 16 external interrupt lines (#sink).

  An usable external interrupt has an associated sink end-point object
  in the driver. The hardware can be configured to map every 16
  external interrupts to some {bank, pin} tupples. An irq sink index
  in the mutekh API is equivalent to the {bank, pin} tuple. Not all
  associations are allowed; the pin must fall in a specific group
  which depends on the external interrupt.

  On efr32, there are 4 groups of 4 pins in a bank. An external
  interrupt has one hardwired group index (#grp), one selectable pin
  index in the group (#idx) and one selectable bank (#bank).

  On efm32, there are 16 groups of 1 pin in a bank. An external
  interrupt has one selectable bank (#bank). Thus #idx is always 0 and
  #grp is always equal to #pin.
*/

#define GPIO_SRC_IRQ_COUNT 2
#define GPIO_BANK_SIZE 16

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# define GPIO_PINIDX_COUNT 1
# define GPIO_PINGRP_COUNT 16
# define GPIO_BANK_COUNT 6
#endif
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
# define GPIO_PINIDX_COUNT 4
# define GPIO_PINGRP_COUNT 4
# define GPIO_BANK_COUNT 12
#endif

/* This specifies which bank is selected for each interrupt line. */
static ALWAYS_INLINE uint_fast8_t
efm32_gpio_icupv_bank(const struct dev_irq_sink_s *sink)
{
  return sink->icu_pv & 0x1f;
}

/* This specifies which pin in the group is selected for an interrupt. */
static ALWAYS_INLINE uint_fast8_t
efm32_gpio_icupv_idx(const struct dev_irq_sink_s *sink)
{
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
  return sink->icu_pv >> 5;
#else
  return 0;
#endif
}

static ALWAYS_INLINE uint8_t
efm32_gpio_icupv(uint_fast8_t bank, uint_fast8_t idx)
{
  return bank
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
    | idx << 5
#endif
    ;
}

DRIVER_PV(struct efm32_gpio_private_s
{
#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  struct dev_irq_sink_s sink[CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT];

  struct dev_irq_src_s src[2];
#endif

#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
  dev_request_queue_root_t queue;
#endif

  struct dev_clock_sink_ep_s    clk_ep;
});

/* This function returns a 32 bits mask from a 8 bits value 
   if msk = 0b10101010 for instance, get_msk(msk) returns
   0xf0f0f0f0 */
static inline uint32_t get_mask(uint8_t msk)
{
  uint32_t p0 = (0x00208208 * (msk & 0x55)) & 0x08080808;
  uint32_t p1 = (0x01041040 * (msk & 0xaa)) & 0x80808080;

  uint32_t r = p0 | p1;

  r |= r >> 1;
  r |= r >> 2;

  return r;
}

static void efm32_gpio_out_reg(gpio_id_t io_first, gpio_id_t io_last,
                               const uint8_t *set_mask, const uint8_t *clear_mask)
{
  uint32_t cm, sm, tmask;
  uint32_t cmp = 0;
  uint32_t smp = 0;

  uint_fast8_t shift = io_first % GPIO_BANK_SIZE;
  int_fast8_t mlen  = io_last - io_first + 1;

 mask:
  /* compute mask word for next clear_mask and set_mask */
  tmask = mlen > GPIO_BANK_SIZE ? 0xffff : bit_mask(0, mlen);

 loop:
  /* compute set and clear masks */
  cm = (uint32_t)((uint16_t)~endian_le16_na_load(clear_mask) & tmask);
  cmp = ((cm << shift) | cmp);
  clear_mask += 2;

  sm = (uint32_t)(endian_le16_na_load(set_mask) & tmask);
  smp = ((sm << shift) | smp);
  set_mask += 2;

  mlen -= GPIO_BANK_SIZE;

 last:;
  /* update DOUT register */
  uintptr_t a = EFM32_GPIO_ADDR + EFM32_GPIO_DOUT_ADDR(io_first / GPIO_BANK_SIZE);
  uint32_t x = endian_le32(cpu_mem_read_32(a));
  x = smp ^ (x & (smp ^ ~cmp));

  cpu_mem_write_32(a, endian_le32(x & EFM32_GPIO_DOUT_MASK));

  cmp >>= GPIO_BANK_SIZE;
  smp >>= GPIO_BANK_SIZE;

  io_first = (io_first | (GPIO_BANK_SIZE - 1)) + 1;

  if (mlen >= GPIO_BANK_SIZE)
    goto loop;   /* mask is still 0xffff, no need to recompute */

  if (io_first <= io_last)
    {
      if (mlen < 0)
	goto last;   /* last remaining bits in next register, mask
                        bits already available */

      goto mask;     /* need to compute new mask for the last word */
    }
}

static void efm32_gpio_mode_reg(gpio_id_t io_first, gpio_id_t io_last,
                                const uint8_t *mask, uint32_t mde)
{
  uint64_t m, mp = 0;

  uint8_t tmask;
  uint_fast8_t shift = io_first % (GPIO_BANK_SIZE/2);
  int_fast8_t mlen  = io_last - io_first + 1;

 mask:
  /* compute mask word for next mask cell */
  tmask = mlen > GPIO_BANK_SIZE/2 ? 0xff : bit_mask(0, mlen);

 loop:
  m = get_mask(*mask++ & tmask);
  mp = (m << (shift * 4)) | mp;

  mlen -= GPIO_BANK_SIZE/2;

 last:;
  uintptr_t a = EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(io_first / GPIO_BANK_SIZE)
              + ((io_first & 8) >> 1);
  uint32_t x = endian_le32(cpu_mem_read_32(a));
  x = (x & ~mp) | (mp & mde);

  cpu_mem_write_32(a, endian_le32(x));

  mp >>= 4 * GPIO_BANK_SIZE/2;

  io_first = (io_first | (GPIO_BANK_SIZE/2 - 1)) + 1;

  if (mlen >= GPIO_BANK_SIZE/2)
    goto loop;   /* mask is still 0xff, no need to recompute */

  if (io_first <= io_last)
    {
      if (mlen < 0)
	goto last;   /* last remaining bits in next register, mask
                        bits already available */

      goto mask;     /* need to compute new mask for the last word */
    }
}

static error_t efm32_gpio_mode(gpio_id_t io_first, gpio_id_t io_last,
                               const uint8_t *mask, enum dev_pin_driving_e mode)
{
  uint32_t mde;

  switch (mode)
    {
    case DEV_PIN_DISABLED:
      mde = EFM32_GPIO_MODEL_MODE_DISABLED;
      break;
    case DEV_PIN_PUSHPULL:
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
      mde = EFM32_GPIO_MODEL_MODE_PUSHPULL;
#else
      mde = EFM32_GPIO_MODEL_MODE_PUSHPULLDRIVE;
#endif
      break;
    case DEV_PIN_INPUT:
      mde = EFM32_GPIO_MODEL_MODE_INPUT;
      break;
    case DEV_PIN_INPUT_PULLUP:
    case DEV_PIN_INPUT_PULLDOWN:
    case DEV_PIN_INPUT_PULL:
      mde = EFM32_GPIO_MODEL_MODE_INPUTPULL;
      break;
    case DEV_PIN_OPENDRAIN:
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
      mde = EFM32_GPIO_MODEL_MODE_WIREDAND;
#else
      mde = EFM32_GPIO_MODEL_MODE_WIREDANDDRIVE;
#endif
      break;
    case DEV_PIN_OPENSOURCE:
      mde = EFM32_GPIO_MODEL_MODE_WIREDOR;
      break;
    case DEV_PIN_OPENDRAIN_PULLUP:
      mde = EFM32_GPIO_MODEL_MODE_WIREDANDPULLUP;
      break;
    case DEV_PIN_OPENSOURCE_PULLDOWN:
      mde = EFM32_GPIO_MODEL_MODE_WIREDORPULLDOWN;
      break;
    default:
      return -ENOTSUP;
    }

  efm32_gpio_mode_reg(io_first, io_last, mask, mde * 0x11111111);

  if (mode & DEV_PIN_RESISTOR_UP_)
    efm32_gpio_out_reg(io_first, io_last, mask, dev_gpio_mask1);
  else if (mode & DEV_PIN_RESISTOR_DOWN_)
    {
      uint8_t m[8];
      endian_64_na_store(m, ~endian_64_na_load(mask));
      efm32_gpio_out_reg(io_first, io_last, dev_gpio_mask0, m);
    }

  return 0;
}

static DEV_GPIO_SET_MODE(efm32_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * GPIO_BANK_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);
  efm32_gpio_mode(io_first, io_last, mask, mode);
  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_SET_OUTPUT(efm32_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * GPIO_BANK_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);
  efm32_gpio_out_reg(io_first, io_last, set_mask, clear_mask);
  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_GET_INPUT(efm32_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * GPIO_BANK_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t vp, v;
  uint_fast8_t bf = io_first / GPIO_BANK_SIZE;
  uint_fast8_t bl = io_last / GPIO_BANK_SIZE;
  uint_fast8_t shift = io_first % GPIO_BANK_SIZE;

  vp = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_DIN_ADDR(bf)));
  vp >>= shift;

  while (bf++ < bl)
    {
      v = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_DIN_ADDR(bf)));
      v = (v << (GPIO_BANK_SIZE - shift)) | vp ;
      vp = v >> GPIO_BANK_SIZE;

      endian_le16_na_store(data, endian_le16((uint16_t)v));
      data += 2;
    }

  endian_le16_na_store(data, endian_le16((uint16_t)vp));

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
static void
efm32_gpio_request_until(struct device_s *dev,
                         struct efm32_gpio_private_s *pv,
                         struct dev_gpio_rq_s *rq)
{
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  const uint8_t *datap = rq->until.data;
  const uint8_t *maskp = rq->until.mask;

  uint16_t extirise = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIRISE_ADDR);
  uint16_t extifall = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIFALL_ADDR);
  uint16_t extien   = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR);
  uint16_t irise = 0, ifall = 0;
  gpio_id_t io;

  for (io = rq->io_first; io <= rq->io_last; io += 32)
    {
      uint32_t mask = endian_le32_na_load(maskp);
      uint32_t data = endian_le32_na_load(datap);
      maskp += 4;
      datap += 4;

      while (mask)
        {
          uint_fast8_t k, j = bit_ctz(mask);
          uint_fast8_t i = io + j;

          if (i > rq->io_last)
            goto push;
          mask ^= 1 << j;

          uint_fast8_t pin = i % GPIO_BANK_SIZE;
          uint_fast8_t bank = i / GPIO_BANK_SIZE;

          /* grpx = #grp * GPIO_PINIDX_COUNT */
          uint_fast8_t grpx = pin & ~(GPIO_PINIDX_COUNT - 1);
          uint_fast8_t sink_id;

          /* Find an external interrupt which can be associated to the
             monitored pin. */
          for (k = 0; k < GPIO_BANK_SIZE / GPIO_PINGRP_COUNT; k++)
            {
              sink_id = grpx + k;
#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
              struct dev_irq_sink_s *sink = pv->sink + sink_id;
              if (sink->base.link_count != 0)
                continue;
#endif
              if (((extien | irise | ifall) >> sink_id) & 1)
                continue;
              goto found;
            }

          rq->error = -EBUSY;
          goto done;

        found:;
          uintptr_t h = (sink_id & 8) >> 1; /* 4 when using high registers */

          /* Select interrupt bank */
          uint32_t x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIPSELL_ADDR + h));
          EFM32_GPIO_EXTIPSELL_EXT_SETVAL(sink_id & 7, x, bank);
          cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIPSELL_ADDR + h, endian_le32(x));

#if GPIO_PINIDX_COUNT > 1
          /* Select pin in the group */
          uint_fast8_t idx = pin & (GPIO_PINIDX_COUNT - 1);
          x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIGSELL_ADDR + h));
          EFM32_GPIO_EXTIGSELL_EXT_SETVAL(sink_id & 7, x, idx);
          cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIGSELL_ADDR + h, endian_le32(x));
#endif

          bool_t expected = (data >> j) & 1;
          uint16_t m = 1 << sink_id;

          /* Enable falling or rising edge interrupt */
          if (expected)
            {
              ifall |= m;
              cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIFALL_ADDR,
                               endian_le32(extifall | ifall));
            }
          else
            {
              irise |= m;
              cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIRISE_ADDR,
                               endian_le32(extirise | irise));
            }

          cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR,
                           endian_le32(m));
          cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR,
                           endian_le32(extien | ifall | irise));

          /* Check current pin status */
          uint16_t din = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_DIN_ADDR(bank)));

          if (((din >> pin) ^ expected) & 1)
            {
              rq->error = 0;
              goto done;
            }
        }
    }

 push:
  ifall |= irise;
  if (!ifall)
    goto empty;
  rq->base.drvuint = ifall;
  dev_gpio_rq_pushback(&pv->queue, rq);
  return;

 done:
  /* revert external interrupts enabling */
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIFALL_ADDR,
                   endian_le32(extifall));
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIRISE_ADDR,
                   endian_le32(extirise));
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR,
                   endian_le32(extien));

  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR,
                   endian_le32(irise | ifall));

 empty:
  dev_gpio_rq_done(rq);
}
#endif

static DEV_GPIO_REQUEST(efm32_gpio_request)
{
#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
  struct device_s *dev = gpio->dev;
  if (rq->type == DEV_GPIO_UNTIL)
    return efm32_gpio_request_until(dev, dev->drv_pv, rq);
#endif

  dev_gpio_request_async_to_sync(gpio, rq);
}

#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
static void efm32_gpio_until_clear(uint32_t m)
{
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIRISE_ADDR,
                   cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIRISE_ADDR)
                   & endian_le32(~m));
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIFALL_ADDR,
                   cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIFALL_ADDR)
                   & endian_le32(~m));
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR,
                   cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR)
                   & endian_le32(~m));
}
#endif

static DEV_GPIO_CANCEL(efm32_gpio_cancel)
{
#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
  struct device_s *dev = gpio->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq->type == DEV_GPIO_UNTIL)
    {
      uint32_t m = rq->base.drvuint;
      if (m == 0)
        return -EBUSY;

      dev_gpio_rq_remove(&pv->queue, rq);
      efm32_gpio_until_clear(m);
      cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR, endian_le32(m));
      rq->base.drvuint = 0;
      return 0;
    }
#endif
  return -ENOTSUP;
}

#define efm32_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn

/******** GPIO iomux controller driver part *********************/

static DEV_IOMUX_SETUP(efm32_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;

  if (io_id >= GPIO_BANK_SIZE * GPIO_BANK_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);
  efm32_gpio_mode(io_id, io_id, dev_gpio_mask1, dir);
  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

/******** GPIO irq controller driver part *********************/

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU

static DEV_IRQ_SINK_UPDATE(efm32_gpio_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sink;
  uint32_t mask = bit(sink_id);

  /* Select polarity of interrupt edge */
  uint32_t e, d;

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE: {
      /* Disable external interrupt */
      uint32_t x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR));
      x &= ~mask;
      cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR, endian_le32(x));
      return;
    }

    case DEV_IRQ_SENSE_FALLING_EDGE:
      e = 0;
      d = 0xffffffff;
      break;
    case DEV_IRQ_SENSE_RISING_EDGE:
      e = 0xffffffff;
      d = 0;
      break;
    case DEV_IRQ_SENSE_ANY_EDGE:
      d = e = 0xffffffff;
      break;

    default:
      return;
    }

  /* Set polarity */
  uint32_t x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIRISE_ADDR));
  x = (mask & e) | (~mask & x);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIRISE_ADDR, endian_le32(x));

  x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIFALL_ADDR));
  x = (mask & d) | (~mask & x);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIFALL_ADDR, endian_le32(x));

  /* Enable interrupt */
  x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR));
  x |= mask;
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR, endian_le32(x));
}

static DEV_ICU_GET_SINK(efm32_gpio_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;
  uint8_t pin = id % GPIO_BANK_SIZE;
  uint_fast8_t bank = id / GPIO_BANK_SIZE;

  if (pin >= CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT || bank >= GPIO_BANK_COUNT)
    return NULL;

  uint_fast8_t i = 0;
  uint_fast8_t grpx = pin & ~(GPIO_PINIDX_COUNT - 1); /* grpx = grp * GPIO_PINIDX_COUNT */
  uint_fast8_t idx  = pin &  (GPIO_PINIDX_COUNT - 1);
  uint8_t icupv = efm32_gpio_icupv(bank, idx);

  /* Find a external interrupt (sink endpoint) which can be associated
     to the selected pin. */
  for (i = 0; i < GPIO_BANK_SIZE / GPIO_PINGRP_COUNT; i++)
    {
      uint_fast8_t sink_id = grpx + i;
      struct dev_irq_sink_s *sink = pv->sink + sink_id;

      if (sink->base.link_count == 0)
        {
#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
          uint16_t mask = 1 << sink_id;
          if (endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR)) & mask)
            continue;
#endif
          sink->icu_pv = icupv;
          return sink;
        }

      /* Already linked with the same configuration */
      if (sink->icu_pv == icupv)
        return sink;
    }

  return NULL;
}

static DEV_ICU_LINK(efm32_gpio_icu_link)
{
  struct device_s *dev = accessor->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sink;
  uint_fast8_t bank = efm32_gpio_icupv_bank(sink);
  uint_fast8_t h = (sink_id & 8) >> 1; /* 4 when using high registers */

#ifdef CONFIG_DEVICE_IRQ_SHARING
  if (sink->base.link_count > 1)
    return 0;
#endif

  if (route_mask == NULL)
    return 0;

#ifdef CONFIG_DEVICE_IRQ_BYPASS
  if (*bypass)
    return 0;
#endif

  /* Select bank */
  uint32_t x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIPSELL_ADDR + h));
  EFM32_GPIO_EXTIPSELL_EXT_SETVAL(sink_id & 7, x, bank);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIPSELL_ADDR + h, endian_le32(x));

  uint_fast8_t idx = efm32_gpio_icupv_idx(sink);
#if GPIO_PINIDX_COUNT > 1
  /* Select pin in the group */
  x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIGSELL_ADDR + h));
  EFM32_GPIO_EXTIGSELL_EXT_SETVAL(sink_id & 7, x, idx);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_EXTIGSELL_ADDR + h, endian_le32(x));
#endif

  /* Change pin mode to input */
  x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(bank) + h));
  uint_fast8_t grpx = sink_id & ~(GPIO_PINIDX_COUNT - 1);
  uint8_t pin_id = grpx + idx;
  EFM32_GPIO_MODEL_MODE_SET(pin_id & 7, x, INPUT);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_MODEL_ADDR(bank) + h, endian_le32(x));

  /* Clear interrupt */
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR, endian_le32(bit(sink_id)));

  return 0;
}

#endif

static DEV_IRQ_SRC_PROCESS(efm32_gpio_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      uint32_t x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IF_ADDR));

      if (!x)
        break;

#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
      /* handle gpio DEV_GPIO_UNTIL requests */
      GCT_FOREACH(dev_request_queue, &pv->queue, rq, {
          uint16_t m = rq->drvuint;
          if (m & x)
            {
              x ^= m;
              efm32_gpio_until_clear(m);
              rq->drvuint = 0;
              rq->error = 0;
              dev_gpio_rq_remove(&pv->queue, dev_gpio_rq_s_cast(rq));
              dev_gpio_rq_done(dev_gpio_rq_s_cast(rq));
            }
      });
#endif

      cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR, endian_le32(x));

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
      x &= bit_mask(0, CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT);
      while (x)
        {
          uint_fast8_t i = bit_ctz(x);
          struct dev_irq_sink_s *sink = pv->sink + i;

# ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
          if (sink->base.link_count != 0)
# endif
            {
              uint_fast8_t bank = efm32_gpio_icupv_bank(sink);
              uint8_t idx = (i & ~(GPIO_PINIDX_COUNT - 1)) + efm32_gpio_icupv_idx(sink);
              int_fast16_t id = (cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_DIN_ADDR(bank)) >> idx) & 1;
              device_irq_sink_process(sink, id);
            }

          x ^= bit(i);
        }
#endif
    }

  lock_release(&dev->lock);
}

/******** GPIO generic driver part *********************/

#define efm32_gpio_use dev_use_generic

static DEV_INIT(efm32_gpio_init)
{
  struct efm32_gpio_private_s *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         EFM32_GPIO_ADDR == addr);

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL)) {
    logk_fatal("Bad clock init");
    goto err_mem;
  }

  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR, 0);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR, 0xffffffff);

#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
  dev_rq_queue_init(&pv->queue);
#endif

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_init(dev, pv->src, GPIO_SRC_IRQ_COUNT,
                    &efm32_gpio_source_process);

  if (device_irq_source_link(dev, pv->src, GPIO_SRC_IRQ_COUNT, -1)) {
    logk_fatal("Bad IRQ init");
    goto err_clk;
  }
#endif

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  device_irq_sink_init(dev, pv->sink, CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT,
                       &efm32_gpio_icu_sink_update,
                       DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE |
                       DEV_IRQ_SENSE_ANY_EDGE);
#endif

#if CONFIG_DRIVER_EFM32_GPIO_DRIVE_STRENGH > 0
  uint_fast8_t i;
  for (i = 0; i < 6; i++)
    cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_CTRL_ADDR(i),
                     ((CONFIG_DRIVER_EFM32_GPIO_DRIVE_STRENGH) >> (i * 2)) & 0x3);
#endif

  return 0;

#if 0
 err_unlink:
  device_irq_source_unlink(dev, pv->src, GPIO_SRC_IRQ_COUNT);
#endif
 err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_gpio_cleanup)
{
  struct efm32_gpio_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR, 0);

  device_irq_source_unlink(dev, pv->src, GPIO_SRC_IRQ_COUNT);
#endif

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

#ifdef CONFIG_DRIVER_EFM32_GPIO_UNTIL
  dev_rq_queue_destroy(&pv->queue);
#endif

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(efm32_gpio_drv, 0, "EFM32 GPIO", efm32_gpio,
               DRIVER_GPIO_METHODS(efm32_gpio),
#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
               DRIVER_ICU_METHODS(efm32_gpio_icu),
#endif
               DRIVER_IOMUX_METHODS(efm32_gpio_iomux));

DRIVER_REGISTER(efm32_gpio_drv);


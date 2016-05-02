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

#include <mutek/mem_alloc.h>
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

#define GPIO_SRC_IRQ_COUNT 2
#define GPIO_BANK_SIZE 16

struct efm32_gpio_private_s
{
#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  struct dev_irq_sink_s sink[CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT];

  /* This specifies which bank is selected for each interrupt line. A
     value of -1 means that no bank is currently bound to an
     interrupt. */
  struct {
    int8_t            bank:5;
  }                   irq[CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT];

  struct dev_irq_src_s src[2];
#endif

  struct dev_clock_sink_ep_s    clk_ep;
};

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
  tmask = mlen > GPIO_BANK_SIZE ? 0xffff : ((1 << mlen) - 1);

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
  tmask = mlen > GPIO_BANK_SIZE/2 ? 0xff : ((1 << mlen) - 1);

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
      mde = EFM32_GPIO_MODEL_MODE_PUSHPULL;
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
      mde = EFM32_GPIO_MODEL_MODE_WIREDAND;
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

  if (io_last >= GPIO_BANK_SIZE * 6)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);
  efm32_gpio_mode(io_first, io_last, mask, mode);
  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_SET_OUTPUT(efm32_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * 6)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);
  efm32_gpio_out_reg(io_first, io_last, set_mask, clear_mask);
  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

/* This function returns the GPIO input values of pins whose index
   is included in range [io_first, io_last]. Size of data is a
   multiple of 4 bytes. We do not care about bits outside range in 
   data
*/
static DEV_GPIO_GET_INPUT(efm32_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * 6)
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

#define efm32_gpio_request dev_gpio_request_async_to_sync
#define efm32_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn

/******** GPIO iomux controller driver part *********************/

static DEV_IOMUX_SETUP(efm32_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;

  if (io_id >= GPIO_BANK_SIZE * 6)
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
  uint32_t mask = 1 << sink_id;

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
  uint8_t line = id % GPIO_BANK_SIZE;
  uint_fast8_t bank = id / GPIO_BANK_SIZE;

  if (line >= CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT || bank >= 6)
    return NULL;

  struct dev_irq_sink_s *sink = pv->sink + line;

  /* We actually keep only one end-point object per line for all
     banks. We have to keep track of which bank the end-point is
     associated to. */
  if (sink->base.link_count == 0)
    pv->irq[line].bank = bank;
  else if (pv->irq[line].bank != bank)
    return NULL;

  return sink;
}

static DEV_ICU_LINK(efm32_gpio_icu_link)
{
  if (!route_mask || *bypass)
    return 0;

  struct device_s *dev = accessor->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sink;
  uint_fast8_t bank = pv->irq[sink_id].bank;

#ifdef CONFIG_DEVICE_IRQ_SHARING
  if (sink->base.link_count > 1)
    return 0;
#endif

  /* Select bank */
  uintptr_t a = sink_id >= 8 ? EFM32_GPIO_EXTIPSELH_ADDR : EFM32_GPIO_EXTIPSELL_ADDR;
  uint32_t x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + a));
  EFM32_GPIO_EXTIPSELL_EXT_SETVAL(sink_id & 7, x, bank);
  cpu_mem_write_32(EFM32_GPIO_ADDR + a, endian_le32(x));

  /* Change pin mode to input */
  a = sink_id >= 8 ? EFM32_GPIO_MODEH_ADDR(bank) : EFM32_GPIO_MODEL_ADDR(bank);
  x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + a));
  EFM32_GPIO_MODEL_MODE_SET(sink_id & 7, x, INPUT);
  cpu_mem_write_32(EFM32_GPIO_ADDR + a, endian_le32(x));

  /* Clear interrupt */
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR, endian_le32(1 << sink_id));
  x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR));
  x |= (1 << sink_id);
  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IEN_ADDR, endian_le32(x));

  return 0;
}

static DEV_IRQ_SRC_PROCESS(efm32_gpio_source_process)
{
  struct efm32_gpio_private_s *pv = ep->base.dev->drv_pv;

  while (1)
    {
      uint32_t x = cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_IF_ADDR);

      if (!x)
        break;

      cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_IFC_ADDR, x);
      x = endian_le32(x);

      while (x)
        {
          uint_fast8_t i = __builtin_ctz(x);
          struct dev_irq_sink_s *sink = pv->sink + i;
          int_fast16_t id = (cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_DIN_ADDR(pv->irq[i].bank)) >> i) & 1;
          device_irq_sink_process(sink, id);
          x ^= 1 << i;
        }
    }
}

#endif

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

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_SINK_SYNC, NULL))
    goto err_mem;

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  device_irq_source_init(dev, pv->src, GPIO_SRC_IRQ_COUNT,
                    &efm32_gpio_source_process);

  if (device_irq_source_link(dev, pv->src, GPIO_SRC_IRQ_COUNT, -1))
    goto err_clk;

  device_irq_sink_init(dev, pv->sink, CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT,
                       &efm32_gpio_icu_sink_update,
                       DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE |
                       DEV_IRQ_SENSE_ANY_EDGE);
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


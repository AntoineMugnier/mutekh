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
#include <arch/efm32_gpio.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/iomux.h>
#include <device/class/icu.h>
#include <device/class/clock.h>

#include <mutek/printk.h>
#include <string.h>

#define GPIO_SRC_IRQ_COUNT 2
#define GPIO_BANK_SIZE 16

struct efm32_gpio_private_s
{
  uintptr_t           addr; 

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  struct dev_irq_ep_s sink[CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT];

  /* This specifies which bank is selected for each interrupt line. A
     value of -1 means that no bank is currently bound to an
     interrupt. */
  struct {
    int8_t            bank:5;
    int8_t            enabled:1;
  }                   irq[CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT];

  struct dev_irq_ep_s src[2];
#endif

#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s    clk_ep;
#endif
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

static error_t efm32_gpio_mode(enum dev_pin_driving_e mode,
                               uint32_t *mde)
{
  switch (mode)
    {
    case DEV_PIN_DISABLED:
      *mde = EFM32_GPIO_MODEL_MODE_DISABLED;
      break;
    case DEV_PIN_PUSHPULL:
      *mde = EFM32_GPIO_MODEL_MODE_PUSHPULL;
      break;
    case DEV_PIN_INPUT:
      *mde = EFM32_GPIO_MODEL_MODE_INPUT;
      break; 
    case DEV_PIN_INPUT_PULL:
      *mde = EFM32_GPIO_MODEL_MODE_INPUTPULL;
      break;
      /* FIXME support for DEV_PIN_INPUT_PULLUP and
         DEV_PIN_INPUT_PULLDOWN would require updating DOUT regs too. */
    case DEV_PIN_OPENDRAIN:
      *mde = EFM32_GPIO_MODEL_MODE_WIREDAND;
      break;
    case DEV_PIN_OPENSOURCE:
      *mde = EFM32_GPIO_MODEL_MODE_WIREDOR;
      break;
    case DEV_PIN_OPENDRAIN_PULLUP:
      *mde = EFM32_GPIO_MODEL_MODE_WIREDANDPULLUP;
      break;
    case DEV_PIN_OPENSOURCE_PULLDOWN:
      *mde = EFM32_GPIO_MODEL_MODE_WIREDANDPULLUP;
      break;
    default:
      return -ENOTSUP;
    }
  return 0;
}

static DEV_GPIO_SET_MODE(efm32_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;

  if (io_last >= GPIO_BANK_SIZE * 6)
    return -ERANGE;

  uint32_t mde;
  if (efm32_gpio_mode(mode, &mde))
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  /* Build a 32 bits mode value */ 
  mde *= 0x11111111;

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
  uintptr_t a = pv->addr + EFM32_GPIO_MODEL_ADDR(io_first / GPIO_BANK_SIZE)
              + ((io_first & 8) >> 1);
  uint32_t x = endian_le32(cpu_mem_read_32(a));
  x = (x & ~mp) | (mp & mde); 

  //  printk("gpio mode: reg=%08x value=%08x\n", a, x);
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

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_SET_OUTPUT(efm32_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * 6)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  struct efm32_gpio_private_s *pv = dev->drv_pv;

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
  uintptr_t a = pv->addr + EFM32_GPIO_DOUT_ADDR(io_first / GPIO_BANK_SIZE);
  uint32_t x = endian_le32(cpu_mem_read_32(a));
  uint32_t tg = cmp & smp;
  x = ((x ^ tg) & (~cmp | smp)) | smp;
  //  printk("gpio set : reg=%08x value=%08x clr=%08x set=%08x\n", a, x, cmp, smp);
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

  struct efm32_gpio_private_s *pv = dev->drv_pv;
  uint32_t vp, v;
  uint_fast8_t bf = io_first / GPIO_BANK_SIZE;
  uint_fast8_t bl = io_last / GPIO_BANK_SIZE;
  uint_fast8_t shift = io_first % GPIO_BANK_SIZE;

  vp = endian_le32(cpu_mem_read_32(pv->addr + EFM32_GPIO_DIN_ADDR(bf)));
  vp >>= shift;

  while (bf++ < bl)
    {
      v = endian_le32(cpu_mem_read_32(pv->addr + EFM32_GPIO_DIN_ADDR(bf)));
      v = (v << (GPIO_BANK_SIZE - shift)) | vp ;
      vp = v >> GPIO_BANK_SIZE;

      endian_le16_na_store(data, endian_le16((uint16_t)v));
      data += 2;
    }

  endian_le16_na_store(data, endian_le16((uint16_t)vp));

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;

}

static const struct driver_gpio_s efm32_gpio_gpio_drv =
  {
    .class_         = DRIVER_CLASS_GPIO,
    .f_set_mode     = efm32_gpio_set_mode,
    .f_set_output   = efm32_gpio_set_output,
    .f_get_input    = efm32_gpio_get_input,
    .f_request      = dev_gpio_request_async_to_sync,
  };

/******** GPIO iomux controller driver part *********************/

static DEV_IOMUX_SETUP(efm32_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;

  if (io_id >= GPIO_BANK_SIZE * 6)
    return -ERANGE;

  uint32_t mde;
  if (efm32_gpio_mode(dir, &mde))
    return -ENOTSUP;

  uintptr_t a = pv->addr + EFM32_GPIO_MODEL_ADDR(io_id / GPIO_BANK_SIZE)
              + ((io_id & 8) >> 1);

  uint_fast8_t shift = (io_id % 8) * 4;
  uint32_t mp = 0xf << shift;

  uint32_t x = endian_le32(cpu_mem_read_32(a));
  x = (x & ~mp) | (mde << shift);

  cpu_mem_write_32(a, endian_le32(x));

  return 0;
}

static const struct driver_iomux_s efm32_gpio_iomux_drv =
  {
    .class_         = DRIVER_CLASS_IOMUX,
    .f_setup        = efm32_gpio_iomux_setup,
  };

/******** GPIO irq controller driver part *********************/

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU

static DEV_ICU_GET_ENDPOINT(efm32_gpio_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK: {
      uint8_t line = id % 16;
      uint_fast8_t bank = id / 16;

      if (line >= CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT || bank >= 6)
        return NULL;
      struct dev_irq_ep_s *ep = pv->sink + line;

      /* We actually keep only one end-point object per line for all
         banks. We have to keep track of which bank the end-point is
         associated to. */
      if (!ep->link_count)
        {
          pv->irq[line].bank = bank;
          pv->irq[line].enabled = 0;
          ep->sense = DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE;
        }
      else if (pv->irq[line].bank != bank)
        return NULL;

      return ep;
    }

    case DEV_IRQ_EP_SOURCE:
      if (id < GPIO_SRC_IRQ_COUNT)
        return pv->src + id;

    default:
      return NULL;
    }
}

static DEV_ICU_ENABLE_IRQ(efm32_gpio_icu_enable_irq)
{
  struct device_s *dev = accessor->dev;
  struct efm32_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sink;

  if (irq_id > 0)
    {
      printk("EFM32 GPIO %p: single wire IRQ must use 0 as logical IRQ id for %p device\n", dev, dev_ep->dev);
      return 0;
    }

  uint_fast8_t sense = src->sense & sink->sense;

  if (!sense)
    return 0;

  uint_fast8_t line = icu_in_id % 16;
  uint_fast8_t bank = pv->irq[line].bank;

  /* Select polarity of interrupt edge */
  uintptr_t e = EFM32_GPIO_EXTIRISE_ADDR;
  uintptr_t d = EFM32_GPIO_EXTIFALL_ADDR;
  if (sense & DEV_IRQ_SENSE_FALLING_EDGE)
    {
      e = EFM32_GPIO_EXTIFALL_ADDR;
      d = EFM32_GPIO_EXTIRISE_ADDR;
    }
  else if (!(sense & DEV_IRQ_SENSE_RISING_EDGE))
    return 0;

  if (!device_icu_irq_enable(pv->src + icu_in_id % 2, 0, NULL, dev_ep))
    {
      printk("EFM32 ODD GPIO: source end-point can not relay interrupt for %p device\n", dev_ep->dev);
      return 0;
    }

  /* if more than one mode is left, keep only the lsb */
  sense = sense & ~(sense - 1);
  src->sense = sink->sense = sense;

  if (!pv->irq[line].enabled)
    {
      /* set polarity */
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + e));
      x |= 1 << line;
      cpu_mem_write_32(pv->addr + e, endian_le32(x));

      x = cpu_mem_read_32(pv->addr + d);
      x &= ~(1 << line);
      cpu_mem_write_32(pv->addr + d, endian_le32(x));

      /* Select bank */
      uintptr_t a = line >= 8 ? EFM32_GPIO_EXTIPSELH_ADDR : EFM32_GPIO_EXTIPSELL_ADDR;
      x = endian_le32(cpu_mem_read_32(pv->addr + a));
      EFM32_GPIO_EXTIPSELL_EXT_SETVAL(line & 7, x, bank);
      cpu_mem_write_32(pv->addr + a, endian_le32(x));

      /* Change pin mode to input */
      a = line >= 8 ? EFM32_GPIO_MODEH_ADDR(bank) : EFM32_GPIO_MODEL_ADDR(bank);
      x = endian_le32(cpu_mem_read_32(pv->addr + a));
      EFM32_GPIO_MODEL_MODE_SET(line & 7, x, INPUT);
      cpu_mem_write_32(pv->addr + a, endian_le32(x));

      /* Clear and enable interrupt */
      cpu_mem_write_32(pv->addr + EFM32_GPIO_IFC_ADDR, endian_le32(1 << line));
      x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_GPIO_IEN_ADDR));
      x |= (1 << line);
      cpu_mem_write_32(pv->addr + EFM32_GPIO_IEN_ADDR, endian_le32(x));

      pv->irq[line].enabled = 1;
    }

  return 1;
}

static DEV_ICU_DISABLE_IRQ(efm32_gpio_icu_disable_irq)
{
  struct efm32_gpio_private_s *pv = accessor->dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sink;

  uint_fast8_t line = icu_in_id % 16;

  /* Disable external interrupt */
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_GPIO_IEN_ADDR));
  x &= ~(1 << line);
  cpu_mem_write_32(pv->addr + EFM32_GPIO_IEN_ADDR, endian_le32(x));
}

static DEV_IRQ_EP_PROCESS(efm32_gpio_source_process)
{
  struct efm32_gpio_private_s *pv = ep->dev->drv_pv;

  while (1)
    {
      uint32_t x = cpu_mem_read_32(pv->addr + EFM32_GPIO_IF_ADDR);

      if (!x)
        break;

      cpu_mem_write_32(pv->addr + EFM32_GPIO_IFC_ADDR, x);
      x = endian_le32(x);

      while (x)
        {
          uint_fast8_t i = __builtin_ctz(x);
          struct dev_irq_ep_s *sink = pv->sink + i;
          sink->process(sink, id);
          x ^= 1 << i;
        }
    }
}

const struct driver_icu_s efm32_gpio_icu_drv =
  {
    .class_         = DRIVER_CLASS_ICU,
    .f_get_endpoint = efm32_gpio_icu_get_endpoint,
    .f_enable_irq   = efm32_gpio_icu_enable_irq,
    .f_disable_irq  = efm32_gpio_icu_disable_irq,
  };

#endif

static DEV_INIT(efm32_gpio_init);
static DEV_CLEANUP(efm32_gpio_cleanup);

const struct driver_s efm32_gpio_drv =
  {
    .desc       = "EFM32 GPIO",
    .f_init     = efm32_gpio_init,
    .f_cleanup  = efm32_gpio_cleanup,
    .classes    = {
      &efm32_gpio_gpio_drv,
      &efm32_gpio_iomux_drv,
#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
      &efm32_gpio_icu_drv,
#endif
      NULL
    }
  };

REGISTER_DRIVER(efm32_gpio_drv);

static DEV_INIT(efm32_gpio_init)
{
  struct efm32_gpio_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_CLOCK
  /* enable clock */
  dev_clock_sink_init(dev, &pv->clk_ep, NULL);

  if (dev_clock_sink_link(dev, &pv->clk_ep, NULL, NULL, 0, 0))
    goto err_mem;

  if (dev_clock_sink_hold(&pv->clk_ep, NULL))
    goto err_clku;
#endif

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  device_irq_source_init(dev, pv->src, GPIO_SRC_IRQ_COUNT,
                    &efm32_gpio_source_process, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, pv->src, GPIO_SRC_IRQ_COUNT, 0))
    goto err_clk;

  device_irq_sink_init(dev, pv->sink, CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT,
                    DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE);
#endif

  dev->drv = &efm32_gpio_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#if 0
 err_unlink:
  device_irq_source_unlink(dev, pv->src, GPIO_SRC_IRQ_COUNT);
#endif
 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
#endif
 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_gpio_cleanup)
{
  struct efm32_gpio_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_EFM32_GPIO_ICU
  cpu_mem_write_32(pv->addr + EFM32_GPIO_IEN_ADDR, 0);

  device_irq_source_unlink(dev, pv->src, GPIO_SRC_IRQ_COUNT);
  device_irq_source_unlink(dev, pv->sink, CONFIG_DRIVER_EFM32_GPIO_IRQ_COUNT);
#endif

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  mem_free(pv);
}


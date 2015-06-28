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

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/gpio.h>
#include <device/class/iomux.h>
#include <device/class/icu.h>

#include <mutek/printk.h>
#include <string.h>

#include <arch/bcm2835_gpio.h>

#define GPIO_SRC_IRQ_COUNT 2
#define GPIO_BANK_SIZE 32
#define GPIO_IO_COUNT 54

struct bcm2835_gpio_private_s
{
  uintptr_t addr;

  uint64_t  pull_up;
  uint64_t  pull_down;

#ifdef CONFIG_DRIVER_BCM2835_GPIO_ICU
  uint32_t edge[GPIO_SRC_IRQ_COUNT];

  struct dev_irq_sink_s sink[GPIO_IO_COUNT];
  struct dev_irq_src_s src[GPIO_SRC_IRQ_COUNT];
#endif
};

/* This function returns a 30 bits mask from a 10 bits value for
   instance, get_msk(0b1010) returns 0b111000111000 */
static inline uint32_t bcm2835_gpio_get_mask(uint16_t msk)
{
  uint32_t p0 = (0x00011044 * (msk & 0x00a5)) & 0x00820104;
  uint32_t p1 = (0x00044110 * (msk & 0x014a)) & 0x04100820;
  uint32_t p2 = (0x00100400 * (msk & 0x0210)) & 0x20004000;

  uint32_t r = p0 | p1 | p2;

  r |= r >> 1;
  r |= r >> 1;

  return r;
}

/* This function waits for tcycles of peripheral timer */
static void bcm2835_gpio_delay(uint32_t tcycle)
{
  /* read timer value */
  uint32_t low = endian_le32(cpu_mem_read_32(0x20003004));

  while (endian_le32(cpu_mem_read_32(0x20003004)) - low < tcycle)
    ;
}

static error_t bcm2835_gpio_mode(struct bcm2835_gpio_private_s *pv,
                                 enum dev_pin_driving_e mode,
                                 uint64_t ckmsk, uint32_t *sel)
{
  uint32_t pull;

  switch (mode)
    {
    case DEV_PIN_DISABLED:
    case DEV_PIN_INPUT:
    case DEV_PIN_PUSHPULL:
      *sel = mode == DEV_PIN_PUSHPULL
        ? BCM2835_GPIO_GPFSEL_FSEL_OUTPUT
        : BCM2835_GPIO_GPFSEL_FSEL_INPUT;
      if (!((pv->pull_up | pv->pull_down) & ckmsk))
        return 0;
      pull = BCM2835_GPIO_GPPUD_PUD_OFF;
      pv->pull_up &= ~ckmsk;
      pv->pull_down &= ~ckmsk;
      break;
    case DEV_PIN_INPUT_PULLUP:
      *sel = BCM2835_GPIO_GPFSEL_FSEL_INPUT;
      if (!((~pv->pull_up | pv->pull_down) & ckmsk))
        return 0;
      pull = BCM2835_GPIO_GPPUD_PUD_PULLUP;
      pv->pull_up |= ckmsk;
      pv->pull_down &= ~ckmsk;
      break;
    case DEV_PIN_INPUT_PULLDOWN:
      *sel = BCM2835_GPIO_GPFSEL_FSEL_INPUT;
      if (!((pv->pull_up | ~pv->pull_down) & ckmsk))
        return 0;
      pull = BCM2835_GPIO_GPPUD_PUD_PULLDOWN;
      pv->pull_up &= ~ckmsk;
      pv->pull_down |= ckmsk;
      break;
    default:
      return -EINVAL;
    }

  /* Configure pull resistors */

  /* Set PUD register to PULL value*/
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPPUD_ADDR, endian_le32(pull));

  /* Wait for 150 cycles */
  bcm2835_gpio_delay(150);

  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPPUDCLK_ADDR(0), ckmsk);
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPPUDCLK_ADDR(1), ckmsk >> 32);

  /* Wait for 150 cycles */
  bcm2835_gpio_delay(150);

  /* Set PUD register to OFF */
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPPUD_ADDR, BCM2835_GPIO_GPPUD_PUD_OFF);
  /* Set PUDCLK register to 0 */
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPPUDCLK_ADDR(0), 0);
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPPUDCLK_ADDR(1), 0);

  return 0;
}

/********************** gpio controller driver part *********************/

static DEV_GPIO_SET_MODE(bcm2835_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;
  struct bcm2835_gpio_private_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  /* Retrieve the 64-bit mask */
  uint64_t msk = endian_le64_na_load(mask);

  int_fast8_t mlen = io_last - io_first + 1;
  uint32_t sel;

  err = bcm2835_gpio_mode(pv, mode, (msk & ((1ULL << mlen) - 1)) << io_first, &sel);
  if (err)
    goto err;

  /* Build a 32 bits mode value */
  sel *= 0x9249249;

  uint64_t tmask, m, mp = 0;
  uint_fast8_t shift = io_first % 10;

 tmask:
  /* compute mask word for next mask cell */
  tmask = mlen > 10 ? 0x3ff : ((1 << mlen) - 1);

 loop:
  m = bcm2835_gpio_get_mask(msk & tmask);
  mp = (m << (shift * 3)) | mp;
  msk >>= 10;
  mlen -= 10;

 last:;
  uintptr_t a = pv->addr + BCM2835_GPIO_GPFSEL_ADDR(io_first/10);
  uint32_t x = endian_le32(cpu_mem_read_32(a));
  x = (x & ~mp) | (mp & sel);
#ifdef BCM2835_GPIO_DEBUG
  printk("gpio mode: reg=%08x value=%08x\n", a, x);
#endif
  cpu_mem_write_32(a, endian_le32(x));

  mp >>= 30;

  io_first = io_first + 10 - (io_first%10);

  if (mlen >= 10)
    goto loop;   /* mask is still 0x3ff, no need to recompute */

  if (io_first <= io_last)
    {
      if (mlen < 0)
        goto last;   /* last remaining bits in next register, mask
                              bits already available */

      goto tmask;     /* need to compute new mask for the last word */
    }

 err:;
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_GPIO_SET_OUTPUT(bcm2835_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_IO_COUNT)
    return -EINVAL;

  LOCK_SPIN_IRQ(&dev->lock);

  struct bcm2835_gpio_private_s *pv = dev->drv_pv;

  int_fast8_t mlen = io_last - io_first + 1;

  /* compute mask word for clear_mask and set_mask */
  uint64_t tmask = (1ULL << mlen) - 1;

  /* compute set and clear masks */
  uint64_t cm = (~endian_le64_na_load(clear_mask) & tmask) << io_first;
  uint64_t sm = ( endian_le64_na_load(set_mask)   & tmask) << io_first;

  uint64_t tg = cm & sm;
  if (tg != 0)                  /* toggle, must read previous value */
    {
      uint64_t x = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_GPIO_GPLEV_ADDR(1)));
      x = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_GPIO_GPLEV_ADDR(0))) | (x << 32);
      x = sm ^ (x & (sm ^ ~cm));

      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPSET_ADDR(0), endian_le32(x));
      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPCLR_ADDR(0), endian_le32(~x));
      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPSET_ADDR(1), endian_le32(x >> 32));
      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPCLR_ADDR(1), endian_le32(~x >> 32));
    }
  else
    {
      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPSET_ADDR(0), endian_le32(sm));
      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPCLR_ADDR(0), endian_le32(cm));
      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPSET_ADDR(1), endian_le32(sm >> 32));
      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPCLR_ADDR(1), endian_le32(cm >> 32));
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_GET_INPUT(bcm2835_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_IO_COUNT)
    return -EINVAL;

  LOCK_SPIN_IRQ(&dev->lock);

  struct bcm2835_gpio_private_s *pv = dev->drv_pv;
  uint64_t v;

  v = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_GPIO_GPLEV_ADDR(1)));
  v <<= GPIO_BANK_SIZE;
  v |= endian_le32(cpu_mem_read_32(pv->addr + BCM2835_GPIO_GPLEV_ADDR(0)));
  v >>= io_first;

  endian_le64_na_store(data, v);

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

#define bcm2835_gpio_request dev_gpio_request_async_to_sync
#define bcm2835_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn

/********************** iomux controller driver part *********************/

#ifdef CONFIG_DEVICE_IOMUX

static DEV_IOMUX_SETUP(bcm2835_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;
  struct bcm2835_gpio_private_s *pv = dev->drv_pv;

  if (io_id >= GPIO_IO_COUNT)
    return -ERANGE;

  uint32_t sel;

  if (mux == BCM2835_GPIO_GPFSEL_FSEL_INPUT ||
      mux == BCM2835_GPIO_GPFSEL_FSEL_OUTPUT)
    {
      /* in/out */
      error_t err = bcm2835_gpio_mode(pv, dir, 1ULL << io_id, &sel);
      if (err)
        return err;
    }
  else
    {
      /* special function */
      sel = mux;
    }

  uintptr_t a = pv->addr + BCM2835_GPIO_GPFSEL_ADDR(io_id / 10);

  uint_fast8_t shift = (io_id % 10) * 3;
  uint32_t mp = 0x7 << shift;

  uint32_t x = endian_le32(cpu_mem_read_32(a));
  x = (x & ~mp) | (sel << shift);
  cpu_mem_write_32(a, endian_le32(x));

  return 0;
}

#endif

/********************** irq controller driver part *********************/

#ifdef CONFIG_DRIVER_BCM2835_GPIO_ICU

static DEV_ICU_GET_SINK(bcm2835_gpio_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct bcm2835_gpio_private_s *pv = dev->drv_pv;

  if (id > GPIO_IO_COUNT)
    return NULL;
  return pv->sink + id;
}

static void bcm2835_gpio_icu_dis_irq(struct bcm2835_gpio_private_s *pv, uint_fast8_t id)
{
  uint_fast8_t bank = id / GPIO_BANK_SIZE;
  uint32_t mask = endian_le32(~(1 << (id % GPIO_BANK_SIZE)));
  uintptr_t a;

  a = pv->addr + BCM2835_GPIO_GPREN_ADDR(bank);
  cpu_mem_write_32(a, cpu_mem_read_32(a) & mask);
  a = pv->addr + BCM2835_GPIO_GPFEN_ADDR(bank);
  cpu_mem_write_32(a, cpu_mem_read_32(a) & mask);
  a = pv->addr + BCM2835_GPIO_GPHEN_ADDR(bank);
  cpu_mem_write_32(a, cpu_mem_read_32(a) & mask);
  a = pv->addr + BCM2835_GPIO_GPLEN_ADDR(bank);
  cpu_mem_write_32(a, cpu_mem_read_32(a) & mask);
}

static void bcm2835_gpio_icu_disall_irq(struct bcm2835_gpio_private_s *pv)
{
  uint_fast8_t bank;

  for (bank = 0; bank < 2; bank++)
    {
      uintptr_t a;

      a = pv->addr + BCM2835_GPIO_GPREN_ADDR(bank);
      cpu_mem_write_32(a, 0);
      a = pv->addr + BCM2835_GPIO_GPFEN_ADDR(bank);
      cpu_mem_write_32(a, 0);
      a = pv->addr + BCM2835_GPIO_GPHEN_ADDR(bank);
      cpu_mem_write_32(a, 0);
      a = pv->addr + BCM2835_GPIO_GPLEN_ADDR(bank);
      cpu_mem_write_32(a, 0);

      cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPEDS_ADDR(bank), 0xffffffff);
    }
}

static DEV_IRQ_SINK_UPDATE(bcm2835_gpio_icu_sink_update)
{
  struct device_s *dev = sink->base.dev;
  struct bcm2835_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sink;

  uint8_t bank = sink_id / GPIO_BANK_SIZE;
  uint8_t line = sink_id % GPIO_BANK_SIZE;
  uintptr_t a;

  /* disable irq in all modes */
  bcm2835_gpio_icu_dis_irq(pv, sink_id);

  switch (sense)
    {
      case DEV_IRQ_SENSE_RISING_EDGE:
        a =  BCM2835_GPIO_GPREN_ADDR(bank);
        pv->edge[bank] |= 1 << line;
        break;
      case DEV_IRQ_SENSE_FALLING_EDGE:
        a =  BCM2835_GPIO_GPFEN_ADDR(bank);
        pv->edge[bank] |= 1 << line;
        break;
      case DEV_IRQ_SENSE_HIGH_LEVEL:
        a =  BCM2835_GPIO_GPHEN_ADDR(bank);
        pv->edge[bank] &= ~(1 << line);
        break;
      case DEV_IRQ_SENSE_LOW_LEVEL:
        a =  BCM2835_GPIO_GPLEN_ADDR(bank);
        pv->edge[bank] &= ~(1 << line);
        break;
      case DEV_IRQ_SENSE_NONE:
        return;
      default:
        return;
    }

  /* enable irq in requested sense mode */
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + a));
  x |= 1 << line;
  cpu_mem_write_32(pv->addr + a, endian_le32(x));
}

static DEV_ICU_LINK(bcm2835_gpio_icu_link)
{
  if (!route_mask || *bypass)
    return 0;

  struct device_s *dev = accessor->dev;
  struct bcm2835_gpio_private_s *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sink;

  uint8_t bank = sink_id / GPIO_BANK_SIZE;
  uint8_t line = sink_id % GPIO_BANK_SIZE;

  /* Change pin mode to input */
  uintptr_t a = BCM2835_GPIO_GPFSEL_ADDR(sink_id / 10);
  uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + a));
  BCM2835_GPIO_GPFSEL_FSEL_SET(line % 10, x, INPUT);
  cpu_mem_write_32(pv->addr + a, endian_le32(x));

  /* Clear interrupt */
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPEDS_ADDR(bank), endian_le32(1 << line));

  return 0;
}

static DEV_IRQ_SRC_PROCESS(bcm2835_gpio_source_process)
{

  struct bcm2835_gpio_private_s *pv = ep->base.dev->drv_pv;

  uint_fast8_t src_id = ep - pv->src;
  uint32_t x;

  if (src_id >= GPIO_SRC_IRQ_COUNT)
    return;

  while (1)
    {
      x = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_GPIO_GPEDS_ADDR(src_id)));

      if (!x)
        break;

      /* clear edge irqs */
      uint32_t e = x & pv->edge[src_id];
      if (e)
        cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPEDS_ADDR(src_id), endian_le32(e));

      while (x)
        {
          uint_fast8_t i = __builtin_ctz(x);
          struct dev_irq_sink_s *sink = pv->sink + i + GPIO_BANK_SIZE * src_id;
          device_irq_sink_process(sink, 0);
          x ^= 1 << i;
        }

      /* clear level irqs */
      e = x & ~pv->edge[src_id];
      if (e)
        cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPEDS_ADDR(src_id), endian_le32(e));
    }
}

#endif

/***********************************************************************/

static DEV_INIT(bcm2835_gpio_init);
static DEV_CLEANUP(bcm2835_gpio_cleanup);

#define bcm2835_gpio_use dev_use_generic

DRIVER_DECLARE(bcm2835_gpio_drv, "BCM2835 GPIO", bcm2835_gpio
               , DRIVER_GPIO_METHODS(bcm2835_gpio)
#ifdef CONFIG_DEVICE_IOMUX
               , DRIVER_IOMUX_METHODS(bcm2835_gpio_iomux)
#endif
#ifdef CONFIG_DRIVER_BCM2835_GPIO_ICU
               , DRIVER_ICU_METHODS(bcm2835_gpio_icu)
#endif
               );

DRIVER_REGISTER(bcm2835_gpio_drv);

static DEV_INIT(bcm2835_gpio_init)
{
  struct bcm2835_gpio_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DRIVER_BCM2835_GPIO_ICU
  /* Disable and clear all interrupts */
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPAREN_ADDR(0), 0);
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPAFEN_ADDR(0), 0);
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPAREN_ADDR(1), 0);
  cpu_mem_write_32(pv->addr + BCM2835_GPIO_GPAFEN_ADDR(1), 0);
  bcm2835_gpio_icu_disall_irq(pv);

  device_irq_source_init(dev, pv->src, GPIO_SRC_IRQ_COUNT,
                    &bcm2835_gpio_source_process);

  if (device_irq_source_link(dev, pv->src, GPIO_SRC_IRQ_COUNT, -1))
    goto err_mem;

  device_irq_sink_init(dev, pv->sink, GPIO_IO_COUNT, &bcm2835_gpio_icu_sink_update,
                    DEV_IRQ_SENSE_FALLING_EDGE | DEV_IRQ_SENSE_RISING_EDGE |
                    DEV_IRQ_SENSE_LOW_LEVEL | DEV_IRQ_SENSE_HIGH_LEVEL);

#endif

  dev->drv = &bcm2835_gpio_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_unlink:
#ifdef CONFIG_DRIVER_BCM2835_GPIO_ICU
  device_irq_source_unlink(dev, pv->src, GPIO_SRC_IRQ_COUNT);
#endif
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(bcm2835_gpio_cleanup)
{
  struct bcm2835_gpio_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_BCM2835_GPIO_ICU
  bcm2835_gpio_icu_disall_irq(pv);

  device_irq_source_unlink(dev, pv->src, GPIO_SRC_IRQ_COUNT);
#endif

  mem_free(pv);
}

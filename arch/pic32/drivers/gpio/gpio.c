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
#include <arch/pic32/gpio.h>
#include <arch/pic32/devaddr.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/gpio.h>
#include <device/class/iomux.h>
#include <device/class/icu.h>
#include <device/clock.h>

#include <mutek/printk.h>

#define GPIO_BANK_SIZE 16
#define GPIO_BANK_COUNT 10
#define GPIO_IRQ_PIN_COUNT GPIO_BANK_SIZE * GPIO_BANK_COUNT

DRIVER_PV(struct pic32_gpio_private_s
{
#ifdef CONFIG_DRIVER_PIC32_GPIO_ICU
  struct dev_irq_sink_s sinks[CONFIG_DRIVER_PIC32_GPIO_IRQ_COUNT];
  /* One irq by bank */
  struct dev_irq_src_s src[GPIO_BANK_COUNT];
  uint8_t               sinks_map[GPIO_IRQ_PIN_COUNT];
#endif
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s    clk_ep;
#endif
});

static void pic32_gpio_write_reg(uintptr_t ba, gpio_id_t io_first, gpio_id_t io_last,
                                 const uint8_t *mask)
{
  uint32_t m, tmask;
  uint32_t mp = 0;

  uint_fast8_t shift = io_first % GPIO_BANK_SIZE;
  int_fast8_t mlen  = io_last - io_first + 1;

mask:
  /* compute mask word for next clear_mask and set_mask */
  tmask = mlen > GPIO_BANK_SIZE ? 0xffff : ((1 << mlen) - 1);

loop:
  /* compute masks */
  m = (uint32_t)(endian_le16_na_load(mask) & tmask);
  mp = ((m << shift) | mp);
  mask += 2;

  mlen -= GPIO_BANK_SIZE;

last:;
     /* update register */
     uintptr_t a = PIC32_GPIO_ADDR + ba + (io_first / GPIO_BANK_SIZE) * 256; 

//     printk("gpio set : reg=%08x value=%08x\n", a, mp);
     cpu_mem_write_32(a, endian_le32(mp & ((1 << GPIO_BANK_SIZE) - 1)));

     mp >>= GPIO_BANK_SIZE;

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

static error_t pic32_gpio_mode(gpio_id_t io_first, gpio_id_t io_last,
                               const uint8_t *mask, enum dev_pin_driving_e mode)
{
  /* Digital I/O mode */
  pic32_gpio_write_reg(PIC32_GPIO_ANSEL_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);

  switch (mode)
    {
    case DEV_PIN_DISABLED:
    case DEV_PIN_INPUT:
      pic32_gpio_write_reg(PIC32_GPIO_TRIS_ADDR(0) + PIC32_SET_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPU_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPD_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      break;

    case DEV_PIN_PUSHPULL:
      pic32_gpio_write_reg(PIC32_GPIO_TRIS_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPU_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPD_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_ODC_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      break;

    case DEV_PIN_INPUT_PULLUP:
      pic32_gpio_write_reg(PIC32_GPIO_TRIS_ADDR(0) + PIC32_SET_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPU_ADDR(0) + PIC32_SET_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPD_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_ODC_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      break;

    case DEV_PIN_INPUT_PULLDOWN:
      pic32_gpio_write_reg(PIC32_GPIO_TRIS_ADDR(0) + PIC32_SET_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPU_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPD_ADDR(0) + PIC32_SET_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_ODC_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      break;

    case DEV_PIN_OPENDRAIN:
      pic32_gpio_write_reg(PIC32_GPIO_TRIS_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPU_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_CNPD_ADDR(0) + PIC32_CLR_OFF, io_first, io_last, mask);
      pic32_gpio_write_reg(PIC32_GPIO_ODC_ADDR(0) + PIC32_SET_OFF, io_first, io_last, mask);
      break;

    default:
      return -ENOTSUP;
    }
  return 0;
}

static DEV_GPIO_SET_MODE(pic32_gpio_set_mode)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * GPIO_BANK_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);
  pic32_gpio_mode(io_first, io_last, mask, mode);
  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_GPIO_SET_OUTPUT(pic32_gpio_set_output)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * GPIO_BANK_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

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
     /* update PORT register */
     uintptr_t a = PIC32_GPIO_ADDR + PIC32_GPIO_PORT_ADDR(io_first / GPIO_BANK_SIZE);
     uint32_t x = endian_le32(cpu_mem_read_32(a));
     x = smp ^ (x & (smp ^ ~cmp));

     cpu_mem_write_32(a, endian_le32(x & ((1 << GPIO_BANK_SIZE) - 1)));

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
 
static DEV_GPIO_GET_INPUT(pic32_gpio_get_input)
{
  struct device_s *dev = gpio->dev;

  if (io_last >= GPIO_BANK_SIZE * GPIO_BANK_COUNT)
    return -ERANGE;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t vp, v;
  uint_fast8_t bf = io_first / GPIO_BANK_SIZE;
  uint_fast8_t bl = io_last / GPIO_BANK_SIZE;
  uint_fast8_t shift = io_first % GPIO_BANK_SIZE;

  vp = endian_le32(cpu_mem_read_32(PIC32_GPIO_ADDR + PIC32_GPIO_PORT_ADDR(bf)));
  vp >>= shift;

  while (bf++ < bl)
  {
    v = endian_le32(cpu_mem_read_32(PIC32_GPIO_ADDR + PIC32_GPIO_PORT_ADDR(bf)));
    v = (v << (GPIO_BANK_SIZE - shift)) | vp ;
    vp = v >> GPIO_BANK_SIZE;

    endian_le16_na_store(data, endian_le16((uint16_t)v));
    data += 2;
  }

  endian_le16_na_store(data, endian_le16((uint16_t)vp));

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;

}

/******** GPIO iomux controller driver part *********************/

static const uint8_t pic32mz_input_mux[58] = {
  49, 50, 51, 57, 102, 103, 104, 105, 24, 30, 84, 85, 31, 48, 58, 59,
  52, 80, 81, -1, 16, 17, 22, 25, 26, 53, 67, 69, 18, 23, 45, 46,
  19, 21, 83, -1, 92, 93, -1, -1, 33, 36, 60, -1, 62, 63, 82, 88,
  34, 35, 96, 97, 14, 15, 72, 73, 54, 55, 
};

static DEV_IOMUX_SETUP(pic32_gpio_iomux_setup)
{
  struct device_s *dev = accessor->dev;

  if (io_id >= CONFIG_DRIVER_PIC32_GPIO_IRQ_COUNT)
    return -ERANGE;

  error_t err = 0; 

  LOCK_SPIN_IRQ(&dev->lock);

  switch (dir)
  {
    case DEV_PIN_INPUT:
    case DEV_PIN_INPUT_PULLUP:
    case DEV_PIN_INPUT_PULLDOWN: 
      /* For input only, PPS functionality does not have priority over TRISx settings.*/
      err = pic32_gpio_mode(io_id, io_id, dev_gpio_mask1, dir); 
      /* Retrieve mux value from pic32mz_input_mux */
      for (uint8_t i = 0; i < sizeof(pic32mz_input_mux); i++)
        {
          if (pic32mz_input_mux[i] == io_id)
            {
              cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_INPUT_MUX_OFFSET + mux, i >> 2);
              break;
            }
        }
      break;
    case DEV_PIN_PUSHPULL:
    case DEV_PIN_OPENDRAIN:
        err = pic32_gpio_mode(io_id, io_id, dev_gpio_mask1, dir); 
        cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_RP_ADDR(io_id), endian_le32(mux));
      break;
    default:
      err = -ENOTSUP;
      break;
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#define pic32_gpio_cancel (dev_gpio_cancel_t*)&dev_driver_notsup_fcn
#define pic32_gpio_request dev_gpio_request_async_to_sync
#define pic32_gpio_input_irq_range (dev_gpio_input_irq_range_t*)dev_driver_notsup_fcn
#define pic32_gpio_use dev_use_generic

/******** GPIO irq controller driver part *********************/

#ifdef CONFIG_DRIVER_PIC32_GPIO_ICU

static inline uint8_t icu_pv_get_pin_id(uint16_t icu_pv)
{
  return icu_pv & 0xff;
}

static inline void icu_pv_set_pin_id(uint16_t *icu_pv, uint8_t pin_id)
{
  *icu_pv &= ~0xff;
  *icu_pv |= pin_id;
}

static inline uint8_t icu_pv_get_irq_lvl_flag(uint16_t icu_pv)
{
  return icu_pv >> 8;
}

static inline void icu_pv_set_irq_lvl_flag(uint16_t *icu_pv, uint8_t flag)
{
  *icu_pv &= ~0xff00;
  *icu_pv |= (flag << 8);
}

static DEV_IRQ_SINK_UPDATE(pic32_gpio_icu_sink_update)
{
  uint8_t pin_id = icu_pv_get_pin_id(sink->icu_pv);

  uint8_t b = pin_id / GPIO_BANK_SIZE;
  uintptr_t a = PIC32_GPIO_ADDR + PIC32_GPIO_PORT_ADDR(b);
  uint32_t mask = 1 << (pin_id % GPIO_BANK_SIZE);

  icu_pv_set_irq_lvl_flag(&sink->icu_pv, 0);
  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE: {
      /* Disable external interrupt */
      cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_CNCON_ADDR(b) + PIC32_CLR_OFF, PIC32_GPIO_CNCON_ON);
      cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_CNEN_ADDR(b) + PIC32_CLR_OFF, endian_le32(mask));
      return;
    }

    case DEV_IRQ_SENSE_LOW_LEVEL:
      icu_pv_set_irq_lvl_flag(&sink->icu_pv, 1);
    case DEV_IRQ_SENSE_HIGH_LEVEL:{
      /* Rearm irq */
      __unused__ uint32_t x = endian_le32(cpu_mem_read_32(a));
      cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_CNCON_ADDR(b) + PIC32_SET_OFF, PIC32_GPIO_CNCON_ON);
      cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_CNEN_ADDR(b) + PIC32_SET_OFF, endian_le32(mask));
      break;
    }
    default:
      return;
    }
}

static DEV_ICU_GET_SINK(pic32_gpio_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct pic32_gpio_private_s *pv = dev->drv_pv;

  if (id >= GPIO_IRQ_PIN_COUNT)
    return NULL;

  for (uint_fast8_t i = 0; i < CONFIG_DRIVER_PIC32_GPIO_IRQ_COUNT; i++)
    {
      if (!pv->sinks[i].base.link_count)
        {
          pv->sinks_map[id] = i;
          icu_pv_set_pin_id(&pv->sinks[i].icu_pv, (uint8_t)id);
          return pv->sinks + i;
        }
    }

  return NULL;
}

static DEV_ICU_LINK(pic32_gpio_icu_link)
{
  if (!route_mask || *bypass)
    return 0;

  uint_fast8_t pin_id = icu_pv_get_pin_id(sink->icu_pv);

#ifdef CONFIG_DEVICE_IRQ_SHARING
  if (sink->base.link_count > 1)
    return 0;
#endif

  /* Change pin mode to input */
  pic32_gpio_mode(pin_id, pin_id, dev_gpio_mask1, DEV_PIN_INPUT); 

  /* Clear interrupt */
  uintptr_t a = PIC32_GPIO_ADDR + PIC32_GPIO_PORT_ADDR(pin_id / GPIO_BANK_SIZE);
  __unused__ uint32_t x = endian_le32(cpu_mem_read_32(a));

  return 0;
}

static DEV_IRQ_SRC_PROCESS(pic32_gpio_source_process)
{
  struct pic32_gpio_private_s *pv = ep->base.dev->drv_pv;

  uint_fast8_t src_id = ep - pv->src;
  uintptr_t a = PIC32_GPIO_ADDR + PIC32_GPIO_PORT_ADDR(src_id);
  uint32_t p;

  while (1)
    {
      uint32_t x = cpu_mem_read_32(PIC32_GPIO_ADDR + PIC32_GPIO_CNSTAT_ADDR(src_id));
      uint32_t msk = cpu_mem_read_32(PIC32_GPIO_ADDR + PIC32_GPIO_CNEN_ADDR(src_id));

      x &= msk;

      if (!x)
        break;

      x = endian_le32(x);

      while (x)
        {
          uint_fast8_t i = bit_ctz(x);
          struct dev_irq_sink_s *sink = pv->sinks + pv->sinks_map[i + src_id * GPIO_BANK_SIZE];

          do
            {
              device_irq_sink_process(sink, 0);
              p = (endian_le32(cpu_mem_read_32(a)) >> i) & 1; 
            } while(p ^ icu_pv_get_irq_lvl_flag(sink->icu_pv));

          x ^= 1 << i;
        }
    }
}

#endif

static DEV_INIT(pic32_gpio_init)
{
  struct pic32_gpio_private_s *pv;


  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (sizeof (*pv) && !pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

#ifdef CONFIG_DRIVER_PIC32_GPIO_ICU
  device_irq_source_init(dev, pv->src, GPIO_BANK_COUNT,
                    &pic32_gpio_source_process);

  if (device_irq_source_link(dev, pv->src, GPIO_BANK_COUNT, -1))
    goto err_mem;

  device_irq_sink_init(dev, pv->sinks, CONFIG_DRIVER_PIC32_GPIO_IRQ_COUNT,
                       &pic32_gpio_icu_sink_update,
                       DEV_IRQ_SENSE_LOW_LEVEL |
                       DEV_IRQ_SENSE_HIGH_LEVEL);

  memset(pv->sinks_map, 0, sizeof(*(pv->sinks_map)));
#endif


  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(pic32_gpio_cleanup)
{
  struct pic32_gpio_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_DRIVER_PIC32_GPIO_ICU
  device_irq_source_unlink(dev, pv->src, GPIO_BANK_COUNT);
  for (uint_fast8_t i = 0; i < CONFIG_DRIVER_PIC32_GPIO_IRQ_COUNT; i++)
    {
      if (!pv->sinks[i].base.link_count)
        {
          __unused__ uint8_t pin_id = icu_pv_get_pin_id(pv->sinks[i].icu_pv);
          /* TODO : disable external interrupt for pin_id */
        }
    }
#endif
  mem_free(pv);
}

DRIVER_DECLARE(pic32_gpio_drv, 0, "PIC32 GPIO", pic32_gpio,
               DRIVER_GPIO_METHODS(pic32_gpio),
#ifdef CONFIG_DRIVER_PIC32_GPIO_ICU
               DRIVER_ICU_METHODS(pic32_gpio_icu),
#endif
               DRIVER_IOMUX_METHODS(pic32_gpio_iomux));

DRIVER_REGISTER(pic32_gpio_drv);


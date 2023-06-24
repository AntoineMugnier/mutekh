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
    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2023
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>
#include <device/class/iomux.h>
#include <device/resource/uart.h>
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
# include <device/class/valio.h>
# include <device/valio/uart_config.h>
#endif

#include "pl011.h"

/**************************************************************/

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
#include <gct_platform.h>
#include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_uart_fifo RING
GCT_CONTAINER_TYPES(uart_fifo, uint8_t, CONFIG_DRIVER_ARM_PL011_UART_SWFIFO);
GCT_CONTAINER_FCNS(uart_fifo, static inline, uart_fifo,
                   init, destroy, isempty, pop, pop_array, pushback, pushback_array);
#endif

struct pl011_uart_pv_s
{
  uintptr_t addr;

  /* tty input request queue and char fifo */
  dev_request_queue_root_t	read_q;
  dev_request_queue_root_t	write_q;
#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
  uart_fifo_root_t		read_fifo;
  uart_fifo_root_t		write_fifo;
#endif

  struct dev_clock_sink_ep_s clk_ep;
  struct dev_freq_s freq;
  uint32_t baudrate;

  struct dev_irq_src_s           irq_ep;

  bool_t                        BITFIELD(read_started,1);
  bool_t                        BITFIELD(write_started,1);
};

DRIVER_PV(pl011_uart_pv_s);

static bool_t pl011_uart_try_read(struct device_s *dev)
{
  struct pl011_uart_pv_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;
  bool_t ack_done = 0;

  while ((rq = dev_char_rq_head(&pv->read_q)))
    {
      size_t size = 0;

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
      /* read as many characters as possible from driver fifo */
      size = uart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

      /* try to read more characters directly from device fifo */
      while ( size < rq->size && !(cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
                                   & endian_le32(PL011_FR_RXFE)) )
        {
          rq->data[size++] = PL011_DR_DATA_GET(endian_le32(cpu_mem_read_32(pv->addr + PL011_DR_ADDR)));
          ack_done = 1;
        }

      if (size)
        {
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_char_rq_pop(&pv->read_q);
              dev_char_rq_done(rq);
              continue;
            }
        }

      /* more data will be available on next interrupt */
      return ack_done;
    }

  pv->read_started = 0;

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
  /* copy more data from device fifo to driver fifo if no request currently need it */
  while (!(cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
           & endian_le32(PL011_FR_RXFE)))
    {
      uart_fifo_pushback(&pv->read_fifo, PL011_DR_DATA_GET(endian_le32(cpu_mem_read_32(pv->addr + PL011_DR_ADDR))));
      ack_done = 1;
    }
#endif

  return ack_done;
}

static void pl011_uart_try_write(struct device_s *dev)
{
  struct pl011_uart_pv_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
  /* try to write data from driver fifo to device */
  while (!uart_fifo_isempty(&pv->write_fifo) &&
         !(cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
           & endian_le32(PL011_FR_TXFF)) )
    {
      uint8_t c = uart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + PL011_DR_ADDR, endian_le32(PL011_DR_DATA(c)));
    }
#endif

  while ((rq = dev_char_rq_head(&pv->write_q)))
    {
      size_t size = 0;

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
      if (uart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, try to write as many characters as
             possible from request directly to the device fifo */
#endif
          while (size < rq->size)
            {
              if (cpu_mem_read_32(pv->addr + PL011_FR_ADDR)
                   & endian_le32(PL011_FR_TXFF))
                break;

              cpu_mem_write_32(pv->addr + PL011_DR_ADDR, endian_le32(PL011_DR_DATA(rq->data[size++])));
            }

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
        }

      /* some characters were not written to the device fifo, push to driver fifo */
      if (size < rq->size)
          size += uart_fifo_pushback_array(&pv->write_fifo, rq->data + size, rq->size - size);
#endif

      if (size)
        {
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_char_rq_pop(&pv->write_q);
              dev_char_rq_done(rq);
              continue;
            }
        }

      /* more fifo space will be available on next interrupt */
      return;
    }

  pv->write_started = 0;
}

static
void pl011_uart_rate_update(struct pl011_uart_pv_s *pv)
{
  // Divisor is freq/(16*baudrate) in a f16.6 format. This acutally
  // means we need to calculate freq/baudrate in f12.10 first, then
  // select parts we need for various registers. To do rounding, add
  // one more bit -> f12.11.
  //
  // Multiplying base freq by 2048 would overflow with any clock above
  // 2M. We do not want this.  Note 9600 = 75x128, so we can divide
  // most baudrates by 128 and still loose no precision.
  //
  // Here max input freq will be ~260M, should be enough.
  uint32_t div_f12_11 = (uint32_t)(pv->freq.num * 16)
    / (uint32_t)(pv->freq.denom * pv->baudrate / 128);
  uint32_t div_f12_10 = (div_f12_11 + 1) / 2;

  logk_debug("New divisor, freq %d, br %d, div %d + %d/64",
             (uint32_t)(pv->freq.num / pv->freq.denom),
             pv->baudrate,
             div_f12_10 >> 6,
             div_f12_10 & 0x3f);

  cpu_mem_write_32(pv->addr + PL011_IBRD_ADDR, endian_le32(div_f12_10 >> 6));
  cpu_mem_write_32(pv->addr + PL011_FBRD_ADDR, endian_le32(div_f12_10 & 0x3f));
}

#define pl011_uart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static DEV_CHAR_REQUEST(pl011_uart_request)
{
  struct device_s               *dev = accessor->dev;
  struct pl011_uart_pv_s	*pv = dev->drv_pv;
  error_t err = 0;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ: {
      dev_char_rq_pushback(&pv->read_q, rq);
      if (!pv->read_started)
        {
          pv->read_started = 1;
          pl011_uart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_char_rq_pushback(&pv->write_q, rq);
      if (!pv->write_started)
        {
          pv->write_started = 1;
          pl011_uart_try_write(dev);
        }
      break;
    }
    default:
      err = -ENOTSUP;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (err)
    {
      rq->error = err;
      dev_char_rq_done(rq);
    }
}

static
error_t pl011_uart_config_set(struct pl011_uart_pv_s *pv,
                              struct dev_uart_config_s *cfg)
{
  uint32_t lcrh = 0;
  uint32_t config = endian_le32(cpu_mem_read_32(pv->addr + PL011_CR_ADDR));

  lcrh |= PL011_LCRH_FEN;

  if (cfg->baudrate > 10000000)
    return -ENOTSUP;

  if (cfg->flow_ctrl)
    return -ENOTSUP;

  switch (cfg->data_bits) {
  case 7:
    lcrh |= PL011_LCRH_WLEN(PL011_LCRH_WLEN_7_BITS);
    break;

  case 8:
    lcrh |= PL011_LCRH_WLEN(PL011_LCRH_WLEN_8_BITS);
    break;

  default:
    return -ENOTSUP;
  }

  switch (cfg->parity) {
  case DEV_UART_PARITY_NONE:
    break;
  case DEV_UART_PARITY_EVEN:
    lcrh |= PL011_LCRH_WLEN(PL011_LCRH_PEN);
    lcrh |= PL011_LCRH_WLEN(PL011_LCRH_EPS);
    break;
  case DEV_UART_PARITY_ODD:
    lcrh |= PL011_LCRH_WLEN(PL011_LCRH_PEN);
    break;
  }

  // Now we are certain to be able to set config
  pv->baudrate = cfg->baudrate;

  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, endian_le32(config & ~PL011_CR_UARTEN;));
  cpu_mem_write_32(pv->addr + PL011_LCRH_ADDR, endian_le32(lcrh));
  pl011_uart_rate_update(pv);
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, endian_le32(config));

  return 0;
}

static DEV_IRQ_SRC_PROCESS(pl011_uart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct pl011_uart_pv_s	*pv = dev->drv_pv;

  LOCK_SPIN_SCOPED(&dev->lock);

  while (1)
    {
      uint32_t ir = cpu_mem_read_32(pv->addr + PL011_MIS_ADDR);

      if (!ir)
        break;

      if (ir & endian_le32(PL011_MIS_TXMIS))
        {
          pl011_try_write(dev);
          cpu_mem_write_32(pv->addr + PL011_ICR_ADDR, endian_le32(PL011_MIS_TXMIS));
        }

      if (ir & endian_le32(PL011_MIS_RXMIS))
        {
          if (!pl011_try_read(dev))
            /* discard 1 byte to acknowledge RX fifo level irq */
            cpu_mem_read_32(pv->addr + PL011_DR_ADDR);
        }
      else if (ir & endian_le32(PL011_MIS_RTMIS))
        {
          if (!pl011_try_read(dev))
            cpu_mem_write_32(pv->addr + PL011_ICR_ADDR, endian_le32(PL011_MIS_RTMIS));
        }
    }
}

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)

static DEV_VALIO_REQUEST(pl011_uart_valio_request)
{
  struct device_s *dev = accessor->dev;
  struct pl011_uart_pv_s *pv = dev->drv_pv;

  if (rq->type != DEVICE_VALIO_WRITE
      || rq->attribute != VALIO_UART_CONFIG) {
    rq->error = -ENOTSUP;
  } else {
    LOCK_SPIN_IRQ_SCOPED(&dev->lock);

    rq->error = pl011_uart_config_set(pv, rq->data);
  }

  dev_valio_rq_done(rq);
}

#define pl011_uart_valio_cancel (dev_valio_cancel_t*)dev_driver_notsup_fcn
#endif

static DEV_USE(linflex_uart_use)
{
    switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
        struct dev_clock_notify_s *chg = param;
        struct dev_clock_sink_ep_s *sink = chg->sink;
        struct device_s *dev = sink->dev;
        struct pl011_uart_pv_s *pv = dev->drv_pv;

        LOCK_SPIN_IRQ_SCOPED(&dev->lock);

        pv->freq = chg->freq;
        uint32_t config = endian_le32(cpu_mem_read_32(pv->addr + PL011_CR_ADDR));
        cpu_mem_write_32(pv->addr + PL011_CR_ADDR, endian_le32(config & ~PL011_CR_UARTEN;));
        pl011_uart_rate_update(pv);
        cpu_mem_write_32(pv->addr + PL011_CR_ADDR, endian_le32(config));
        pl011_uart_rate_update(pv);

        return 0;
    }
#endif

    default:
        return dev_use_generic(param, op);
    }
}

static DEV_INIT(pl011_uart_init)
{
  struct pl011_uart_pv_s	*pv;
  error_t err;
  struct dev_resource_s *r;
  struct dev_uart_config_s config = {
    .baudrate = 115200,
    .data_bits = 8,
    .stop_bits = 1,
    .parity = DEV_UART_PARITY_NONE,
    .flow_ctrl = 0,
  };

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_IOMUX
  /* setup pinmux */
  if (device_iomux_setup(dev, "<rx? >tx?", NULL, NULL, NULL))
    goto err_mem;
#endif

  err = dev_drv_clock_init(dev, &pv->clk_ep, 0, 0
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
                           | DEV_CLOCK_EP_FREQ_NOTIFY
#endif
                           | DEV_CLOCK_EP_POWER_CLOCK
                           | DEV_CLOCK_EP_GATING_SYNC, &pv->freq);
  if (err)
    goto err_mem;

  r = device_res_get(dev, DEV_RES_UART, 0);
  if (r) {
    config.baudrate    = r->u.uart.baudrate;
    config.data_bits   = r->u.uart.data_bits;
    config.stop_bits   = r->u.uart.stop_bits;
    config.parity      = r->u.uart.parity;
    config.flow_ctrl   = r->u.uart.flow_ctrl;
  }

  pv->read_started = pv->write_started = 0;

  /* disable the uart */
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, 0);
  cpu_mem_write_32(pv->addr + PL011_DMACR_ADDR, 0);
  cpu_mem_write_32(pv->addr + PL011_LCRH_ADDR, 0);

  dev_rq_queue_init(&pv->read_q);
  dev_rq_queue_init(&pv->write_q);

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
  uart_fifo_init(&pv->read_fifo);
#endif

# if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
  uart_fifo_init(&pv->write_fifo);
# endif

  device_irq_source_init(dev, &pv->irq_ep, 1, &pl011_irq);
  err = device_irq_source_link(dev, &pv->irq_ep, 1, 1);
  if (err)
    goto err_fifo;
    
  /* configure uart */
  cpu_mem_write_32(pv->addr + PL011_IFLS_ADDR,
                   endian_le32(PL011_IFLS_TXIFLSEL(PL011_IFLS_TXIFLSEL_1_8) |
                               PL011_IFLS_RXIFLSEL(PL011_IFLS_RXIFLSEL_7_8)));

  err = pl011_uart_config_set(pv, &config);
  if (err)
    goto err_fifo;

  /* enable irqs */
  cpu_mem_write_32(pv->addr + PL011_IMSC_ADDR, endian_le32(PL011_IMSC_RXIM | PL011_IMSC_TXIM | PL011_IMSC_RTIM));

  /* enable the uart */
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, endian_le32(PL011_CR_TXE | PL011_CR_RXE | PL011_CR_UARTEN));


  return 0;

 err_fifo:
# if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
# endif
  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);
 err_mem:
  mem_free(pv);

  return err;
}

static DEV_CLEANUP(pl011_uart_cleanup)
{
  struct pl011_uart_pv_s	*pv = dev->drv_pv;

  /* disable irqs */
  cpu_mem_write_32(pv->addr + PL011_IMSC_ADDR, 0);

  /* disable the uart */
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, 0);

#if CONFIG_DRIVER_ARM_PL011_UART_SWFIFO > 0
  uart_fifo_destroy(&pv->write_fifo);
  uart_fifo_destroy(&pv->read_fifo);
#endif

  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(pl011_uart_drv, 0, "ARM PL011 UART", pl011_uart,
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
               DRIVER_VALIO_METHODS(pl011_uart_valio),
#endif
               DRIVER_CHAR_METHODS(pl011_uart));

DRIVER_REGISTER(pl011_uart_drv,
                DEV_ENUM_FDTNAME_ENTRY("arm,pl011"));


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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/char.h>
#include <device/class/iomux.h>
#include <device/resource/uart.h>
#include <device/class/valio.h>
#include <device/valio/uart_config.h>
#include <device/resource/uart.h>

#include <arch/stm32/usart.h>

#if CONFIG_STM32_FAMILY == L4
# define STM32_USART_SR_ADDR        STM32_USART_ISR_ADDR
# define STM32_USART_SR_RXNE        STM32_USART_ISR_RXNE
# define STM32_USART_SR_TXE         STM32_USART_ISR_TXE
# define STM32_USART_SR_RXNE_GET    STM32_USART_ISR_RXNE_GET
# define STM32_USART_SR_TXE_GET     STM32_USART_ISR_TXE_GET
# define STM32_USART_RDR_DATA_GET   STM32_USART_RDR_RDR_GET
# define STM32_USART_TDR_DATA_SET   STM32_USART_TDR_TDR_SET
#else
# define STM32_USART_RDR_ADDR       STM32_USART_DR_ADDR
# define STM32_USART_TDR_ADDR       STM32_USART_DR_ADDR
# define STM32_USART_RDR_DATA_GET   STM32_USART_DR_DATA_GET
# define STM32_USART_TDR_DATA_SET   STM32_USART_DR_DATA_SET
#endif


#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
# include <gct_platform.h>
# include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_usart_fifo RING
GCT_CONTAINER_TYPES(usart_fifo, uint8_t, CONFIG_DRIVER_STM32_USART_SWFIFO);
GCT_CONTAINER_FCNS(usart_fifo, static inline, usart_fifo,
                   init, destroy, pop_array, pushback, pushback_array, isempty, pop);
#endif

#if CONFIG_DEVICE_START_LOG2INC < 2
# error cannot use start bits to manage device state
#endif

enum stm32_usart_start_e
{
  STM32_USART_STARTED_READ  = 1,
  STM32_USART_STARTED_WRITE = 2
};

DRIVER_PV(struct stm32_usart_context_s
{
  /* usart controller address. */
  uintptr_t             addr;

  /* usart request queue for read requests. */
  dev_request_queue_root_t read_q;

  /* usart request queue for write requests. */
  dev_request_queue_root_t write_q;

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  /* read fifo for incoming data. */
  usart_fifo_root_t     read_fifo;

#if defined(CONFIG_DEVICE_IRQ)
  /* write fifo for outgoing data (only necessary in interruptible mode). */
  usart_fifo_root_t     write_fifo;
#endif
#endif

  /* interrupt endpoints (TX and RX on the same wire). */
  struct dev_irq_src_s   irq_ep[1];

#if !defined(CONFIG_DEVICE_CLOCK)
  struct dev_freq_s     busfreq;
#endif
});


static
void stm32_usart_try_read(struct device_s *dev)
{
  struct stm32_usart_context_s *pv = dev->drv_pv;
  struct dev_char_rq_s         *rq;

  uint32_t a, x;

  while ((rq = dev_char_rq_head(&pv->read_q)))
    {
      size_t size = 0;

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
      /* read as much as possible from fifo. */
      size = usart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

      a = pv->addr + STM32_USART_SR_ADDR;
      x = endian_le32(cpu_mem_read_32(a));

    /* read characters if the request asked for more. */
      if (size < rq->size && STM32_USART_SR_RXNE_GET(x))
        {
          a = pv->addr + STM32_USART_RDR_ADDR;
          x = endian_le32(cpu_mem_read_32(a));
          rq->data[size++] = STM32_USART_RDR_DATA_GET(x);
        }

      /* if a data was read, then process the read request. */
      if (size)
        {
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_char_rq_pop(&pv->read_q);
              dev_char_rq_done(rq);
              /* look for another pending read request. */
              continue;
            }
        }

#if defined(CONFIG_DEVICE_IRQ)
      /* otherwise, return and wait for another interrupt. */
      return;
#endif
    }

  dev->start_count &= ~STM32_USART_STARTED_READ;

  /* if no request need the data, discard it or save it in the read fifo. */
  a = pv->addr + STM32_USART_SR_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  if (STM32_USART_SR_RXNE_GET(x))
    {
      a = pv->addr + STM32_USART_RDR_ADDR;
      x = endian_le32(cpu_mem_read_32(a));
      __unused__ uint8_t c = STM32_USART_RDR_DATA_GET(x);
#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
      usart_fifo_pushback(&pv->read_fifo, c);
#endif
    }
}

static
void stm32_usart_try_write(struct device_s *dev)
{
  struct stm32_usart_context_s *pv = dev->drv_pv;
  struct dev_char_rq_s         *rq;

  uint32_t a, x;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  /* try to write as much as possible data from the fifo first. */
  a = pv->addr + STM32_USART_SR_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  while (!usart_fifo_isempty(&pv->write_fifo) && STM32_USART_SR_TXE_GET(x))
    {
      uint8_t c = usart_fifo_pop(&pv->write_fifo);
      cpu_mem_write_32(pv->addr + STM32_USART_TDR_ADDR, endian_le32(c));
      x = endian_le32(cpu_mem_read_32(a));
    }
#endif

  while ((rq = dev_char_rq_head(&pv->write_q)))
    {
      size_t size = 0;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_STM32_USART_SWFIFO > 0
      if (usart_fifo_isempty(&pv->write_fifo))
        {
          /* driver fifo is empty, so try to write as many characters as
             possible from request directly. */
#endif

          /* write data if some are pending and the controller is ready for
             it. */
          a = pv->addr + STM32_USART_SR_ADDR;
          x = endian_le32(cpu_mem_read_32(a));
          if (size < rq->size && STM32_USART_SR_TXE_GET(x))
            cpu_mem_write_32(pv->addr + STM32_USART_TDR_ADDR, endian_le32(rq->data[size++]));

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_STM32_USART_SWFIFO > 0
        }

      if (size < rq->size)
        {
          size += usart_fifo_pushback_array(&pv->write_fifo, &rq->data[size],
            rq->size - size);

          /* wait for the next write interrupt as the fifo is not empty
             anymore. */
          a = pv->addr + STM32_USART_CR1_ADDR;
          x = endian_le32(cpu_mem_read_32(a));
          STM32_USART_CR1_TXEIE_SET(x, 1);
          cpu_mem_write_32(a, endian_le32(x));
        }
#endif

      if (size)
        {
          /* update request size and data pointer. */
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_char_rq_pop(&pv->write_q);
              dev_char_rq_done(rq);
              /* look for another pending write request. */
              continue;
            }
        }

#if defined(CONFIG_DEVICE_IRQ)
      /* wait for the next interrupt, when the controller will be ready to
         send. */
      a = pv->addr + STM32_USART_CR1_ADDR;
      x = endian_le32(cpu_mem_read_32(a));
      STM32_USART_CR1_TXEIE_SET(x, 1);
      cpu_mem_write_32(a, endian_le32(x));
      return;
#endif
    }

  dev->start_count &= ~STM32_USART_STARTED_WRITE;

#if defined(CONFIG_DEVICE_IRQ)
  /* if there is no more write request in the queue or fifo, then
     disable TX interrupt. */
  a = pv->addr + STM32_USART_CR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  if (usart_fifo_isempty(&pv->write_fifo))
# endif
  STM32_USART_CR1_TXEIE_SET(x, 0);
# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  else
    STM32_USART_CR1_TXEIE_SET(x, 1);
#endif
  cpu_mem_write_32(a, endian_le32(x));
#endif
}

#define stm32_usart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static
DEV_CHAR_REQUEST(stm32_usart_request)
{
  struct device_s              *dev = accessor->dev;
  struct stm32_usart_context_s *pv  = dev->drv_pv;
  error_t err = 0;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
  {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ: {
      dev_char_rq_pushback(&pv->read_q, rq);
      if (!(dev->start_count & STM32_USART_STARTED_READ))
        {
          dev->start_count |= STM32_USART_STARTED_READ;
          stm32_usart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_char_rq_pushback(&pv->write_q, rq);
      if (!(dev->start_count & STM32_USART_STARTED_WRITE))
        {
          dev->start_count |= STM32_USART_STARTED_WRITE;
          stm32_usart_try_write(dev);
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

#if defined(CONFIG_DEVICE_IRQ)

static
DEV_IRQ_SRC_PROCESS(stm32_usart_irq)
{
  struct device_s              *dev = ep->base.dev;
  struct stm32_usart_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
  {
    uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + STM32_USART_SR_ADDR));

    /* if the controller has no pending data and cannot send data, then
     * break and wait for another interrupt.
     */
    if (!(x & ( STM32_USART_SR_RXNE | STM32_USART_SR_TXE )))
      break;

    if (x & STM32_USART_SR_TXE)
      stm32_usart_try_write(dev);

    if (x & STM32_USART_SR_RXNE)
      stm32_usart_try_read(dev);

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
    if (usart_fifo_isempty(&pv->write_fifo))
#else
    if (dev_rq_queue_isempty(&pv->write_q))
#endif
      break;
  }

  lock_release(&dev->lock);
}

#endif

static
error_t stm32_usart_check_config(struct dev_uart_config_s *cfg)
{
  /* check data bits. */
  switch (cfg->data_bits)
    {
    default:
      return -ENOTSUP;

#if CONFIG_STM32_FAMILY == L4
    case 7:
#endif
    case 8:
    case 9:
      break;
    }

  /* check stop bits (all supported). */

  /* check parity (all supported). */

  /* check flow control (not supported). */
  if (cfg->flow_ctrl)
    return -ENOTSUP;

  return 0;
}

static
error_t stm32_usart_config_simple(struct stm32_usart_context_s *pv,
                                  struct dev_uart_config_s *cfg)
{
  uint32_t a, x;

  /* check baudrate. */
  error_t err = stm32_usart_check_config(cfg);
  if (err)
    return err;

  /* configure data, stop and parity. */
  a = pv->addr + STM32_USART_CR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  switch (cfg->data_bits)
  {
  default:
    break;

#if CONFIG_STM32_FAMILY == L4
  case 7:
    STM32_USART_CR1_M0_SET(x, 0);
    STM32_USART_CR1_M1_SET(x, 1);
    break;
#endif

  case 8:
#if CONFIG_STM32_FAMILY == L4
    STM32_USART_CR1_M0_SET(x, 0);
    STM32_USART_CR1_M1_SET(x, 0);
#else
    STM32_USART_CR1_M_SET(x, 0);
#endif
    break;

  case 9:
#if CONFIG_STM32_FAMILY == L4
    STM32_USART_CR1_M0_SET(x, 1);
    STM32_USART_CR1_M1_SET(x, 0);
#else
    STM32_USART_CR1_M_SET(x, 1);
#endif
    break;
  }
  cpu_mem_write_32(a, endian_le32(x));

  a = pv->addr + STM32_USART_CR2_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  switch (cfg->stop_bits)
  {
  default:
    break;

  case 1:
    STM32_USART_CR2_STOP_SET(x, 0);
    break;

  case 2:
    STM32_USART_CR2_STOP_SET(x, 2);
    break;
  }
  cpu_mem_write_32(a, endian_le32(x));

  a = pv->addr + STM32_USART_CR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  switch (cfg->parity)
  {
  default:
    break;

  case DEV_UART_PARITY_NONE:
    STM32_USART_CR1_PCE_SET(x, 0);
    STM32_USART_CR1_PS_SET(x, 0);
    break;

  case DEV_UART_PARITY_ODD:
    STM32_USART_CR1_PCE_SET(x, 1);
    STM32_USART_CR1_PS_SET(x, 1);
    break;

  case DEV_UART_PARITY_EVEN:
    STM32_USART_CR1_PCE_SET(x, 1);
    STM32_USART_CR1_PS_SET(x, 0);
    break;
  }
  cpu_mem_write_32(a, endian_le32(x));

  uint64_t brr = 0;
#if !defined(CONFIG_DEVICE_CLOCK)
  /* configure the baudrate. */
  brr = (uint64_t)pv->busfreq.num / cfg->baudrate / pv->busfreq.denom;
#endif
  cpu_mem_write_32(pv->addr + STM32_USART_BRR_ADDR, endian_le32(brr));
  return 0;
}

static
DEV_VALIO_REQUEST(stm32_usart_valio_request)
{
  struct device_s *dev = accessor->dev;
  struct stm32_usart_context_s *pv = dev->drv_pv;

  if (rq->type != DEVICE_VALIO_WRITE
      || rq->attribute != VALIO_UART_CONFIG) {
    rq->error = -ENOTSUP;
    dev_valio_rq_done(rq);
  }

  struct dev_uart_config_s *cfg = rq->data;

  /* disable the usart. */
  uint32_t a = pv->addr + STM32_USART_CR1_ADDR;
  uint32_t x = endian_le32(cpu_mem_read_32(a));
  bool_t enabled = STM32_USART_CR1_UE_GET(x);

  STM32_USART_CR1_UE_SET(x, 0);
  cpu_mem_write_32(a, endian_le32(x));

  rq->error = stm32_usart_config_simple(pv, cfg);

  /* (re-)enable the usart. */
  if (!rq->error && enabled)
    {
      x = endian_le32(cpu_mem_read_32(a));
      STM32_USART_CR1_UE_SET(x, 1);
      cpu_mem_write_32(a, endian_le32(x));
    }

#if defined(CONFIG_DEBUG)
  if (rq->error && enabled)
    printk("uart: configuration left unchanged.\n");
#endif

  dev_valio_rq_done(rq);
}

#define stm32_usart_valio_cancel (dev_valio_cancel_t*)dev_driver_notsup_fcn


#define stm32_usart_use dev_use_generic

/* ************************************************************************* */

static DEV_INIT(stm32_usart_init)
{
  struct stm32_usart_context_s *pv;


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#if !defined(CONFIG_DEVICE_CLOCK)
  if (device_get_res_freq(dev, &pv->busfreq, 0))
#endif
    goto err_mem;

  /* disable and reset the usart. */
  cpu_mem_write_32(pv->addr + STM32_USART_CR1_ADDR, 0);
  cpu_mem_write_32(pv->addr + STM32_USART_CR2_ADDR, 0);
  cpu_mem_write_32(pv->addr + STM32_USART_CR3_ADDR, 0);

  /* initialize request queues. */
  dev_rq_queue_init(&pv->read_q);
  dev_rq_queue_init(&pv->write_q);

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_init(&pv->read_fifo);
# if defined(CONFIG_DEVICE_IRQ)
  usart_fifo_init(&pv->write_fifo);
# endif
#endif

  /* configure gpio. */
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, "<rx? >tx?", loc, NULL, NULL))
    goto err_fifo;

  if (loc[0] == IOMUX_INVALID_MUX && loc[1] == IOMUX_INVALID_MUX)
    goto err_fifo;

  uint32_t a = pv->addr + STM32_USART_CR1_ADDR;
  uint32_t x = endian_le32(cpu_mem_read_32(a));
  if (loc[0] != IOMUX_INVALID_MUX)
    STM32_USART_CR1_RE_SET(x, 1);
  if (loc[1] != IOMUX_INVALID_MUX)
    STM32_USART_CR1_TE_SET(x, 1);
  cpu_mem_write_32(a, endian_le32(x));

  /* configure over-sampling by 16. */
#if CONFIG_STM32_FAMILY == 4 || CONFIG_STM32_FAMILY == L4
  STM32_USART_CR1_OVER8_SET(x, 0);
  cpu_mem_write_32(a, endian_le32(x));

  a = pv->addr + STM32_USART_CR3_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_USART_CR3_ONEBIT_SET(x, 0);
  cpu_mem_write_32(a, endian_le32(x));
#endif

  /* check for default configuration resource. */
  struct dev_uart_config_s cfg;
  if (!device_get_res_uart(dev, &cfg) &&
      stm32_usart_config_simple(pv, &cfg))
    goto err_fifo;

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_init(dev, pv->irq_ep, 1, &stm32_usart_irq);
  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto err_fifo;

  /* enable RX interrupt if RX is wired. */
  a = pv->addr + STM32_USART_CR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_USART_CR1_RXNEIE_SET(x, 1);
  cpu_mem_write_32(a, endian_le32(x));
#endif

  /* enable usart. */
  a = pv->addr + STM32_USART_CR1_ADDR;
  x = endian_le32(cpu_mem_read_32(a));
  STM32_USART_CR1_UE_SET(x, 1);
  cpu_mem_write_32(a, endian_le32(x));

  /* link the driver. */
  dev->drv_pv = pv;

  return 0;

err_fifo:
#if defined(CONFIG_DEVICE_IRQ)
# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_destroy(&pv->read_fifo);
  usart_fifo_destroy(&pv->write_fifo);
# endif
  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);
#endif

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32_usart_cleanup)
{
  struct stm32_usart_context_s *pv = dev->drv_pv;

  if (dev->start_count & (STM32_USART_STARTED_READ | STM32_USART_STARTED_WRITE))
    return -EBUSY;

  /* disable and reset the usart. */
  cpu_mem_write_32(pv->addr + STM32_USART_CR1_ADDR, 0);
  cpu_mem_write_32(pv->addr + STM32_USART_CR2_ADDR, 0);
  cpu_mem_write_32(pv->addr + STM32_USART_CR3_ADDR, 0);

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_unlink(dev, pv->irq_ep, 1);

# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_destroy(&pv->read_fifo);
  usart_fifo_destroy(&pv->write_fifo);
# endif
#endif

  /* destroy request queues. */
  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(stm32_usart_drv, 0, "STM32 USART", stm32_usart,
               DRIVER_CHAR_METHODS(stm32_usart),
               DRIVER_VALIO_METHODS(stm32_usart_valio));

DRIVER_REGISTER(stm32_usart_drv);


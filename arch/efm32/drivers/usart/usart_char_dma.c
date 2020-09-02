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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2017
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
#include <device/clock.h>

#include <arch/efm32/usart.h>

#include <device/class/dma.h>
#include <arch/efm32/dma_source.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/prs.h>
#include <arch/efm32/devaddr.h>
#include <arch/efm32/timer.h>

#include <cpu/arm32m/pl230_channel.h>

enum efm32_usart_start_e
{
  EFM32_USART_STARTED_READ = 1,
  EFM32_USART_STARTED_WRITE = 2,
};

DRIVER_PV(struct efm32_usart_context_s
{
  uintptr_t addr;
  /* tty input request queue and char fifo */
  dev_request_queue_root_t	read_q;
  dev_request_queue_root_t	write_q;
  struct kroutine_s             read_kr;
  struct kroutine_s             write_kr;
  struct dev_dma_rq_s           dma_rd_rq;
  struct dev_dma_desc_s         dma_rd_desc[2];

  struct dev_dma_rq_s           dma_wr_rq;
  struct dev_dma_desc_s         dma_wr_desc;

  struct device_dma_s           dma;
  struct device_s               *usart;
  bool_t                        dma_started;

  uintptr_t                     timer_addr;
  struct dev_irq_src_s          irq_ep[2];
  uint8_t                       rx_buffer[CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE];
  uint8_t                       *rptr;
  uint8_t                       *wptr;
  uint8_t                       pid:1;
  uint8_t                       err:1;
  struct dev_freq_s             timer_freq;
  struct dev_clock_sink_ep_s    clk_ep[3];
  struct dev_uart_config_s      cfg;
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  uint32_t                      clkdiv;
#endif
  struct dev_freq_s             freq;
});

STRUCT_COMPOSE(efm32_usart_context_s, read_kr)
STRUCT_COMPOSE(efm32_usart_context_s, write_kr)
STRUCT_COMPOSE(efm32_usart_context_s, dma_wr_rq)
STRUCT_COMPOSE(efm32_usart_context_s, dma_rd_rq)

static uint32_t efm32_usart_char_bauds(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  return (256 * pv->freq.num) / (4 * pv->cfg.baudrate * pv->freq.denom) - 256;
}

static void efm32_usart_char_cfg_apply(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  pv->clkdiv = endian_le32(efm32_usart_char_bauds(dev));
  cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, pv->clkdiv);
#endif

  uint32_t frame = 0;

  EFM32_USART_FRAME_DATABITS_SETVAL(frame,
    EFM32_USART_FRAME_DATABITS_FOUR + pv->cfg.data_bits - 4);

  switch (pv->cfg.parity)
    {
    case DEV_UART_PARITY_EVEN:
      EFM32_USART_FRAME_PARITY_SET(frame, EVEN);
      break;
    case DEV_UART_PARITY_ODD:
      EFM32_USART_FRAME_PARITY_SET(frame, ODD);
      break;
    case DEV_UART_PARITY_NONE:
    default:
      EFM32_USART_FRAME_PARITY_SET(frame, NONE);
      break;
    }

  switch (pv->cfg.stop_bits)
    {
    case 2:
      EFM32_USART_FRAME_STOPBITS_SET(frame, TWO);
      break;
    case 1:
    default:
      EFM32_USART_FRAME_STOPBITS_SET(frame, ONE);
      break;
    }

  cpu_mem_write_32(pv->addr + EFM32_USART_FRAME_ADDR,
                   endian_le32(frame));
}

static void efm32_usart_entry_low_power(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

   if (dev->start_count == 0 && pv->dma_started)
     {
       /* End DMA read operation */
       ensure(DEVICE_OP(&pv->dma, cancel, &pv->dma_rd_rq) == 0);
       pv->wptr = pv->rptr = pv->rx_buffer;
       pv->dma_started = 0;
     }

#ifdef CONFIG_DEVICE_CLOCK_GATING
   if (dev->start_count == 0)
     dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER);
#endif

}

static void efm32_usart_try_write(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s		*rq;
  bool_t done = 0;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  if (pv->clkdiv)
    {
      cpu_mem_write_32(pv->addr + EFM32_USART_CLKDIV_ADDR, pv->clkdiv);
      pv->clkdiv = 0;
    }
#endif

  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR,
                   endian_le32(EFM32_USART_IFC_TXC));
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, EFM32_USART_IEN_TXC);

  while ((rq = dev_char_rq_head(&pv->write_q)))
    {
      size_t size = 0;

      if (size < rq->size && (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
                & endian_le32(EFM32_USART_STATUS_TXBL)))
        {
          cpu_mem_write_32(pv->addr + EFM32_USART_TXDATAX_ADDR,
                           endian_le32(rq->data[size++]));
          done = 1;
        }

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

  if (!done)
    dev->start_count &= ~EFM32_USART_STARTED_WRITE;
}

static void efm32_usart_check_dma_ovf(struct efm32_usart_context_s *pv)
{
  if (pv->err)
    return;

  pv->err = pv->pid ? (pv->wptr > pv->rptr) : (pv->rptr > pv->wptr);
}

static void efm32_usart_end_rq(struct efm32_usart_context_s *pv,
                               struct dev_char_rq_s *rq,
                               error_t err)
{
  rq->error = err;
  dev_char_rq_pop(&pv->read_q);
  dev_char_rq_done(rq);
}

static void efm32_usart_update_read_rq(struct efm32_usart_context_s *pv, size_t size)
{
  struct dev_char_rq_s *rq = dev_char_rq_head(&pv->read_q);

  assert(rq);

  memcpy(rq->data, (uint8_t *)pv->rptr, size);

  LOCK_SPIN_IRQ(&pv->usart->lock);

  rq->size -= size;
  rq->data += size;
  pv->rptr += size;

  uint8_t * top = pv->rx_buffer + CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE;

  if (pv->rptr == top)
  /* Reset read pointer */
    {
      pv->pid ^= 1;
      pv->rptr = pv->rx_buffer;
    }

  if (rq->size == 0 || (rq->type & _DEV_CHAR_PARTIAL))
  /* Request is terminated */
    efm32_usart_end_rq(pv, rq, 0);

  LOCK_RELEASE_IRQ(&pv->usart->lock);
}

static size_t efm32_usart_get_read_size(struct efm32_usart_context_s *pv)
{
  size_t size;

  LOCK_SPIN_IRQ(&pv->usart->lock);

  uint8_t * top = pv->rx_buffer + CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE;

  while(1)
    {
      struct dev_char_rq_s *rq = dev_char_rq_head(&pv->read_q);

      size = 0;

      if (rq == NULL)
        {
          pv->usart->start_count &= ~EFM32_USART_STARTED_READ;
          cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR, EFM32_USART_CMD_RXDIS);
          goto end;
        }

      uint8_t *wptr = pv->wptr;

      if (pv->err)
      /* An overflow has occurred */
        {
          pv->err = 0;
          pv->pid = 0;
          pv->rptr = wptr;
          efm32_usart_end_rq(pv, rq, -EPIPE);
          continue;
        }

      size = wptr - pv->rptr + pv->pid * CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE;

      if (size == 0)
      /* Fifo is empty */
        goto end;

      if (pv->pid)
      /* Write pointer has wrapped */
        size = top - pv->rptr;

      if (size > rq->size)
        size = rq->size;

      break;
    }

end:

  efm32_usart_entry_low_power(pv->usart);

  LOCK_RELEASE_IRQ(&pv->usart->lock);
  return size;
}

static KROUTINE_EXEC(efm32_usart_dma_try_read)
{
  struct efm32_usart_context_s *pv = efm32_usart_context_s_from_read_kr(kr);

  while(1)
    {
      size_t size = efm32_usart_get_read_size(pv);

      if (size == 0)
        break;

      efm32_usart_update_read_rq(pv, size);
    }
}

static DEV_DMA_CALLBACK(efm32_usart_dma_read_done)
{
  assert(err == 0);

  struct efm32_usart_context_s *pv =
    efm32_usart_context_s_from_dma_rd_rq(rq);

  lock_spin(&pv->usart->lock);

  pv->wptr = pv->rx_buffer + ((desc_id ^ 1) * (CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE/2));

  if (desc_id)
    pv->pid ^= 1;

  efm32_usart_check_dma_ovf(pv);

  kroutine_exec(&pv->read_kr);

  lock_release(&pv->usart->lock);

  return 1;
}

static void efm32_usart_start_tx(struct device_s *dev);

static KROUTINE_EXEC(efm32_usart_dma_process_next_write)
{
  struct efm32_usart_context_s *pv = efm32_usart_context_s_from_write_kr(kr);

  LOCK_SPIN_IRQ(&pv->usart->lock);

  struct dev_char_rq_s *rq = dev_char_rq_head(&pv->write_q);

  assert(rq);

  dev_char_rq_pop(&pv->write_q);
  dev_char_rq_done(rq);

  /* Process next request */
  efm32_usart_start_tx(pv->usart);

  efm32_usart_entry_low_power(pv->usart);

  LOCK_RELEASE_IRQ(&pv->usart->lock);
}

static void efm32_usart_tx_dma_end(struct efm32_usart_context_s *pv, error_t err)
{
  struct dev_char_rq_s *crq = dev_char_rq_head(&pv->write_q);

  crq->data += crq->size;
  crq->size = 0;
  crq->error = err;

  /* We can not restart DMA core in DMA callback */
  kroutine_exec(&pv->write_kr);
}

static void efm32_usart_usart_start_tx_dma(struct efm32_usart_context_s *pv)
{
  struct dev_char_rq_s *rq = dev_char_rq_head(&pv->write_q);
  struct dev_dma_desc_s * desc = &pv->dma_wr_desc;

  /* TX */
  desc->src.mem.addr = (uintptr_t)rq->data;
  desc->src.mem.size = rq->size - 1;

  /* Start DMA request */
  if (DEVICE_OP(&pv->dma, request, &pv->dma_wr_rq, NULL))
    efm32_usart_tx_dma_end(pv, -EIO);
}


static DEV_DMA_CALLBACK(efm32_usart_dma_write_done)
{
  struct efm32_usart_context_s *pv =
    efm32_usart_context_s_from_dma_wr_rq(rq);

  lock_spin(&pv->usart->lock);

  efm32_usart_tx_dma_end(pv, 0);

  lock_release(&pv->usart->lock);

  return 0;
}

static void efm32_usart_dma_timeout_irq(struct efm32_usart_context_s *pv)
{
  assert(pv->timer_addr);

  uint32_t ir = endian_le32(cpu_mem_read_32(pv->timer_addr + EFM32_TIMER_IF_ADDR));

  cpu_mem_write_32(pv->timer_addr + EFM32_TIMER_IFC_ADDR, ir);

  if (!pv->dma_started)
    return;

  struct dev_dma_status_s status;
  /* Get DMA status */
  DEVICE_OP(&pv->dma, get_status, &pv->dma_rd_rq, &status);
  /* Update write pointer with DMA status */
  pv->wptr = (uint8_t*)status.dst_addr;
  /* Check Overflow */
  efm32_usart_check_dma_ovf(pv);

  kroutine_exec(&pv->read_kr);
}

static void efm32_usart_start_tx(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  struct dev_char_rq_s *rq = dev_char_rq_head(&pv->write_q);

  if (rq == NULL)
  /* Write queue is empty */
    {
      dev->start_count &= ~EFM32_USART_STARTED_WRITE;
      return;
    }

  if (rq->type == DEV_CHAR_WRITE)
    {
      /* interrupt disabled */
      uint32_t i = cpu_mem_read_32(pv->addr + EFM32_USART_IEN_ADDR);
      i &= ~EFM32_USART_IEN_TXC;
      cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, endian_le32(i));

      efm32_usart_usart_start_tx_dma(pv);
      return;
    }

  efm32_usart_try_write(dev);
}

static void efm32_usart_start_rx(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  dev->start_count |= EFM32_USART_STARTED_READ;
#ifdef CONFIG_DEVICE_CLOCK_GATING
   dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER_CLOCK);
#endif
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR, EFM32_USART_CMD_RXEN);

  if (!pv->dma_started)
  /* Start DMA read */
    {
      DEVICE_OP(&pv->dma, request, &pv->dma_rd_rq, NULL);
      pv->dma_started = 1;
    }

  kroutine_exec(&pv->read_kr);
  return;
}

#define efm32_usart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static DEV_CHAR_REQUEST(efm32_usart_request)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  error_t err = 0;

  assert(rq->size);


  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_CHAR_READ:
    case DEV_CHAR_READ_PARTIAL: {
      dev_char_rq_pushback(&pv->read_q, rq);
      efm32_usart_start_rx(dev);
      break;
    }
    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      bool_t empty = dev_rq_queue_isempty(&pv->write_q);
      dev_char_rq_pushback(&pv->write_q, rq);
      dev->start_count |= EFM32_USART_STARTED_WRITE;
#ifdef CONFIG_DEVICE_CLOCK_GATING
      dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER_CLOCK);
#endif
      if (empty)
        efm32_usart_start_tx(dev);
      break;
    }
    default:
      err = -ENOTSUP;
      break;
    }

  efm32_usart_entry_low_power(dev);

  LOCK_RELEASE_IRQ(&dev->lock);

  if (err)
    {
      rq->error = err;
      dev_char_rq_done(rq);
    }
}

#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)

static DEV_VALIO_REQUEST(efm32_usart_valio_request)
{
  struct device_s *dev = accessor->dev;
  struct efm32_usart_context_s *pv = dev->drv_pv;

  rq->error = -ENOTSUP;

  if (rq->attribute == VALIO_UART_CONFIG)
    {
      switch (rq->type)
        {
        case DEVICE_VALIO_READ:
          LOCK_SPIN_IRQ(&dev->lock);
          *(struct dev_uart_config_s*)rq->data = pv->cfg;
          LOCK_RELEASE_IRQ(&dev->lock);

          rq->error = 0;
          break;

        case DEVICE_VALIO_WRITE:
          LOCK_SPIN_IRQ(&dev->lock);
          pv->cfg = *(struct dev_uart_config_s*)rq->data;
          efm32_usart_char_cfg_apply(dev);
          LOCK_RELEASE_IRQ(&dev->lock);

          rq->error = 0;
          break;

        default:
          break;
        }
    }

  dev_valio_rq_done(rq);
}

#define efm32_usart_valio_cancel (dev_valio_cancel_t*)dev_driver_notsup_fcn
#endif


static DEV_IRQ_SRC_PROCESS(efm32_usart_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  lock_spin(&dev->lock);

  uint32_t ir;

  switch (ep - pv->irq_ep)
    {
      case 0:
        while (1)
          {
            ir = endian_le32(cpu_mem_read_32(pv->addr + EFM32_USART_IF_ADDR));

            if (!(ir & (EFM32_USART_IF_TXC)))
              break;

            if (ir & EFM32_USART_IF_TXC)
              efm32_usart_try_write(dev);
          }
        break;
      case 1:
        efm32_usart_dma_timeout_irq(pv);
        break;
      default:
        break;
  }

  efm32_usart_entry_low_power(dev);
  lock_release(&dev->lock);
}

static DEV_USE(efm32_usart_char_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_usart_context_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      efm32_usart_char_cfg_apply(dev);
      return 0;
    }
#endif

#ifdef CONFIG_DEVICE_CLOCK_GATING
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct efm32_usart_context_s *pv = dev->drv_pv;
      if (dev->start_count == 0)
        dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER_CLOCK);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      efm32_usart_entry_low_power(dev);
      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}

#define PRS_CHANNEL 1
#define CC_CHANNEL 0

static error_t efm32_usart_timeout_init(struct efm32_usart_context_s *pv,
                                        uint32_t pin)
{
  uint32_t x;

  /* Prs init */

  bool_t low = (pin % 16) < 8;

  uintptr_t offset = low ? EFM32_GPIO_EXTIPSELL_ADDR : EFM32_GPIO_EXTIPSELH_ADDR;

  x = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + offset));
  EFM32_GPIO_EXTIPSELH_EXT_SETVAL(pin % 8, x, pin /16);
  cpu_mem_write_32(EFM32_GPIO_ADDR + offset, endian_le32(x));

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
  x = low ? EFR32_PRS_CH_CTRL_SOURCESEL(GPIOL) : EFR32_PRS_CH_CTRL_SOURCESEL(GPIOH);
  x |= EFR32_PRS_CH_CTRL_EDSEL(OFF);
  EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, pin % 8);
  cpu_mem_write_32(EFM32_PRS_ADDR + EFR32_PRS_CH_CTRL_ADDR(PRS_CHANNEL), endian_le32(x));
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  x = low ? EFM32_PRS_CTRL_SOURCESEL(GPIOL) : EFM32_PRS_CTRL_SOURCESEL(GPIOH);
  x |= EFM32_PRS_CTRL_EDSEL(OFF);
  EFM32_PRS_CTRL_SIGSEL_SET(x, pin % 8);
  cpu_mem_write_32(EFM32_PRS_ADDR + EFM32_PRS_CTRL_ADDR(PRS_CHANNEL), endian_le32(x));
#endif

  uintptr_t addr = pv->timer_addr;

  /* Stop timer and clear irq */
  cpu_mem_write_32(addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
  cpu_mem_write_32(addr + EFM32_TIMER_IEN_ADDR, 0);
  cpu_mem_write_32(addr + EFM32_TIMER_IFC_ADDR, endian_le32(EFM32_TIMER_IFC_MASK));

  /* Select HFPERCLK as source */
  x = EFM32_TIMER_CTRL_MODE(UP) |
      EFM32_TIMER_CTRL_OSMEN |
      EFM32_TIMER_CTRL_CLKSEL(PRESCHFPERCLK) |
      EFM32_TIMER_CTRL_RISEA(RELOADSTART) ;

  cpu_mem_write_32(addr + EFM32_TIMER_CTRL_ADDR, endian_le32(x));

  /* CC0 configuration */

  /* Set input PRS channel as timer control */
  x = EFM32_TIMER_CC_CTRL_MODE(OFF) |
      EFM32_TIMER_CC_CTRL_FILT |
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
      EFM32_TIMER_CC_CTRL_INSEL;
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
      EFM32_TIMER_CC_CTRL_INSEL(PRS);
#endif
  EFM32_TIMER_CC_CTRL_PRSSEL_SETVAL(x, PRS_CHANNEL);

  cpu_mem_write_32(addr + EFM32_TIMER_CC_CTRL_ADDR(CC_CHANNEL), endian_le32(x));

  struct dev_freq_s freq = {
      .num   = pv->cfg.baudrate,
      .denom = CONFIG_DRIVER_EFM32_USART_CHAR_DMA_TIMEOUT_SYMBOL,
    };

  /* Compute scale factor to the requested frequency. */
  uint64_t scale = (pv->timer_freq.num * freq.denom) / (pv->timer_freq.denom * freq.num);

  uint32_t msb = scale >> 16;

  uint8_t div = msb ? bit_msb_index(msb) + 1 : 0;

  if (div > 10)
  /* Frequency can not be achieved */
    return -ERANGE;

  scale = scale / (1 << div);

  if ((scale - 1) >> 16)
    return -ERANGE;

  /* Top value */
  cpu_mem_write_32(addr + EFM32_TIMER_TOP_ADDR, endian_le32(scale - 1));

  /* Prescaler */
  x = endian_le32(cpu_mem_read_32(addr + EFM32_TIMER_CTRL_ADDR));
  EFM32_TIMER_CTRL_PRESC_SETVAL(x, div);
  cpu_mem_write_32(addr + EFM32_TIMER_CTRL_ADDR, endian_le32(x));

  cpu_mem_write_32(addr + EFM32_TIMER_IEN_ADDR, EFM32_TIMER_IEN_OF);

  return 0;
}

static DEV_INIT(efm32_usart_char_init)
{
  struct efm32_usart_context_s	*pv;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* setup pinmux */
  iomux_io_id_t pin[2];
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, "<rx? >tx?", loc, pin, NULL))
    goto err_mem;

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
  uint32_t enable = 0;
  uint32_t route = 0;

  if (loc[0] != IOMUX_INVALID_DEMUX)
    {
      enable |= EFM32_USART_ROUTEPEN_RXPEN;
      EFM32_USART_ROUTELOC0_RXLOC_SETVAL(route, loc[0]);
    }
  if (loc[1] != IOMUX_INVALID_DEMUX)
    {
      enable |= EFM32_USART_ROUTEPEN_TXPEN;
      EFM32_USART_ROUTELOC0_TXLOC_SETVAL(route, loc[1]);
    }
  if (enable == 0)
    goto err_mem;

#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  uint32_t route = 0;
  if (loc[0] != IOMUX_INVALID_DEMUX)
    route |= EFM32_USART_ROUTE_RXPEN;
  if (loc[1] != IOMUX_INVALID_DEMUX)
    route |= EFM32_USART_ROUTE_TXPEN;

  EFM32_USART_ROUTE_LOCATION_SETVAL(route, loc[0] != IOMUX_INVALID_DEMUX ? loc[0] : loc[1]);

  if (route == 0)
    goto err_mem;
#else
# error
#endif

  /* init software fifos */
  dev_rq_queue_init(&pv->read_q);
  dev_rq_queue_init(&pv->write_q);

  if (dev_drv_clock_init(dev, &pv->clk_ep[0], 0, DEV_CLOCK_EP_FREQ_NOTIFY |
                     DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto err_irq;

  /* wait for current TX to complete */
  if (cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
      & endian_le32(EFM32_USART_STATUS_TXENS))
      while (!(cpu_mem_read_32(pv->addr + EFM32_USART_STATUS_ADDR)
               & endian_le32(EFM32_USART_STATUS_TXBL)))
        ;

  /* disable and clear the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTELOC0_ADDR, endian_le32(route));
  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTEPEN_ADDR, endian_le32(enable));
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  cpu_mem_write_32(pv->addr + EFM32_USART_ROUTE_ADDR, endian_le32(route));
#else
# error
#endif

  /* enable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IFC_ADDR, endian_le32(EFM32_USART_IFC_MASK));

  /* configure */
  cpu_mem_write_32(pv->addr + EFM32_USART_CTRL_ADDR, endian_le32(EFM32_USART_CTRL_OVS(X4)));

  /* set config */
  if (device_get_res_uart(dev, &pv->cfg)) {
    pv->cfg.baudrate = CONFIG_DRIVER_EFM32_USART_RATE;
    pv->cfg.data_bits = 8;
    pv->cfg.parity = DEV_UART_PARITY_NONE;
    pv->cfg.stop_bits = 1;
  }

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  pv->clkdiv = 0;
#endif
  efm32_usart_char_cfg_apply(dev);

  /* enable the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_TXEN | EFM32_USART_CMD_RXBLOCKDIS));

  if (device_res_get_uint(dev, DEV_RES_MEM, 1, &pv->timer_addr, NULL))
    {
      pv->timer_addr = 0;
      goto err_irq;
    }

  if (dev_drv_clock_init(dev, &pv->clk_ep[1], 1, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->timer_freq))
    goto err_irq;
  if (dev_drv_clock_init(dev, &pv->clk_ep[2], 2, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL))
    goto err_irq;

  pv->wptr = pv->rptr = pv->rx_buffer;
  pv->dma_started = 0;
  pv->err = 0;

  if (efm32_usart_timeout_init(pv, pin[0]))
    goto err_irq;

  uint32_t read_link, write_link;
  uint32_t read_mask, write_mask;

  pv->usart = dev;

  if (device_res_get_dma(dev, 0, &read_mask, &read_link) ||
      device_res_get_dma(dev, 1, &write_mask, &write_link) ||
      device_get_param_dev_accessor(dev, "dma", &pv->dma.base, DRIVER_CLASS_DMA))
    goto err_irq;

  struct dev_dma_rq_s *rq = &pv->dma_rd_rq;
  struct dev_dma_desc_s *desc = pv->dma_rd_desc;

  /* RX */

  for (uint8_t i = 0; i < 2; i++)
    {
      desc = pv->dma_rd_desc + i;
      desc->src.reg.addr = pv->addr + EFM32_USART_RXDATA_ADDR;
      desc->src.reg.width = 0;
      desc->src.reg.burst = 1;
      desc->dst.mem.inc = DEV_DMA_INC_1_UNITS;
      desc->src.reg.size = CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE/2 - 1;
      desc->dst.mem.addr = (uintptr_t)pv->rx_buffer + i * (CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE/2);
    }

  rq->dev_link.src = read_link | (EFM32_DMA_SIGNAL_USARTRXDATAV << 8);
  rq->type = DEV_DMA_REG_MEM_CONT;
  rq->desc_count_m1 = 1;
  rq->loop_count_m1 = 0;
  rq->chan_mask = read_mask;
  rq->f_done = efm32_usart_dma_read_done;
  rq->cache_ptr = NULL;

  /* TX */

  rq = &pv->dma_wr_rq;
  desc = &pv->dma_wr_desc;

  desc->dst.reg.addr = pv->addr + EFM32_USART_TXDATA_ADDR;
  desc->dst.reg.burst = 1;
  desc->src.mem.inc = DEV_DMA_INC_1_UNITS;
  desc->src.mem.width = 0;

  rq->dev_link.dst = write_link | (EFM32_DMA_SIGNAL_USARTTXBL << 8);
  rq->type = DEV_DMA_MEM_REG;
  rq->desc_count_m1 = 0;
  rq->loop_count_m1 = 0;
  rq->chan_mask = write_mask;
  rq->f_done = efm32_usart_dma_write_done;
  rq->cache_ptr = NULL;

  kroutine_init_deferred(&pv->read_kr, &efm32_usart_dma_try_read);
  kroutine_init_deferred(&pv->write_kr, &efm32_usart_dma_process_next_write);

  /* init irq endpoints */
  device_irq_source_init(dev, pv->irq_ep, 2, &efm32_usart_irq);

  if (device_irq_source_link(dev, pv->irq_ep, 2, -1))
    goto err_fifo;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER);
#endif

  return 0;

 err_irq:
  device_irq_source_unlink(dev, pv->irq_ep, 2);
 err_fifo:
  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_usart_char_cleanup)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  /* disable irqs */
  cpu_mem_write_32(pv->addr + EFM32_USART_IEN_ADDR, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 2);

  /* disable the uart */
  cpu_mem_write_32(pv->addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

  dev_drv_clock_cleanup(dev, &pv->clk_ep[0]);
  dev_drv_clock_cleanup(dev, &pv->clk_ep[1]);

  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(efm32_usart_dma_drv, 0, "EFM32 USART (char,dma)", efm32_usart_char,
#if defined(CONFIG_DEVICE_VALIO_UART_CONFIG)
               DRIVER_VALIO_METHODS(efm32_usart_valio),
#endif
               DRIVER_CHAR_METHODS(efm32_usart));

DRIVER_REGISTER(efm32_usart_dma_drv);

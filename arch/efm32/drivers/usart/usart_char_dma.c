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

/*
DEV_DECLARE_STATIC(usart1_dev, "uart_mux", 0, efm32_usart_dma_drv,
                   DEV_STATIC_RES_MEM(0x4000c400, 0x4000c800),

                   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_USART1, 0),
		   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_TIMER1, 1),
		   DEV_STATIC_RES_CLK_SRC("/recmu", EFM32_CLOCK_PRS, 2),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_MEM(0x40010400, 0x40010800),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_TIMER1, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("dma", "/dma"),
                   DEV_STATIC_RES_DMA((1 << 0), EFM32_DMA_SOURCE_USART1),
                   DEV_STATIC_RES_DMA((1 << 1), EFM32_DMA_SOURCE_USART1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("rx", EFM32_LOC1, EFM32_PD1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", EFM32_LOC1, EFM32_PD0, 0, 0),
                   DEV_STATIC_RES_UART(9600, 8, 0, 0, 0)
                   );
*/

#define LOGK_MODULE_ID "ecdu"

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

#if CONFIG_DEVICE_START_LOG2INC < 2
# error
#endif

enum efm32_usart_start_e
{
  EFM32_USART_STARTED_READ = 1,
  EFM32_USART_STARTED_WRITE = 2,
};

enum efm32_usart_rx_dma_state_s
{
  EFM32_USART_RX_DESC_0 = 0,
  EFM32_USART_RX_DESC_1 = 1,
  EFM32_USART_RX_STOPPED = 2,
};

DRIVER_PV(struct efm32_usart_context_s
{
  struct device_s               *dev;

  uintptr_t usart_addr;
  uintptr_t timer_addr;

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

  struct dev_irq_src_s          irq_ep;

  uint8_t                       rx_buffer[CONFIG_DRIVER_EFM32_USART_CHAR_DMA_FIFO_SIZE];
  /* next data available for copy to read requests */
  uint16_t                      rx_rptr;
  /* amount of data that is available */
  uint16_t                      rx_size;
  bool_t                        rx_overflow:1;
  enum efm32_usart_rx_dma_state_s rx_state:2;
  enum efm32_usart_start_e      enabled:2;

  struct dev_freq_s             timer_freq;
  struct dev_clock_sink_ep_s    clk_ep[3]; /* USART / TIMER / PRS */
  struct dev_uart_config_s      cfg;
  uint32_t                      clkdiv;
  struct dev_freq_s             freq;
});

STRUCT_COMPOSE(efm32_usart_context_s, read_kr)
STRUCT_COMPOSE(efm32_usart_context_s, write_kr)
STRUCT_COMPOSE(efm32_usart_context_s, dma_wr_rq)
STRUCT_COMPOSE(efm32_usart_context_s, dma_rd_rq)

static uint32_t efm32_usart_char_bauds(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  return (256 * pv->freq.num)
    / (4 * pv->cfg.baudrate * pv->freq.denom) - 256;
}

static void efm32_usart_char_cfg_apply(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  pv->clkdiv = endian_le32(efm32_usart_char_bauds(dev));
  cpu_mem_write_32(pv->usart_addr + EFM32_USART_CLKDIV_ADDR, pv->clkdiv);

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

  cpu_mem_write_32(pv->usart_addr + EFM32_USART_FRAME_ADDR,
                   endian_le32(frame));

  /* XXX need to update timer delay as well */
}

static void efm32_usart_start(struct device_s *dev, enum efm32_usart_start_e s)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  if (!dev->start_count)
    {
      dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER_CLOCK);
      dev_clock_sink_gate(&pv->clk_ep[1], DEV_CLOCK_EP_POWER_CLOCK);
      dev_clock_sink_gate(&pv->clk_ep[2], DEV_CLOCK_EP_POWER_CLOCK);
    }
#endif

  if (pv->rx_state == EFM32_USART_RX_STOPPED &&
      (pv->enabled & EFM32_USART_STARTED_READ) &&
      s != EFM32_USART_STARTED_WRITE)
    {
      cpu_mem_write_32(pv->usart_addr + EFM32_USART_CMD_ADDR, EFM32_USART_CMD_RXEN);

      /* Start DMA read */
      pv->rx_state = EFM32_USART_RX_DESC_1;
      pv->rx_overflow = 0;
      pv->rx_rptr = 0;
      pv->rx_size = 0;

      DEVICE_OP(&pv->dma, request, &pv->dma_rd_rq, NULL);
    }

  dev->start_count |= s;
}

static void efm32_usart_stop(struct device_s *dev)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  if (dev->start_count)
    return;

  if (pv->rx_state != EFM32_USART_RX_STOPPED)
    {
      cpu_mem_write_32(pv->usart_addr + EFM32_USART_CMD_ADDR, EFM32_USART_CMD_RXDIS);
      ensure(DEVICE_OP(&pv->dma, cancel, &pv->dma_rd_rq) == 0);
      pv->rx_state = EFM32_USART_RX_STOPPED;
    }

  dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER);
  dev_clock_sink_gate(&pv->clk_ep[1], DEV_CLOCK_EP_POWER);

  /* XXX need refcount on clock ep because PRS may be shared between
     multiple drivers */
  dev_clock_sink_gate(&pv->clk_ep[2], DEV_CLOCK_EP_POWER);
}

static void efm32_usart_try_stop(struct device_s *dev, enum efm32_usart_start_e s)
{
  if (dev->start_count &= ~s)
    return;

  device_sleep_schedule(dev);
}

/*********************************************************************** rx */

static DEV_DMA_CALLBACK(efm32_usart_dma_read_done)
{
  bool_t wakeup;
  assert(err == 0);

  struct efm32_usart_context_s *pv =
    efm32_usart_context_s_from_dma_rd_rq(rq);

  LOCK_SPIN_IRQ(&pv->dev->lock);

  wakeup = !dev_rq_queue_isempty(&pv->read_q);

  /* there are 2 cases of rx overflow that we can detect:
     - the DMA has completed a descriptor but there is still unconsumed
       data in the other buffer half (the rx ptr is located in the other half).
     - the DMA reports completion of the same descriptor twice
  */

  const uint_fast16_t h = sizeof(pv->rx_buffer) / 2;

  if ((desc_id ^ (pv->rx_rptr >= h)) ||
      (desc_id == pv->rx_state))
    {
      /* discard data in the other half */
      pv->rx_overflow = 1;
      pv->rx_rptr = desc_id * h;
      pv->rx_size = h;
    }
  else
    {
      /* this half is now full of data */
      pv->rx_size = h - (pv->rx_rptr % h);
    }

  pv->rx_state = desc_id;

  LOCK_RELEASE_IRQ(&pv->dev->lock);

  if (wakeup)
    kroutine_exec(&pv->read_kr);

  return 1;
}

static uint_fast16_t
efm32_usart_dma_update_rx_size(struct efm32_usart_context_s *pv)
{
  if (pv->rx_state == EFM32_USART_RX_STOPPED)
    return 0;

  /* Get DMA status */
  struct dev_dma_status_s status;
  DEVICE_OP(&pv->dma, get_status, &pv->dma_rd_rq, &status);

  /* current DMA write pointer */
  uint_fast16_t p = status.dst_addr - (uintptr_t)pv->rx_buffer;

  uint_fast16_t r = (uint_fast16_t)(p - pv->rx_rptr) % sizeof(pv->rx_buffer);
  pv->rx_size = r;

  return r;
}

static DEV_IRQ_SRC_PROCESS(efm32_usart_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  bool_t wakeup;

  lock_spin(&dev->lock);

  wakeup = !dev_rq_queue_isempty(&pv->read_q);

  if (wakeup && !pv->rx_overflow)
    {
      uint_fast16_t r = efm32_usart_dma_update_rx_size(pv);
      if (!r)
        wakeup = 0;
    }

  uint32_t ir = endian_le32(cpu_mem_read_32(pv->timer_addr + EFM32_TIMER_IF_ADDR));
  cpu_mem_write_32(pv->timer_addr + EFM32_TIMER_IFC_ADDR, ir);

  lock_release(&dev->lock);

  if (wakeup)
    kroutine_exec(&pv->read_kr);
}

static KROUTINE_EXEC(efm32_usart_dma_try_read)
{
  struct efm32_usart_context_s *pv = efm32_usart_context_s_from_read_kr(kr);

  while (1)
    {
      struct dev_char_rq_s *rq = dev_char_rq_head(&pv->read_q);
      uint_fast16_t p;
      size_t r;
      bool_t o;

      if (!rq)
        {
          efm32_usart_try_stop(pv->dev, EFM32_USART_STARTED_READ);
          break;
        }

      /* snapshot fifo state */
    retry:
      LOCK_SPIN_IRQ(&pv->dev->lock);
      r = pv->rx_size;
      p = pv->rx_rptr;
      o = pv->rx_overflow;
      pv->rx_overflow = 0;

      /* we do not want to wait for the next timer/dma irq to learn
         about the actual amount of data that has been rx. */
      if (!r && !o)
        r = efm32_usart_dma_update_rx_size(pv);

      LOCK_RELEASE_IRQ(&pv->dev->lock);

      if (o)
         {
          rq->error = -EPIPE;
          goto overflow;
        }

      /* amount of data that can be copied */
      size_t s = __MIN(rq->size, r);

      if (s && rq->type != DEV_CHAR_READ_POLL)
        {
          /* amount of data available before wrap */
          size_t w = sizeof(pv->rx_buffer) - p;

          if (rq->type != DEV_CHAR_DISCARD)
            {
              if (s > w)
                {
                  memcpy(rq->data, pv->rx_buffer + p, w);
                  memcpy(rq->data + w, pv->rx_buffer, s - w);
                }
              else
                {
                  memcpy(rq->data, pv->rx_buffer + p, s);
                }
            }

          /* update fifo state */
          LOCK_SPIN_IRQ(&pv->dev->lock);
          o = pv->rx_overflow;
          pv->rx_overflow = 0;
          if (!o)
            {
              pv->rx_rptr = (p + s) % sizeof(pv->rx_buffer);
              pv->rx_size -= s;
            }
          LOCK_RELEASE_IRQ(&pv->dev->lock);

          if (o)
            {
              rq->error = -EPIPE;
              goto overflow;
            }

          rq->data += s;
          rq->size -= s;
        }

      switch (rq->type)
        {
        case DEV_CHAR_READ:
        case DEV_CHAR_DISCARD:
          if (rq->size == 0)
            break;
          if (!r)
            return;
         goto retry;

        case DEV_CHAR_READ_POLL:
        case DEV_CHAR_READ_PARTIAL:
          if (!r)
            return;
          break;

        case DEV_CHAR_READ_NONBLOCK:
          break;

        default:
          UNREACHABLE();
        }

      rq->error = 0;
    overflow:
      dev_char_rq_pop(&pv->read_q);
      dev_char_rq_done(rq);
    }
}

/*********************************************************************** tx */

static void efm32_usart_start_tx(struct device_s *dev,
                                 struct dev_char_rq_s *rq)
{
  struct efm32_usart_context_s *pv = dev->drv_pv;
  struct dev_dma_desc_s *desc = &pv->dma_wr_desc;

  /* TX */
  desc->src.mem.addr = (uintptr_t)rq->data;
  desc->src.mem.size = rq->size - 1;

  /* Start DMA request */
  ensure(DEVICE_OP(&pv->dma, request, &pv->dma_wr_rq, NULL) == 0);
}

static KROUTINE_EXEC(efm32_usart_dma_process_next_write)
{
  struct efm32_usart_context_s *pv = efm32_usart_context_s_from_write_kr(kr);

  struct dev_char_rq_s *rq = dev_char_rq_head(&pv->write_q);
  assert(rq);

  efm32_usart_start_tx(pv->dev, rq);
}

static DEV_DMA_CALLBACK(efm32_usart_dma_write_done)
{
  struct efm32_usart_context_s *pv =
    efm32_usart_context_s_from_dma_wr_rq(rq);

  lock_spin(&pv->dev->lock);

  struct dev_char_rq_s *crq = dev_char_rq_pop(&pv->write_q);
  assert(crq);

  crq->data += crq->size;
  crq->size = 0;
  crq->error = 0;
  dev_char_rq_done(crq);

  if (!dev_rq_queue_isempty(&pv->write_q))
    kroutine_exec(&pv->write_kr);
  else
    efm32_usart_try_stop(pv->dev, EFM32_USART_STARTED_WRITE);

  lock_release(&pv->dev->lock);

  return 0;    /* XXX could we let the DMA run intead of restarting? */
}

/************************************************************************/

#define efm32_usart_cancel (dev_char_cancel_t*)&dev_driver_notsup_fcn

static DEV_CHAR_REQUEST(efm32_usart_request)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_usart_context_s	*pv = dev->drv_pv;
  error_t err = 0;

  switch (rq->type)
    {
    case DEV_CHAR_READ_POLL:
      if (rq->size != 1)
        goto not_sup;
    case DEV_CHAR_READ:
    case DEV_CHAR_DISCARD:
    case DEV_CHAR_READ_NONBLOCK:
    case DEV_CHAR_READ_PARTIAL: {
      if (!(pv->enabled & EFM32_USART_STARTED_READ))
        goto not_sup;
      bool_t empty = dev_rq_queue_isempty(&pv->read_q);
      dev_char_rq_pushback(&pv->read_q, rq);
      if (empty)
        {
          efm32_usart_start(dev, EFM32_USART_STARTED_READ);
          kroutine_exec(&pv->read_kr);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      if (!(pv->enabled & EFM32_USART_STARTED_WRITE))
        goto not_sup;
      bool_t empty = dev_rq_queue_isempty(&pv->write_q);
      dev_char_rq_pushback(&pv->write_q, rq);
      dev->start_count |= EFM32_USART_STARTED_WRITE;
      if (empty)
        {
          efm32_usart_start(dev, EFM32_USART_STARTED_WRITE);
          efm32_usart_start_tx(dev, rq);
        }
      break;
    }

    default:
    not_sup:
      rq->error = -ENOTSUP;
      dev_char_rq_done(rq);
      break;
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

    case DEV_USE_SLEEP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      efm32_usart_stop(dev);
      return 0;
    }

    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      efm32_usart_start(dev, 0);
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      efm32_usart_try_stop(dev, 0);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static error_t efm32_usart_timer_init(struct efm32_usart_context_s *pv)
{
  uintptr_t taddr = pv->timer_addr;

  /* Stop timer and clear irq */
  cpu_mem_write_32(taddr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));
  cpu_mem_write_32(taddr + EFM32_TIMER_IEN_ADDR, 0);
  cpu_mem_write_32(taddr + EFM32_TIMER_IFC_ADDR, endian_le32(EFM32_TIMER_IFC_MASK));

  /* Prs init */
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  #error New code that uses directly UART1RXV as PRS input isn t implemented
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  uint32_t x = 0;

  switch (pv->usart_addr)
    {
    case 0x4000c000:
      EFM32_PRS_CTRL_SOURCESEL_SET(x, USART0);
      break;

    case 0x4000c400:
      EFM32_PRS_CTRL_SOURCESEL_SET(x, USART1);
      break;

    case 0x4000c800:
      EFM32_PRS_CTRL_SOURCESEL_SET(x, USART2);
      break;

    default:
      return -EINVAL;
    }

  EFM32_PRS_CTRL_SIGSEL_SET(x, 2 /* SIGSEL = USARTxRXDATAV */);
  cpu_mem_write_32(EFM32_PRS_ADDR + EFM32_PRS_CTRL_ADDR(CONFIG_DRIVER_EFM32_USART_CHAR_DMA_PRS_CHANNEL), endian_le32(x));
#endif

  /* Select HFPERCLK as timer source */
  x = EFM32_TIMER_CTRL_MODE(UP) |
      EFM32_TIMER_CTRL_OSMEN |
      EFM32_TIMER_CTRL_CLKSEL(PRESCHFPERCLK) |
      EFM32_TIMER_CTRL_RISEA(RELOADSTART) ;

  cpu_mem_write_32(taddr + EFM32_TIMER_CTRL_ADDR, endian_le32(x));

  /* Set input PRS channel as timer control */
  x = EFM32_TIMER_CC_CTRL_MODE(OFF) |
      EFM32_TIMER_CC_CTRL_FILT |
#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
      EFM32_TIMER_CC_CTRL_INSEL;
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
      EFM32_TIMER_CC_CTRL_INSEL(PRS);
#endif
  EFM32_TIMER_CC_CTRL_PRSSEL_SETVAL(x, CONFIG_DRIVER_EFM32_USART_CHAR_DMA_PRS_CHANNEL);

  cpu_mem_write_32(taddr + EFM32_TIMER_CC_CTRL_ADDR(0), endian_le32(x));

  struct dev_freq_s freq = {
    .num   = pv->cfg.baudrate,
    .denom = CONFIG_DRIVER_EFM32_USART_CHAR_DMA_TIMEOUT_SYMBOL,
  };

  /* Compute scale factor according to the requested frequency. */
  uint64_t scale = (pv->timer_freq.num * freq.denom) / (pv->timer_freq.denom * freq.num);
  uint_fast8_t div = bit_msb_index(scale);
  if (!scale || div > 25)
    return -ERANGE;

  if (div > 15)
    {
      div -= 15;
      scale >>= div;
    }

  /* Top value */
  cpu_mem_write_32(taddr + EFM32_TIMER_TOP_ADDR, endian_le32(scale - 1));

  /* Prescaler */
  x = endian_le32(cpu_mem_read_32(taddr + EFM32_TIMER_CTRL_ADDR));
  EFM32_TIMER_CTRL_PRESC_SETVAL(x, div);
  cpu_mem_write_32(taddr + EFM32_TIMER_CTRL_ADDR, endian_le32(x));

  cpu_mem_write_32(taddr + EFM32_TIMER_IEN_ADDR, EFM32_TIMER_IEN_OF);

  return 0;
}

#define logk_fatal logk

static DEV_INIT(efm32_usart_char_init)
{
  struct efm32_usart_context_s	*pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  uintptr_t uaddr;

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &uaddr, NULL);
  if (err)
    goto err_mem;

  err = device_res_get_uint(dev, DEV_RES_MEM, 1, &pv->timer_addr, NULL);
  if (err) {
    logk_fatal("No timer");
    goto err_irq;
  }

  pv->usart_addr = uaddr;

  /* setup pinmux */
  iomux_io_id_t pin[2];
  iomux_demux_t loc[2];
  err = device_iomux_setup(dev, "<rx? >tx?", loc, pin, NULL);
  if (err) {
    logk_fatal("Bad pins");
    goto err_mem;
  }

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  uint32_t enable = 0;
  uint32_t route = 0;

  if (loc[0] != IOMUX_INVALID_DEMUX)
    {
      pv->enabled |= EFM32_USART_STARTED_READ;
      enable |= EFM32_USART_ROUTEPEN_RXPEN;
      EFM32_USART_ROUTELOC0_RXLOC_SETVAL(route, loc[0]);
    }
  if (loc[1] != IOMUX_INVALID_DEMUX)
    {
      pv->enabled |= EFM32_USART_STARTED_WRITE;
      enable |= EFM32_USART_ROUTEPEN_TXPEN;
      EFM32_USART_ROUTELOC0_TXLOC_SETVAL(route, loc[1]);
    }
  if (enable == 0) {
    err = -EINVAL;
    logk_fatal("Bad pin routing");
    goto err_mem;
  }

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  uint32_t route = 0;
  if (loc[0] != IOMUX_INVALID_DEMUX)
    {
      pv->enabled |= EFM32_USART_STARTED_READ;
      route |= EFM32_USART_ROUTE_RXPEN;
    }
  if (loc[1] != IOMUX_INVALID_DEMUX)
    {
      pv->enabled |= EFM32_USART_STARTED_WRITE;
      route |= EFM32_USART_ROUTE_TXPEN;
    }

  EFM32_USART_ROUTE_LOCATION_SETVAL(route, loc[0] != IOMUX_INVALID_DEMUX ? loc[0] : loc[1]);

  if (route == 0) {
    err = -EINVAL;
    logk_fatal("Bad pin routing");
    goto err_mem;
  }
#else
# error
#endif

  /* init software fifos */
  dev_rq_queue_init(&pv->read_q);
  dev_rq_queue_init(&pv->write_q);

  err = dev_drv_clock_init(dev, &pv->clk_ep[0], 0, DEV_CLOCK_EP_FREQ_NOTIFY |
                           DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->freq);
  if (err) {
    logk_fatal("Bad clock config");
    goto err_irq;
  }

  /* wait for current TX to complete */
  if (cpu_mem_read_32(uaddr + EFM32_USART_STATUS_ADDR)
      & endian_le32(EFM32_USART_STATUS_TXENS))
      while (!(cpu_mem_read_32(uaddr + EFM32_USART_STATUS_ADDR)
               & endian_le32(EFM32_USART_STATUS_TXBL)))
        ;

  /* disable and clear the uart */
  cpu_mem_write_32(uaddr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

  cpu_mem_write_32(uaddr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX));

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  cpu_mem_write_32(uaddr + EFM32_USART_ROUTELOC0_ADDR, endian_le32(route));
  cpu_mem_write_32(uaddr + EFM32_USART_ROUTEPEN_ADDR, endian_le32(enable));
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
  cpu_mem_write_32(uaddr + EFM32_USART_ROUTE_ADDR, endian_le32(route));
#else
# error
#endif

  /* configure */
  cpu_mem_write_32(uaddr + EFM32_USART_CTRL_ADDR, endian_le32(EFM32_USART_CTRL_OVS(X4)));

  /* set config */
  if (device_get_res_uart(dev, &pv->cfg))
    {
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
  cpu_mem_write_32(uaddr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_TXEN | EFM32_USART_CMD_RXBLOCKDIS));

  err = dev_drv_clock_init(dev, &pv->clk_ep[1], 1, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->timer_freq);
  if (err) {
    logk_fatal("Unable to gate clock for TIMER");
    goto err_irq;
  }

  logk("num: %llu denom: %llu", pv->timer_freq.num, pv->timer_freq.denom);

  err = dev_drv_clock_init(dev, &pv->clk_ep[2], 2, DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, NULL);
  if (err) {
    logk_fatal("Unable to gate clock for PRS");
    goto err_irq;
  }

  err = efm32_usart_timer_init(pv);
  if (err) {
    logk_fatal("Unable to setup timer");
    goto err_irq;
  }

  uint32_t read_link, write_link;
  uint32_t read_mask, write_mask;

  pv->dev = dev;

  err = device_res_get_dma(dev, 0, &read_mask, &read_link);
  if (err) {
    logk_fatal("Unable to get DMA read link");
    goto err_irq;
  }
  
  err = device_res_get_dma(dev, 1, &write_mask, &write_link);
  if (err) {
    logk_fatal("Unable to get DMA write link");
    goto err_irq;
  }

  err = device_get_param_dev_accessor(dev, "dma", &pv->dma.base, DRIVER_CLASS_DMA);
  if (err) {
    logk_fatal("Unable to get DMA");
    goto err_irq;
  }

  struct dev_dma_rq_s *rq = &pv->dma_rd_rq;
  struct dev_dma_desc_s *desc = pv->dma_rd_desc;

  /* RX */

  for (uint8_t i = 0; i < 2; i++)
    {
      desc = pv->dma_rd_desc + i;
      desc->src.reg.addr = uaddr + EFM32_USART_RXDATA_ADDR;
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

  desc->dst.reg.addr = uaddr + EFM32_USART_TXDATA_ADDR;
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
  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_usart_timer_irq);

  err = device_irq_source_link(dev, &pv->irq_ep, 1, -1);
  if (err) {
    logk_fatal("Unable to setup IRQ");
    goto err_fifo;
  }

  pv->rx_state = EFM32_USART_RX_STOPPED;

#ifdef CONFIG_DEVICE_CLOCK_GATING
  dev_clock_sink_gate(&pv->clk_ep[0], DEV_CLOCK_EP_POWER);
#endif

  return 0;

 err_irq:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
 err_fifo:
  dev_rq_queue_destroy(&pv->read_q);
  dev_rq_queue_destroy(&pv->write_q);
 err_mem:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(efm32_usart_char_cleanup)
{
  struct efm32_usart_context_s	*pv = dev->drv_pv;

  if (dev->start_count)
    return -EBUSY;

  /* stop rx DMA */
  if (pv->rx_state != EFM32_USART_RX_STOPPED)
    ensure(DEVICE_OP(&pv->dma, cancel, &pv->dma_rd_rq) == 0);

  /* disable the timer */
  cpu_mem_write_32(pv->timer_addr + EFM32_TIMER_IEN_ADDR, 0);
  cpu_mem_write_32(pv->timer_addr + EFM32_TIMER_CMD_ADDR, endian_le32(EFM32_TIMER_CMD_STOP));

  /* disable the usart */
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  cpu_mem_write_32(pv->usart_addr + EFM32_USART_CMD_ADDR,
                   endian_le32(EFM32_USART_CMD_RXDIS | EFM32_USART_CMD_TXDIS));

  dev_drv_clock_cleanup(dev, &pv->clk_ep[0]);
  dev_drv_clock_cleanup(dev, &pv->clk_ep[1]);
  dev_drv_clock_cleanup(dev, &pv->clk_ep[2]);

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

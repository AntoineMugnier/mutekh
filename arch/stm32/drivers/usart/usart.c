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
#include <device/class/uart.h>

#include <cpp/device/helpers.h>
#include <arch/stm32f4xx_rcc.h>
#include <arch/stm32_usart.h>
#include <arch/stm32_memory_map.h>

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
# include <gct_platform.h>
# include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_usart_fifo RING
GCT_CONTAINER_TYPES(usart_fifo, uint8_t, CONFIG_DRIVER_STM32_USART_SWFIFO);
GCT_CONTAINER_FCNS(usart_fifo, static inline, usart_fifo,
                   init, destroy, pop_array, pushback, pushback_array, isempty, pop);
#endif

extern uint32_t stm32f4xx_clock_freq_ahb1;
extern uint32_t stm32f4xx_clock_freq_apb1;
extern uint32_t stm32f4xx_clock_freq_apb2;


struct stm32_usart_context_s
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

  /* interrupt end-points (TX and RX on the same wire). */
  struct dev_irq_ep_s   irq_ep[1];

  bool_t                read_started:1;
  bool_t                write_started:1;
};


static
void stm32_usart_try_read(struct device_s *dev)
{
  struct stm32_usart_context_s *pv = dev->drv_pv;
  struct dev_char_rq_s         *rq;

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->read_q))))
    {
      size_t size = 0;

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
      /* read as much as possible from fifo. */
      size = usart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

    /* read characters if the request asked for more. */
      if (size < rq->size &&
          DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, RXNE))
        rq->data[size++] =
          DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, DR, DATA);

      /* if a data was read, then process the read request. */
      if (size)
        {
          rq->size -= size;
          rq->error = 0;
          rq->data += size;

          if (rq->type == DEV_CHAR_READ_PARTIAL || rq->size == 0)
            {
              dev_request_queue_pop(&pv->read_q);
              lock_release(&dev->lock);
              kroutine_exec(&rq->base.kr, 0);
              lock_spin(&dev->lock);
              /* look for another pending read request. */
              continue;
            }
        }

#if defined(CONFIG_DEVICE_IRQ)
      /* otherwise, return and wait for another interrupt. */
      return;
#endif
    }

  pv->read_started = 0;

  /* if no request need the data, discard it or save it in the read fifo. */
  if (DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, RXNE))
    {
      __unused__ uint8_t c =
        DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, DR, DATA);
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

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  /* try to write as much as possible data from the fifo first. */
  while (!usart_fifo_isempty(&pv->write_fifo) &&
      DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, TXE))
    {
      uint8_t c = usart_fifo_pop(&pv->write_fifo);
      DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, DR, DATA, c);
    }
#endif

  while ((rq = dev_char_rq_s_cast(dev_request_queue_head(&pv->write_q))))
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
          if (size < rq->size &&
              DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, TXE))
            {
              DEVICE_REG_FIELD_UPDATE_DEV(
                USART, pv->addr,
                DR, DATA,
                rq->data[size++]
              );
            }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_STM32_USART_SWFIFO > 0
        }

      if (size < rq->size)
        {
          size += usart_fifo_pushback_array(
            &pv->write_fifo,
            &rq->data[size],
            rq->size - size
          );

          /* wait for the next write interrupt as the fifo is not empty
             anymore. */
          DEVICE_REG_FIELD_SET_DEV(USART, pv->addr, CR1, TXEIE);
        }
#endif

      if (size)
        {
          /* update request size and data pointer. */
          rq->size -= size;
          rq->data += size;
          rq->error = 0;

          if (rq->type == DEV_CHAR_WRITE_PARTIAL || rq->size == 0)
            {
              dev_request_queue_pop(&pv->write_q);

              lock_release(&dev->lock);
              kroutine_exec(&rq->base.kr, 0);
              lock_spin(&dev->lock);

              /* look for another pending write request. */
              continue;
            }
        }

#if defined(CONFIG_DEVICE_IRQ)
      /* wait for the next interrupt, when the controller will be ready to
         send. */
      DEVICE_REG_FIELD_SET_DEV(USART, pv->addr, CR1, TXEIE);
      return;
#endif
    }

  pv->write_started = 0;

#if defined(CONFIG_DEVICE_IRQ)
  /* if there is no more write request in the queue or fifo, then
     disable TX interrupt. */
# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  if (usart_fifo_isempty(&pv->write_fifo))
# endif
    DEVICE_REG_FIELD_CLR_DEV(USART, pv->addr, CR1, TXEIE);
#endif
}

static
DEV_CHAR_REQUEST(stm32_usart_request)
{
  struct device_s              *dev = accessor->dev;
  struct stm32_usart_context_s *pv  = dev->drv_pv;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
  {
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ: {
      dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));
      if (!pv->read_started)
        {
          pv->read_started = 1;
          stm32_usart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE: {
      dev_request_queue_pushback(&pv->write_q, dev_char_rq_s_base(rq));
      if (!pv->write_started)
        {
          pv->write_started = 1;
          stm32_usart_try_write(dev);
        }
      break;
    }
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

#if defined(CONFIG_DEVICE_IRQ)

static
DEV_IRQ_EP_PROCESS(stm32_usart_irq)
{
  struct device_s              *dev = ep->dev;
  struct stm32_usart_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
  {
    uint32_t ir = DEVICE_REG_VALUE_DEV(USART, pv->addr, SR);

    /* if the controller has no pending data and cannot send data, then
     * break and wait for another interrupt.
     */
    if ((ir & ( STM32_USART_SR_RXNE | STM32_USART_SR_TXE )) == 0)
      break;

    if (ir & STM32_USART_SR_TXE)
      stm32_usart_try_write(dev);

    if (ir & STM32_USART_SR_RXNE)
      stm32_usart_try_read(dev);

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
    if (usart_fifo_isempty(&pv->write_fifo))
#else
    if (dev_request_queue_isempty(&pv->write_q))
#endif
      break;
  }

  lock_release(&dev->lock);
}

#endif

static const struct driver_char_s stm32_usart_char_cls =
{
  .class_    = DRIVER_CLASS_CHAR,
  .f_request = stm32_usart_request
};

static
error_t stm32_usart_check_config(struct dev_uart_config_s *cfg)
{
  /* check data bits. */
  switch (cfg->data_bits)
    {
    default:
      return -ENOTSUP;

    case 8:
    case 9:
      break;
    }

  /* check stop bits (all supported). */

  /* check parity (not supported). */
  if (cfg->parity != DEV_UART_PARITY_NONE)
    return -ENOTSUP;

  /* check flow control (not supported). */
  if (cfg->flow_ctrl)
    return -ENOTSUP;

  return 0;
}

static
error_t stm32_usart_config_simple(struct device_s          *dev,
                                  struct dev_uart_config_s *cfg)
{
  struct stm32_usart_context_s *pv = dev->drv_pv;

  /* check baudrate. */
  error_t err = stm32_usart_check_config(cfg);
  if (err)
    return err;

  /* configure data, stop and parity. */
  switch (cfg->data_bits)
  {
  default:
    break;

  case 8:
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, M, 8_BITS);
    break;

  case 9:
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, M, 9_BITS);
    break;
  }

  switch (cfg->stop_bits)
  {
  default:
    break;

  case 1:
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR2, STOP, 1_BIT);
    break;

  case 2:
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR2, STOP, 2_BITS);
    break;
  }

  switch (cfg->parity)
  {
  default:
    break;

  case DEV_UART_PARITY_NONE:
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, PCE, NONE);
    break;

  case DEV_UART_PARITY_ODD:
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, PCE, ENABLE);
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, PS, ODD);
    break;

  case DEV_UART_PARITY_EVEN:
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, PCE, ENABLE);
    DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, PS, EVEN);
    break;
  }

  /* configure the baudrate. */
  uint64_t brr = 0;
  switch (pv->addr)
  {
  default:
    assert(!"unknown USART base address");
    break;

  case STM32_USART1_ADDR:
  case STM32_USART6_ADDR:
    brr = (((uint64_t)stm32f4xx_clock_freq_apb2) << 8) / (cfg->baudrate << 8);
    break;

  case STM32_USART2_ADDR:
    brr = (((uint64_t)stm32f4xx_clock_freq_apb1) << 8) / (cfg->baudrate << 8);
    break;
  }

  DEVICE_REG_UPDATE_DEV(USART, pv->addr, BRR, (uint32_t)brr);
  return 0;
}

static
DEV_UART_CONFIG(stm32_usart_config)
{
  struct device_s              *dev = accessor->dev;
  struct stm32_usart_context_s *pv  = dev->drv_pv;

  /* wait for previous TX to complete. */
  if (DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, CR1, TE))
    {
      while (!DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, TC));
      DEVICE_REG_FIELD_CLR_DEV(USART, pv->addr, SR, TC);
    }

  /* disable the usart. */
  bool_t enabled = DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, CR1, UE);
  DEVICE_REG_FIELD_CLR_DEV(USART, pv->addr, CR1, UE);

  error_t err = stm32_usart_config_simple(dev, cfg);

  /* (re-)enable the usart. */
  if (!err || enabled)
    DEVICE_REG_FIELD_SET_DEV(USART, pv->addr, CR1, UE);

#if defined(CONFIG_DEBUG)
  if (err && enabled)
    printk("uart: configuration left unchanged.\n");
#endif

  return err;
}

static const struct driver_uart_s stm32_usart_uart_cls =
{
  .class_   = DRIVER_CLASS_UART,
  .f_config = &stm32_usart_config
};

static DEV_INIT(stm32_usart_init);
static DEV_CLEANUP(stm32_usart_cleanup);

const struct driver_s stm32_usart_drv =
{
  .desc      = "STM32 USART",
  .f_init    = stm32_usart_init,
  .f_cleanup = stm32_usart_cleanup,
  .classes   = {
    &stm32_usart_char_cls,
    &stm32_usart_uart_cls,
    0
  }
};

REGISTER_DRIVER(stm32_usart_drv);

/* ****************************************************************************
 *
 * Configure clock and GPIO with alternate functions.
 *
 * The clock must be enabled on the buses on which the USARTS are connected.
 * The GPIO must be configured with USART alternate function on both TX and RX
 * pins.
 */

static inline
void stm32_usart_clock_init(struct device_s *dev)
{
  struct stm32_usart_context_s *pv = dev->drv_pv;

  assert(pv != 0);

  switch (pv->addr)
  {
  default:
    break;

  case STM32_USART1_ADDR:
    DEVICE_REG_FIELD_SET(RCC, , APB2ENR, USART1EN);
    break;

  case STM32_USART2_ADDR:
    DEVICE_REG_FIELD_SET(RCC, , APB1ENR, USART2EN);
    break;

  case STM32_USART6_ADDR:
    DEVICE_REG_FIELD_SET(RCC, , APB2ENR, USART6EN);
    break;
  }
}

/* ************************************************************************* */

static
DEV_INIT(stm32_usart_init)
{
  struct stm32_usart_context_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  pv->read_started = pv->write_started = 0;

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* wait for previous TX to complete. */
  if (DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, CR1, TE))
    {
      while (!DEVICE_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, TC));
      DEVICE_REG_FIELD_CLR_DEV(USART, pv->addr, SR, TC);
    }

  /* disable and reset the usart. */
  DEVICE_REG_UPDATE_DEV(USART, pv->addr, CR1, 0);
  DEVICE_REG_UPDATE_DEV(USART, pv->addr, CR2, 0);
  DEVICE_REG_UPDATE_DEV(USART, pv->addr, CR3, 0);

  /* initialize request queues. */
  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_init(&pv->read_fifo);
# if defined(CONFIG_DEVICE_IRQ)
  usart_fifo_init(&pv->write_fifo);
# endif
#endif

  /* configure clocks. */
  stm32_usart_clock_init(dev);

  /* configure gpio. */
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, "<rx? >tx?", loc, NULL, NULL))
    goto err_fifo;

  if (loc[0] == IOMUX_INVALID_DEMUX && loc[1] == IOMUX_INVALID_DEMUX)
    goto err_fifo;

  if (loc[0] != IOMUX_INVALID_DEMUX)
    DEVICE_REG_FIELD_SET_DEV(USART, pv->addr, CR1, RE);
  if (loc[1] != IOMUX_INVALID_DEMUX)
    DEVICE_REG_FIELD_SET_DEV(USART, pv->addr, CR1, TE);

  /* configure over-sampling. */
  DEVICE_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, OVER8, 16);
  DEVICE_REG_FIELD_CLR_DEV(USART, pv->addr, CR3, ONEBIT);

  /* check for default configuration resource. */
  struct dev_resource_s *r = device_res_get(dev, DEV_RES_UART, 0);

  error_t err = -1;
  if (r != NULL)
    {
      struct dev_uart_config_s cfg =
        {
          .baudrate    = r->u.uart.baudrate,
          .data_bits   = r->u.uart.data_bits,
          .stop_bits   = r->u.uart.stop_bits,
          .parity      = r->u.uart.parity,
          .flow_ctrl   = r->u.uart.flow_ctrl,
          .half_duplex = r->u.uart.half_duplex
        };

      err = stm32_usart_config_simple(dev, &cfg);
      if (err)
        printk("uart: failed to configure uart with default configuration.\n");
    }

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_init(
    dev,
    pv->irq_ep,
    1,
    &stm32_usart_irq,
    DEV_IRQ_SENSE_HIGH_LEVEL
  );

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto err_fifo;

  /* enable RX and TX irqs. */
  DEVICE_REG_FIELD_SET_DEV(USART, pv->addr, CR1, RXNEIE);
#endif

  /* enable usart if configured. */
  if (!err)
    DEVICE_REG_FIELD_SET_DEV(USART, pv->addr, CR1, UE);

  /* link the driver. */
  dev->drv = &stm32_usart_drv;

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_fifo:
#if defined(CONFIG_DEVICE_IRQ)
# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_destroy(&pv->read_fifo);
  usart_fifo_destroy(&pv->write_fifo);
# endif
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);
#endif

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32_usart_cleanup)
{
  struct stm32_usart_context_s *pv = dev->drv_pv;

  /* disable and reset the usart. */
  DEVICE_REG_UPDATE_DEV(USART, pv->addr, CR1, 0);
  DEVICE_REG_UPDATE_DEV(USART, pv->addr, CR2, 0);
  DEVICE_REG_UPDATE_DEV(USART, pv->addr, CR3, 0);

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_unlink(dev, pv->irq_ep, 1);

# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_destroy(&pv->read_fifo);
  usart_fifo_destroy(&pv->write_fifo);
# endif
#endif

  /* destroy request queues. */
  dev_request_queue_destroy(&pv->read_q);
  dev_request_queue_destroy(&pv->write_q);

  mem_free(pv);
}


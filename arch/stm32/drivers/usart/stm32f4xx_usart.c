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

#include <arch/stm32f4xx_rcc.h>
#include <arch/stm32f4xx_usart.h>

#include <arch/stm32f4xx_helpers.h>
#include <arch/stm32f4xx_memory_map.h>

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
# include <hexo/gpct_platform_hexo.h>
# include <gpct/cont_ring.h>

CONTAINER_TYPE(usart_fifo, RING, uint8_t, CONFIG_DRIVER_STM32_USART_SWFIFO);
CONTAINER_FUNC(usart_fifo, RING, static inline, usart_fifo);
#endif

extern uint32_t stm32f4xx_clock_freq_ahb1;
extern uint32_t stm32f4xx_clock_freq_apb1;
extern uint32_t stm32f4xx_clock_freq_apb2;


struct stm32f4xx_usart_context_s
{
  /* usart controller address. */
  uintptr_t             addr;

  /* usart request queue for read requests. */
  dev_char_queue_root_t read_q;

  /* usart request queue for write requests. */
  dev_char_queue_root_t write_q;

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
};

static void stm32f4xx_usart_try_read(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;
  struct dev_char_rq_s              *rq;

  while ((rq = dev_char_queue_head(&pv->read_q)))
    {
      size_t size = 0;

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
      /* read as much as possible from fifo. */
      size = usart_fifo_pop_array(&pv->read_fifo, rq->data, rq->size);
#endif

    /* read characters if the request asked for more. */
      if (size < rq->size &&
          STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, RXNE))
        rq->data[size++] =
          STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, DR, DATA);

      /* if a data was read, then process the read request. */
      if (size)
        {
          rq->size -= size;
          rq->error = 0;

          if (rq->callback(rq, size) || rq->size == 0)
            {
              dev_char_queue_remove(&pv->read_q, rq);

              /* look for another pending read request. */
              continue;
            }

          rq->data += size;
        }

#if defined(CONFIG_DEVICE_IRQ)
      /* otherwise, return and wait for another interrupt. */
      return;
#endif
    }

  /* if no request need the data, discard it or save it in the read fifo. */
  if (STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, RXNE))
    {
      __unused__ uint8_t c =
        STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, DR, DATA);
#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
      usart_fifo_pushback(&pv->read_fifo, c);
#endif
    }
}

static void stm32f4xx_usart_try_write(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;
  struct dev_char_rq_s              *rq;

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  /* try to write as much as possible data from the fifo first. */
  if (!usart_fifo_isempty(&pv->write_fifo) &&
      STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, TC))
    {
      uint8_t c = usart_fifo_pop(&pv->write_fifo);
      STM32F4xx_REG_FIELD_UPDATE_DEV(USART, pv->addr, DR, DATA, c);
    }
#endif

  while ((rq = dev_char_queue_head(&pv->write_q)))
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
              STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, TC))
          {
            STM32F4xx_REG_FIELD_UPDATE_DEV(
              USART,
              pv->addr,
              DR,
              DATA,
              rq->data[size++]
            );
          }

#if defined(CONFIG_DEVICE_IRQ) && CONFIG_DRIVER_STM32_USART_SWFIFO > 0
        }

      if (size < rq->size)
        size += usart_fifo_pushback_array(
          &pv->write_fifo,
          &rq->data[size],
          rq->size - size
        );
#endif

      if (size)
        {
          rq->size -= size;
          rq->error = 0;

          if (rq->callback(rq, size) || rq->size == 0)
            {
              dev_char_queue_remove(&pv->write_q, rq);

              /* look for another pending write request. */
              continue;
            }

          /* update the buffer pointer. */
          rq->data += size;
        }

#if defined(CONFIG_DEVICE_IRQ)
      /* wait for the next interrupt, when the controller will be ready to
         send. */
      STM32F4xx_REG_FIELD_SET_DEV(USART, pv->addr, CR1, TCIE);
      return;
#endif
    }

#if defined(CONFIG_DEVICE_IRQ)
  /* if there is no more write request in the queue or fifo, then
     disable TX interrupt. */
# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  if (usart_fifo_isempty(&pv->write_fifo))
# endif
  {
    STM32F4xx_REG_FIELD_CLR_DEV(USART, pv->addr, SR, TC);
    STM32F4xx_REG_FIELD_CLR_DEV(USART, pv->addr, CR1, TCIE);
  }
#endif
}

static DEVCHAR_REQUEST(stm32f4xx_usart_request)
{
  struct device_s                   *dev = cdev->dev;
  struct stm32f4xx_usart_context_s  *pv  = dev->drv_pv;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
  {
  default:
    break;

  case DEV_CHAR_READ: {
#if defined(CONFIG_DEVICE_IRQ)
    bool_t empty = dev_char_queue_isempty(&pv->read_q);
#endif
    dev_char_queue_pushback(&pv->read_q, rq);
#if defined(CONFIG_DEVICE_IRQ)
    if (empty)
#endif
    stm32f4xx_usart_try_read(dev);
    break;
  }

  case DEV_CHAR_WRITE: {
#if defined(CONFIG_DEVICE_IRQ)
    bool_t empty = dev_char_queue_isempty(&pv->write_q);
#endif
    dev_char_queue_pushback(&pv->write_q, rq);
#if defined(CONFIG_DEVICE_IRQ)
    if (empty)
#endif
    stm32f4xx_usart_try_write(dev);
    break;
  }
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

#if defined(CONFIG_DEVICE_IRQ)

static DEV_IRQ_EP_PROCESS(stm32f4xx_usart_irq)
{
  struct device_s                   *dev = ep->dev;
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
  {
    uint32_t ir = STM32F4xx_REG_VALUE_DEV(USART, pv->addr, SR);

    /* if the controller has no pending data and cannot send data, then
     * break and wait for another interrupt.
     */
    if ((ir & ( STM32F4xx_USART_SR_RXNE | STM32F4xx_USART_SR_TC )) == 0)
      break;

    if (ir & STM32F4xx_USART_SR_TC)
      stm32f4xx_usart_try_write(dev);

    if (ir & STM32F4xx_USART_SR_RXNE)
      stm32f4xx_usart_try_read(dev);
  }

  lock_release(&dev->lock);
}

#endif

static const struct driver_char_s stm32f4xx_usart_char_drv =
{
  .class_       = DRIVER_CLASS_CHAR,
  .f_request    = stm32f4xx_usart_request
};

static DEV_INIT(stm32f4xx_usart_init);
static DEV_CLEANUP(stm32f4xx_usart_cleanup);

const struct driver_s stm32f4xx_usart_drv =
{
  .desc         = "STM32F4xx USART",
  .f_init       = stm32f4xx_usart_init,
  .f_cleanup    = stm32f4xx_usart_cleanup,
  .classes      = {
    &stm32f4xx_usart_char_drv,
    0
  }
};

REGISTER_DRIVER(stm32f4xx_usart_drv);

/* ****************************************************************************
 *
 * Configure clock and GPIO with alternate functions.
 *
 * The clock must be enabled on the buses on which the USARTS are connected.
 * The GPIO must be configured with USART alternate function on both TX and RX
 * pins.
 */

#define STM32F4xx_USART1_BASE       0x40011000
#define STM32F4xx_USART2_BASE       0x40004400
#define STM32F4xx_USART6_BASE       0x40011400

static inline void stm32f4xx_usart_clock_init(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;

  assert(pv != 0);

  switch (pv->addr)
  {
  default:
    break;

  case STM32F4xx_USART1_BASE:
    STM32F4xx_REG_FIELD_SET(RCC, , APB2ENR, USART1EN);
    break;

  case STM32F4xx_USART2_BASE:
    STM32F4xx_REG_FIELD_SET(RCC, , APB1ENR, USART2EN);
    break;

  case STM32F4xx_USART6_BASE:
    STM32F4xx_REG_FIELD_SET(RCC, , APB2ENR, USART6EN);
    break;
  }
}

/* ************************************************************************* */

static DEV_INIT(stm32f4xx_usart_init)
{
  struct stm32f4xx_usart_context_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* wait for previous TX to complete. */
  if (STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, CR1, TE))
    while (!STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, SR, TC));

  /* disable and reset the usart. */
  STM32F4xx_REG_UPDATE_DEV(USART, pv->addr, CR1, 0);
  STM32F4xx_REG_UPDATE_DEV(USART, pv->addr, CR2, 0);
  STM32F4xx_REG_UPDATE_DEV(USART, pv->addr, CR3, 0);

  /* initialize request queues. */
  dev_char_queue_init(&pv->read_q);
  dev_char_queue_init(&pv->write_q);

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_init(&pv->read_fifo);
# if defined(CONFIG_DEVICE_IRQ)
  usart_fifo_init(&pv->write_fifo);
# endif
#endif

  /* configure clocks. */
  stm32f4xx_usart_clock_init(dev);

  /* configure gpio. */
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, "<rx? >tx?", loc, NULL, NULL))
    goto err_fifo;

  if (loc[0] == IOMUX_INVALID_DEMUX && loc[1] == IOMUX_INVALID_DEMUX)
    goto err_fifo;

  /* configure usart . */
  STM32F4xx_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, M,    8_BITS);
  STM32F4xx_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, PCE,  NONE);
  STM32F4xx_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR2, STOP, 1_BIT);

  if (loc[0] != IOMUX_INVALID_DEMUX)
    STM32F4xx_REG_FIELD_SET_DEV(USART, pv->addr, CR1, RE);
  if (loc[1] != IOMUX_INVALID_DEMUX)
    STM32F4xx_REG_FIELD_SET_DEV(USART, pv->addr, CR1, TE);

  /* configure baudrate. */
  STM32F4xx_REG_FIELD_UPDATE_DEV(USART, pv->addr, CR1, OVER8, 16);

  uint32_t brr = 0;
  switch (pv->addr)
  {
  default:
    assert(0 && "unknown USART base address");
    break;

  case STM32F4xx_USART1_BASE:
  case STM32F4xx_USART6_BASE:
    brr = ((int)(stm32f4xx_clock_freq_apb2 / 115200.0 + 0.5)) & 0xffff;
    break;

  case STM32F4xx_USART2_BASE:
    brr = ((int)(stm32f4xx_clock_freq_apb1 / 115200.0 + 0.5)) & 0xffff;
    break;
  }

  STM32F4xx_REG_UPDATE_DEV(USART, pv->addr, BRR, brr);

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_init(
    dev,
    pv->irq_ep,
    1,
    &stm32f4xx_usart_irq,
    DEV_IRQ_SENSE_HIGH_LEVEL
  );

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto err_fifo;

  /* enable RX irq. TX irq is activated on on demand. */
  if (STM32F4xx_REG_FIELD_VALUE_DEV(USART, pv->addr, CR1, RE))
    STM32F4xx_REG_FIELD_SET_DEV(USART, pv->addr, CR1, RXNEIE);
#endif

  /* enable usart. */
  STM32F4xx_REG_FIELD_SET_DEV(USART, pv->addr, CR1, UE);

  /* link the driver. */
  dev->drv = &stm32f4xx_usart_drv;

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#if defined(CONFIG_DEVICE_IRQ)
err_fifo:
# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_destroy(&pv->read_fifo);
  usart_fifo_destroy(&pv->write_fifo);
# endif
  dev_char_queue_destroy(&pv->read_q);
  dev_char_queue_destroy(&pv->write_q);
#endif

err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(stm32f4xx_usart_cleanup)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;

  /* disable and reset the usart. */
  STM32F4xx_REG_UPDATE_DEV(USART, pv->addr, CR1, 0);
  STM32F4xx_REG_UPDATE_DEV(USART, pv->addr, CR2, 0);
  STM32F4xx_REG_UPDATE_DEV(USART, pv->addr, CR3, 0);

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_unlink(dev, pv->irq_ep, 1);

# if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
  usart_fifo_destroy(&pv->read_fifo);
  usart_fifo_destroy(&pv->write_fifo);
# endif
#endif

  /* destroy request queues. */
  dev_char_queue_destroy(&pv->read_q);
  dev_char_queue_destroy(&pv->write_q);

  mem_free(pv);
}


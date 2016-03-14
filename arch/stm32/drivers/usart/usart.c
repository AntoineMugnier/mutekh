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

#include <arch/stm32/usart.h>

#if CONFIG_DRIVER_STM32_USART_SWFIFO > 0
# include <gct_platform.h>
# include <gct/container_ring.h>
#define GCT_CONTAINER_ALGO_usart_fifo RING
GCT_CONTAINER_TYPES(usart_fifo, uint8_t, CONFIG_DRIVER_STM32_USART_SWFIFO);
GCT_CONTAINER_FCNS(usart_fifo, static inline, usart_fifo,
                   init, destroy, pop_array, pushback, pushback_array, isempty, pop);
#endif

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
  struct dev_irq_src_s   irq_ep[1];

  bool_t                read_started:1;
  bool_t                write_started:1;

#if !defined(CONFIG_DEVICE_CLOCK)
  struct dev_freq_s     busfreq;
#endif
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
          STM32_USART_SR_RXNE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))) ))
        rq->data[size++] =
          STM32_USART_DR_DATA_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_DR_ADDR) ))) );

      /* if a data was read, then process the read request. */
      if (size)
        {
          rq->size -= size;
          rq->error = 0;
          rq->data += size;

          if ((rq->type & _DEV_CHAR_PARTIAL) || rq->size == 0)
            {
              dev_request_queue_pop(&pv->read_q);
              kroutine_exec(&rq->base.kr);
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
  if (STM32_USART_SR_RXNE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))) ))
    {
      __unused__ uint8_t c =
        STM32_USART_DR_DATA_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_DR_ADDR) ))) );
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
      STM32_USART_SR_TXE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))) ))
    {
      uint8_t c = usart_fifo_pop(&pv->write_fifo);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_DR_ADDR) ))); STM32_USART_DR_DATA_SET( (_reg), c ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_DR_ADDR) ), endian_le32(_reg) ); } while (0);
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
              STM32_USART_SR_TXE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))) ))
            {
              do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_DR_ADDR) ))); STM32_USART_DR_DATA_SET( (_reg), rq->data[size++] ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_DR_ADDR) ), endian_le32(_reg) ); } while (0)



               ;
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
          do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_TXEIE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
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
              dev_request_queue_pop(&pv->write_q);

              kroutine_exec(&rq->base.kr);

              /* look for another pending write request. */
              continue;
            }
        }

#if defined(CONFIG_DEVICE_IRQ)
      /* wait for the next interrupt, when the controller will be ready to
         send. */
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_TXEIE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
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
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_TXEIE_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
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
      dev_request_queue_pushback(&pv->read_q, dev_char_rq_s_base(rq));
      if (!pv->read_started)
        {
          pv->read_started = 1;
          stm32_usart_try_read(dev);
        }
      break;
    }

    case DEV_CHAR_WRITE_PARTIAL_FLUSH:
    case DEV_CHAR_WRITE_FLUSH:
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
    default:
      err = -ENOTSUP;
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (err)
    {
      rq->error = err;
      kroutine_exec(&rq->base.kr);
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
    uint32_t ir = endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_USART_SR_ADDR) )));

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
error_t stm32_usart_config_simple(struct stm32_usart_context_s *pv,
                                  struct dev_uart_config_s *cfg)
{
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
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_M_SET( (_reg), 8_BITS ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    break;

  case 9:
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_M_SET( (_reg), 9_BITS ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    break;
  }

  switch (cfg->stop_bits)
  {
  default:
    break;

  case 1:
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR2_ADDR) ))); STM32_USART_CR2_STOP_SET( (_reg), 1_BIT ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR2_ADDR) ), endian_le32(_reg) ); } while (0);
    break;

  case 2:
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR2_ADDR) ))); STM32_USART_CR2_STOP_SET( (_reg), 2_BITS ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR2_ADDR) ), endian_le32(_reg) ); } while (0);
    break;
  }

  switch (cfg->parity)
  {
  default:
    break;

  case DEV_UART_PARITY_NONE:
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_PCE_SET( (_reg), NONE ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    break;

  case DEV_UART_PARITY_ODD:
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_PCE_SET( (_reg), ENABLE ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_PS_SET( (_reg), ODD ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    break;

  case DEV_UART_PARITY_EVEN:
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_PCE_SET( (_reg), ENABLE ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_PS_SET( (_reg), EVEN ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    break;
  }

  uint64_t brr = 0;
#if !defined(CONFIG_DEVICE_CLOCK)
  /* configure the baudrate. */
  brr = (uint64_t)pv->busfreq.num / cfg->baudrate / pv->busfreq.denom;
#endif

  cpu_mem_write_32( ( (((pv->addr))) + (STM32_USART_BRR_ADDR) ), endian_le32((uint32_t)brr) );
  return 0;
}

static
DEV_UART_CONFIG(stm32_usart_config)
{
  struct device_s              *dev = accessor->dev;
  struct stm32_usart_context_s *pv  = dev->drv_pv;

  /* wait for previous TX to complete. */
  if (STM32_USART_CR1_TE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))) ))
    {
      while (!STM32_USART_SR_TC_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))) ));
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))); STM32_USART_SR_TC_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ), endian_le32(_reg) ); } while (0);
    }

  /* disable the usart. */
  bool_t enabled = STM32_USART_CR1_UE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))) );
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_UE_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

  error_t err = stm32_usart_config_simple(pv, cfg);

  /* (re-)enable the usart. */
  if (!err || enabled)
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_UE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

#if defined(CONFIG_DEBUG)
  if (err && enabled)
    printk("uart: configuration left unchanged.\n");
#endif

  return err;
}

static DEV_INIT(stm32_usart_init);
static DEV_CLEANUP(stm32_usart_cleanup);

#define stm32_usart_use dev_use_generic

DRIVER_DECLARE(stm32_usart_drv, 0, "STM32 USART", stm32_usart,
               DRIVER_CHAR_METHODS(stm32_usart),
               DRIVER_UART_METHODS(stm32_usart));

DRIVER_REGISTER(stm32_usart_drv);

/* ************************************************************************* */

static
DEV_INIT(stm32_usart_init)
{
  struct stm32_usart_context_s *pv;


  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#if 0
  /* wait for previous TX to complete. */
  if (STM32_USART_CR1_TE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))) ))
    {
      while (!STM32_USART_SR_TC_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))) ));
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ))); STM32_USART_SR_TC_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_SR_ADDR) ), endian_le32(_reg) ); } while (0);
    }
#endif

#if !defined(CONFIG_DEVICE_CLOCK)
  if (device_get_res_freq(dev, &pv->busfreq, 0))
#endif
    goto err_mem;

  /* setup startup state. */
  pv->read_started = pv->write_started = 0;

  /* disable and reset the usart. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_USART_CR1_ADDR) ), endian_le32(0) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_USART_CR2_ADDR) ), endian_le32(0) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_USART_CR3_ADDR) ), endian_le32(0) );

  /* initialize request queues. */
  dev_request_queue_init(&pv->read_q);
  dev_request_queue_init(&pv->write_q);

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

  if (loc[0] == IOMUX_INVALID_DEMUX && loc[1] == IOMUX_INVALID_DEMUX)
    goto err_fifo;

  if (loc[0] != IOMUX_INVALID_DEMUX)
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_RE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
  if (loc[1] != IOMUX_INVALID_DEMUX)
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_TE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

  /* configure over-sampling. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_OVER8_SET( (_reg), 16 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR3_ADDR) ))); STM32_USART_CR3_ONEBIT_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR3_ADDR) ), endian_le32(_reg) ); } while (0);

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

      err = stm32_usart_config_simple(pv, &cfg);
      if (err)
        printk("uart: failed to configure uart with default configuration.\n");
    }

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_init(dev, pv->irq_ep, 1, &stm32_usart_irq);
  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
    goto err_fifo;

  /* enable RX and TX irqs. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_RXNEIE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
#endif

  /* enable usart if configured. */
  if (!err)
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ))); STM32_USART_CR1_UE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_USART_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

  /* link the driver. */
  dev->drv    = &stm32_usart_drv;
  dev->drv_pv = pv;

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

  if (pv->read_started || pv->write_started)
    return -EBUSY;

  /* disable and reset the usart. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_USART_CR1_ADDR) ), endian_le32(0) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_USART_CR2_ADDR) ), endian_le32(0) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_USART_CR3_ADDR) ), endian_le32(0) );

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

  return 0;
}


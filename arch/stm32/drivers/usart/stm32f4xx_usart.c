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

#define STM32F4xx_USART_SR      0x00
#define STM32F4xx_USART_DR      0x04
#define STM32F4xx_USART_BRR     0x08
#define STM32F4xx_USART_CR1     0x0c
#define STM32F4xx_USART_CR2     0x10
#define STM32F4xx_USART_CR3     0x14
#define STM32F4xx_USART_GPTR    0x18

#define STM32F4xx_USART_REG_ADDR(base, offset) \
    ( (base) + (offset) )                      \
/**/

#define STM32F4xx_USART_SR_RXNE             (1 << 5)
#define STM32F4xx_USART_SR_TXE              (1 << 7)

#define STM32F4xx_USART_CR1_RE              (1 << 2)
#define STM32F4xx_USART_CR1_TE              (1 << 3)
#define STM32F4xx_USART_CR1_TXEIE           (1 << 7)
#define STM32F4xx_USART_CR1_RXNEIE          (1 << 5)
#define STM32F4xx_USART_CR1_OVER_16         0
#define STM32F4xx_USART_CR1_OVER_8          (1 << 15)
#define STM32F4xx_USART_CR1_EN              (1 << 13)
#define STM32F4xx_USART_CR1_8_BITS          0
#define STM32F4xx_USART_CR1_9_BITS          (1 << 12)

#define STM32F4xx_USART_CR1_PAR_NONE        0
#define STM32F4xx_USART_CR1_PAR_EN          (1 << 10)
#define STM32F4xx_USART_CR1_PAR_EVEN        0
#define STM32F4xx_USART_CR1_PAR_ODD         (1 << 19)

#define STM32F4xx_USART_CR2_STOP_1_BIT      0
#define STM32F4xx_USART_CR2_STOP_05_BIT     (1 << 12)
#define STM32F4xx_USART_CR2_STOP_2_BIT      (1 << 13)

#define __STM32F4xx_USART_HAS_RDY_RX(base)                    \
  ( (cpu_mem_read_32(                                         \
      STM32F4xx_USART_REG_ADDR((base), STM32F4xx_USART_SR)) & \
    STM32F4xx_USART_SR_RXNE) != 0                             \
  ) \
/**/

#define __STM32F4xx_USART_HAS_RDY_TX(base)                    \
  ( (cpu_mem_read_32(                                         \
      STM32F4xx_USART_REG_ADDR((base), STM32F4xx_USART_SR)) & \
    STM32F4xx_USART_SR_TXE) != 0                              \
  ) \
/**/

#define __STM32F4xx_USART_IS_TX_EN(base)                       \
  ( (cpu_mem_read_32(                                          \
      STM32F4xx_USART_REG_ADDR((base), STM32F4xx_USART_CR1)) & \
    STM32F4xx_USART_CR1_TE) != 0                               \
  ) \
/**/

#define __STM32F4xx_USART_IS_RX_EN(base)                       \
  ( (cpu_mem_read_32(                                          \
      STM32F4xx_USART_REG_ADDR((base), STM32F4xx_USART_CR1)) & \
    STM32F4xx_USART_CR1_RE) != 0                               \
  ) \
/**/

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

    /* read characters if the request asked for more. */
    if (size < rq->size && __STM32F4xx_USART_HAS_RDY_RX(pv->addr))
    {
      rq->data[size++] = endian_le32(
        cpu_mem_read_32(
          STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_DR)));
    }

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

  /* if no request need the data, discard it. */
  if (__STM32F4xx_USART_HAS_RDY_RX(pv->addr))
  {
    (void) cpu_mem_read_32(
      STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_DR));
  }
}

static void stm32f4xx_usart_try_write(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;
  struct dev_char_rq_s              *rq;

  while ((rq = dev_char_queue_head(&pv->write_q)))
  {
    size_t size = 0;

    /* write data if some are pending and the controller is ready for it. */
    if (size < rq->size && __STM32F4xx_USART_HAS_RDY_TX(pv->addr))
    {
      cpu_mem_write_32(
        STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_DR),
        endian_le32(rq->data[size++])
      );
    }

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
     * send.
     */
    break;
#endif
  }

#if defined(CONFIG_DEVICE_IRQ)
  /* if there is no more write request in the queue, disable TX interrupt. */
  if (dev_char_queue_isempty(&pv->write_q))
  {
    uint32_t cr1 = endian_le32(
      cpu_mem_read_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1))
    );
    cpu_mem_write_32(
      STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1),
      (cr1 & ~STM32F4xx_USART_CR1_TXEIE)
    );
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
    {
      uint32_t cr1 = endian_le32(
        cpu_mem_read_32(
          STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1)
        )
      );
      cpu_mem_write_32(
        STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1),
        (cr1 | STM32F4xx_USART_CR1_TXEIE)
      );
#endif
    stm32f4xx_usart_try_write(dev);
#if defined(CONFIG_DEVICE_IRQ)
    }
#endif
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
    uint32_t ir =
    endian_le32(
      cpu_mem_read_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_SR))
    );

    /* if the controller has no pending data and cannot send data, then
     * break and wait for another interrupt.
     */
    if ((ir & ( STM32F4xx_USART_SR_RXNE | STM32F4xx_USART_SR_TXE )) == 0)
    {
      break;
    }

    if ((ir & STM32F4xx_USART_SR_TXE) && !dev_char_queue_isempty(&pv->write_q))
    {
      stm32f4xx_usart_try_write(dev);
    }

    if (ir & STM32F4xx_USART_SR_RXNE)
    {
      stm32f4xx_usart_try_read(dev);
    }
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

#define STM32F4xx_RCC_AHB1ENR_ADDR  0x40023830
#define STM32F4xx_RCC_APB1ENR_ADDR  0x40023840
#define STM32F4xx_RCC_APB2ENR_ADDR  0x40023844

#define STM32F4xx_GPIOA_MODER_ADDR  0x40020000

#define STM32F4xx_GPIOA_AFRL_ADDR   0x40020020
#define STM32F4xx_GPIOA_AFRH_ADDR   0x40020024

static inline void stm32f4xx_usart_clock_init(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;

  assert(pv != 0);

  switch (pv->addr)
  {
  default:
    break;

  case STM32F4xx_USART1_BASE: {
    uint32_t cfg;

    /* enable clock on AHB1 (GPIOA). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_RCC_AHB1ENR_ADDR));
    cfg |= (1 << 0);
    cpu_mem_write_32(STM32F4xx_RCC_AHB1ENR_ADDR, endian_le32(cfg));

    /* enable clock on APB2 (USART1). */
    cfg = cpu_mem_read_32(STM32F4xx_RCC_APB2ENR_ADDR);
    cfg |= (1 << 4);
    cpu_mem_write_32(STM32F4xx_RCC_APB2ENR_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART2_BASE: {
    uint32_t cfg;

    /* enable clock on AHB1 (GPIOA). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_RCC_AHB1ENR_ADDR));
    cfg |= (1 << 0);
    cpu_mem_write_32(STM32F4xx_RCC_AHB1ENR_ADDR, endian_le32(cfg));

    /* enable clock on APB1 (USART2). */
    cfg = cpu_mem_read_32(STM32F4xx_RCC_APB1ENR_ADDR);
    cfg |= (1 << 17);
    cpu_mem_write_32(STM32F4xx_RCC_APB1ENR_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART6_BASE: {
    uint32_t cfg;

    /* enable clock on AHB1 (GPIOA). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_RCC_AHB1ENR_ADDR));
    cfg |= (1 << 0);
    cpu_mem_write_32(STM32F4xx_RCC_AHB1ENR_ADDR, endian_le32(cfg));

    /* enable clock on APB2 (USART6). */
    cfg = cpu_mem_read_32(STM32F4xx_RCC_APB2ENR_ADDR);
    cfg |= (1 << 5);
    cpu_mem_write_32(STM32F4xx_RCC_APB2ENR_ADDR, endian_le32(cfg));

    break;
  }

  }
}

static inline void stm32f4xx_usart_clock_cleanup(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;

  assert(pv != 0);

  switch (pv->addr)
  {
  default:
    break;

  case STM32F4xx_USART1_BASE: {
    uint32_t cfg;

    /* enable clock on APB2 (USART1). */
    cfg = cpu_mem_read_32(STM32F4xx_RCC_APB2ENR_ADDR);
    cfg &= ~(1 << 4);
    cpu_mem_write_32(STM32F4xx_RCC_APB2ENR_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART2_BASE: {
    uint32_t cfg;

    /* enable clock on APB1 (USART2). */
    cfg = cpu_mem_read_32(STM32F4xx_RCC_APB1ENR_ADDR);
    cfg &= ~(1 << 17);
    cpu_mem_write_32(STM32F4xx_RCC_APB1ENR_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART6_BASE: {
    uint32_t cfg;

    /* enable clock on APB2 (USART6). */
    cfg = cpu_mem_read_32(STM32F4xx_RCC_APB2ENR_ADDR);
    cfg &= ~(1 << 5);
    cpu_mem_write_32(STM32F4xx_RCC_APB2ENR_ADDR, endian_le32(cfg));

    break;
  }

  }
}

static inline void stm32f4xx_usart_gpio_init(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;

  assert(pv != 0);

  switch (pv->addr)
  {
  default:
    break;

  case STM32F4xx_USART1_BASE: {
    uint32_t cfg;

    /* configure PA2/PA3 as TX/RX (MODER). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_MODER_ADDR));
    cfg |= (2 << 18) | (2 << 20);
    cpu_mem_write_32(STM32F4xx_GPIOA_MODER_ADDR, endian_le32(cfg));
    
    /* configure PA2/PA3 function to USART. */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_AFRH_ADDR));
    cfg |= (7 << 4) | (7 << 8);
    cpu_mem_write_32(STM32F4xx_GPIOA_AFRH_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART2_BASE: {
    uint32_t cfg;

    /* configure PA2/PA3 as TX/RX (MODER). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_MODER_ADDR));
    cfg |= (2 << 4) | (2 << 6);
    cpu_mem_write_32(STM32F4xx_GPIOA_MODER_ADDR, endian_le32(cfg));
    
    /* configure PA2/PA3 function to USART. */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_AFRL_ADDR));
    cfg |= (7 << 8) | (7 << 12);
    cpu_mem_write_32(STM32F4xx_GPIOA_AFRL_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART6_BASE: {
    uint32_t cfg;

    /* configure PA2/PA3 as TX/RX (MODER). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_MODER_ADDR));
    cfg |= (2 << 22) | (2 << 24);
    cpu_mem_write_32(STM32F4xx_GPIOA_MODER_ADDR, endian_le32(cfg));
    
    /* configure PA2/PA3 function to USART. */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_AFRH_ADDR));
    cfg |= (8 << 12) | (8 << 16);
    cpu_mem_write_32(STM32F4xx_GPIOA_AFRH_ADDR, endian_le32(cfg));

    break;
  }

  }
}

static inline void stm32f4xx_usart_gpio_cleanup(struct device_s *dev)
{
  struct stm32f4xx_usart_context_s  *pv = dev->drv_pv;

  assert(pv != 0);

  switch (pv->addr)
  {
  default:
    break;

  case STM32F4xx_USART1_BASE: {
    uint32_t cfg;

    /* configure PA2/PA3 as TX/RX (MODER). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_MODER_ADDR));
    cfg &= ~((2 << 18) | (2 << 20));
    cpu_mem_write_32(STM32F4xx_GPIOA_MODER_ADDR, endian_le32(cfg));
    
    /* configure PA2/PA3 function to USART. */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_AFRH_ADDR));
    cfg &= ~((7 << 4) | (7 << 8));
    cpu_mem_write_32(STM32F4xx_GPIOA_AFRH_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART2_BASE: {
    uint32_t cfg;

    /* configure PA2/PA3 as TX/RX (MODER). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_MODER_ADDR));
    cfg &= ~((2 << 4) | (2 << 6));
    cpu_mem_write_32(STM32F4xx_GPIOA_MODER_ADDR, endian_le32(cfg));
    
    /* configure PA2/PA3 function to USART. */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_AFRL_ADDR));
    cfg &= ~((7 << 8) | (7 << 12));
    cpu_mem_write_32(STM32F4xx_GPIOA_AFRL_ADDR, endian_le32(cfg));

    break;
  }

  case STM32F4xx_USART6_BASE: {
    uint32_t cfg;

    /* configure PA2/PA3 as TX/RX (MODER). */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_MODER_ADDR));
    cfg &= ~((2 << 22) | (2 << 24));
    cpu_mem_write_32(STM32F4xx_GPIOA_MODER_ADDR, endian_le32(cfg));
    
    /* configure PA2/PA3 function to USART. */
    cfg = endian_le32(cpu_mem_read_32(STM32F4xx_GPIOA_AFRH_ADDR));
    cfg &= ~((8 << 12) | (8 << 16));
    cpu_mem_write_32(STM32F4xx_GPIOA_AFRH_ADDR, endian_le32(cfg));

    break;
  }

  }
}

/* ************************************************************************* */

static DEV_INIT(stm32f4xx_usart_init)
{
  struct stm32f4xx_usart_context_s  *pv;
  uint32_t                          cr1 = 0, cr2 = 0, brr = 0;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
  {
    return -ENOMEM;
  }

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
  {
    goto err_mem;
  }

  /* wait for previous TX to complete. */
  if (__STM32F4xx_USART_IS_TX_EN(pv->addr))
  {
    while (!__STM32F4xx_USART_HAS_RDY_TX(pv->addr));
  }

  /* disable and reset the usart. */
  cpu_mem_write_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1), 0);
  cpu_mem_write_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR2), 0);
  cpu_mem_write_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR3), 0);

  /* initialize request queues. */
  dev_char_queue_init(&pv->read_q);
  dev_char_queue_init(&pv->write_q);

  /* configure clocks. */
  stm32f4xx_usart_clock_init(dev);

  /* configure gpio. */
  stm32f4xx_usart_gpio_init(dev);

  /* configure usart . */
  cr1 |= STM32F4xx_USART_CR1_8_BITS;        //> 8 bits.
  cr1 |= STM32F4xx_USART_CR1_PAR_NONE;      //> no parity.
  cr2 |= STM32F4xx_USART_CR2_STOP_1_BIT;    //> 1 stop bit. 

  cr1 |= STM32F4xx_USART_CR1_TE | STM32F4xx_USART_CR1_RE;

  /* configure baudrate. */
  cr1 |= STM32F4xx_USART_CR1_OVER_16;       //> oversampling.
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

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_init(
    dev,
    pv->irq_ep,
    1,
    &stm32f4xx_usart_irq,
    DEV_IRQ_SENSE_HIGH_LEVEL
  );

  if (device_irq_source_link(dev, pv->irq_ep, 1, -1))
  {
    goto err_irq;
  }

  /* enable RX irq. TX irq is activated on on demand. */
  cr1 |= STM32F4xx_USART_CR1_RXNEIE;
#endif

  /* write configuration. */
  cpu_mem_write_32(
    STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1),
    cr1
  );
  cpu_mem_write_32(
    STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR2),
    cr2
  );
  cpu_mem_write_32(
    STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_BRR),
    brr
  );

  /* enable usart. */
  cr1 |= STM32F4xx_USART_CR1_EN;
  cpu_mem_write_32(
    STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1),
    cr1
  );

  /* link the driver. */
  dev->drv = &stm32f4xx_usart_drv;

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#if defined(CONFIG_DEVICE_IRQ)
err_irq:
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

  /* disable the uart. */
  cpu_mem_write_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR1), 0);
  cpu_mem_write_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR2), 0);
  cpu_mem_write_32(STM32F4xx_USART_REG_ADDR(pv->addr, STM32F4xx_USART_CR3), 0);

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_unlink(dev, pv->irq_ep, 1);
#endif

  /* destroy request queues. */
  dev_char_queue_destroy(&pv->read_q);
  dev_char_queue_destroy(&pv->write_q);

  mem_free(pv);
}


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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014

*/

/**
    This driver can be used to generate most of I2C transfers. 
    Transfers with a START and a STOP condition are always processed.
    RESTART condition between a write and a read transfer can only be
    generated if a timer ressource is available in device tree. Otherwise
    -ENOTSUP is returned.

    The BCM283X I2C controller does not support clock stretching in the
    middle of a byte exchange. Clock stretching is supported only after 
    a acknoledge bit and before the fisrt bit of a new data byte.
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/i2c.h>
#include <device/class/timer.h>
#include <device/class/iomux.h>

#include <arch/bcm283x/i2c.h>

#define BCM283X_I2C_CORE_CLK          150000000
#define BCM283X_I2C_IRQ_MASK          0x30E
#define BCM283X_I2C_IRQ_CLR_ON_WRITE  0x302

enum bcm283x_i2c_state_e
{
  DEV_I2C_BCM283X_IDLE,
  DEV_I2C_BCM283X_START,
  DEV_I2C_BCM283X_WRITE_DATA,
  DEV_I2C_BCM283X_WAIT_END_WRITE,
  DEV_I2C_BCM283X_READ_DATA,
  DEV_I2C_BCM283X_END,
};

struct bcm283x_i2c_context_s
{
  uintptr_t addr;
  /* Interrupt end-point */
  struct dev_irq_src_s irq_ep;
  /* request queue */
  dev_request_queue_root_t queue;
  /* global transfert size */
  uint16_t glen;
  /* local transfert size */
  uint16_t len;
  /* transfert count */
  uint8_t cnt;
  /* Restart */
  bool_t restart;
  /* Fsm state */
  enum bcm283x_i2c_state_e state;
  /* Timer accessor */
  struct device_timer_s timer;
  /* Timeout for tx fifo in timer cycles*/
  uint32_t timeout;
  /* timer request */
  struct dev_timer_rq_s trq;
  uint32_t bperiod;
};


static inline uint32_t bcm283x_i2c_get_rxd(struct bcm283x_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR);
   return (endian_le32(x) & BCM283X_I2C_S_RXD);
}

static inline uint32_t bcm283x_i2c_get_txd(struct bcm283x_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR);
   return (endian_le32(x) & BCM283X_I2C_S_TXD);
}

static inline uint32_t bcm283x_i2c_get_txe(struct bcm283x_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR);
   return (endian_le32(x) & BCM283X_I2C_S_TXE);
}

static inline void bcm283x_i2c_reset(struct bcm283x_i2c_context_s *pv)
{
   /* Disable controller */
   cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(0));  

   uint32_t x = BCM283X_I2C_C_I2CEN |
                BCM283X_I2C_C_INTR  |
                BCM283X_I2C_C_INTD  |
                BCM283X_I2C_C_CLEAR;
   
   cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));  
}

static inline uint32_t bcm283x_i2c_get_dlen(struct bcm283x_i2c_context_s *pv)
{
  return endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_DLEN_ADDR));
}

static inline void bcm283x_i2c_txw_irq(struct bcm283x_i2c_context_s *pv, bool_t enable)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM283X_I2C_C_ADDR);
   if (enable)
     x |= BCM283X_I2C_C_INTT;
   else  
     x &= ~BCM283X_I2C_C_INTT;
   cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));  
}

static void bcm283x_i2c_set_transfer_parameter(struct bcm283x_i2c_context_s *pv)
{
  struct dev_request_s *base = dev_request_queue_head(&pv->queue);
  struct dev_i2c_rq_s *rq = dev_i2c_rq_s_cast(base);
  struct dev_i2c_transfer_s *ctr = rq->transfer + pv->cnt;

  enum dev_i2c_way_e type = ctr->type;

  pv->len = ctr->size;
  pv->glen = ctr->size;
  pv->restart = 0;

  uint8_t cnt = pv->cnt;

  while (cnt < rq->transfer_count - 1)
    {
      cnt++;

      struct dev_i2c_transfer_s *tr = rq->transfer + cnt;

      if (tr->type != type)
        {
          pv->restart = 1;
          break;
        }
      pv->glen += tr->size;
    }

}

static bool_t bcm283x_i2c_valid_request(struct bcm283x_i2c_context_s *pv,
                                        struct dev_i2c_rq_s *rq)
{
  for (uint8_t i = 0; i < rq->transfer_count - 1; i++)
    {
      struct dev_i2c_transfer_s *cur  = rq->transfer + i;
      struct dev_i2c_transfer_s *next = rq->transfer + i + 1;

      if ((cur->type == DEV_I2C_READ) && (next->type == DEV_I2C_WRITE))
        return 0;

      if ((cur->type == DEV_I2C_READ) && (next->type == DEV_I2C_WRITE))
        {
          /* Restart is only possible with a timer * */
          if (!device_check_accessor(&pv->timer))
              return 0;
        }
    }

  return 1; 
}

static bool_t bcm283x_i2c_timer_rq(struct device_s *dev);

static void bcm283x_i2c_fsm(struct device_s *dev, bool_t stop)
{
  struct bcm283x_i2c_context_s   *pv  = dev->drv_pv;
  uint32_t x;

  while(1)
    {

      struct dev_request_s *base = dev_request_queue_head(&pv->queue);
      struct dev_i2c_rq_s *rq = dev_i2c_rq_s_cast(base);
      struct dev_i2c_transfer_s *ctr = rq->transfer + pv->cnt;

      if (rq == NULL)
        return;

    //  printk("state :%d\n", pv->state);

      switch (pv->state)
        {
          case DEV_I2C_BCM283X_IDLE: /* 0 */
            rq->error = 0;
            rq->error_transfer = 0;
            rq->error_offset = 0;
            pv->cnt = 0;
            pv->state = DEV_I2C_BCM283X_START;
            break;

          case DEV_I2C_BCM283X_START: /* 1 */
           
            /* Get length and restart parameters */
            bcm283x_i2c_set_transfer_parameter(pv);

            pv->state = DEV_I2C_BCM283X_END;

            x = pv->glen;

            if (pv->glen)
              {  
                if (ctr->type == DEV_I2C_WRITE)
                  {
                    pv->state = DEV_I2C_BCM283X_WRITE_DATA;
                    if (pv->restart)
                      x += 1;
                  }
                else
                  pv->state = DEV_I2C_BCM283X_READ_DATA;
              }

            /* Set DLEN */
            cpu_mem_write_32(pv->addr + BCM283X_I2C_DLEN_ADDR, endian_le32(x));

            /* Address on 7 bits */
            x = rq->saddr & 0x7F;
            /* Set address byte */
            cpu_mem_write_32(pv->addr + BCM283X_I2C_A_ADDR, endian_le32(x));
            
            x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_C_ADDR));
            x |= BCM283X_I2C_C_ST | BCM283X_I2C_C_CLEAR;

            if (ctr->type == DEV_I2C_WRITE || !pv->glen)
              x &= ~BCM283X_I2C_C_READ;
            else
              x |= BCM283X_I2C_C_READ;

            /* Start transfer */
            cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));

            break;
          
          case DEV_I2C_BCM283X_WRITE_DATA: /* 2 */
            if (!pv->glen)
              {
                pv->cnt++;
                pv->state = DEV_I2C_BCM283X_END;
                /* disable TXW irq */
                bcm283x_i2c_txw_irq(pv, 0);

                if (pv->restart)
                  pv->state = DEV_I2C_BCM283X_WAIT_END_WRITE;

                break;
              }

            if (!pv->len)
              {
                pv->cnt++;
                ctr += 1;
                pv->len = ctr->size;
              }

              if (!bcm283x_i2c_get_txd(pv))
              {
                /* Enable TXW irq */
                bcm283x_i2c_txw_irq(pv, 1);
                return;
              }
              
              x = ctr->data[ctr->size - pv->len];
              cpu_mem_write_32(pv->addr + BCM283X_I2C_FIFO_ADDR, endian_le32(x)); 

              pv->glen--;
              pv->len--;

             break;

        case DEV_I2C_BCM283X_WAIT_END_WRITE: /* 3 */
          while (1)
            {
              if (bcm283x_i2c_get_txe(pv))
                /* Fifo empty */
                {
                  pv->state = DEV_I2C_BCM283X_START;
                  break;
                }
              if (!bcm283x_i2c_timer_rq(dev))
                return;
            }
          break;

          case DEV_I2C_BCM283X_READ_DATA: /* 4 */
            if (!pv->glen)
              {
                pv->state = DEV_I2C_BCM283X_END;
                break;
              }

            if (!pv->len)
              {
                pv->cnt++;
                ctr += 1;
                pv->len = ctr->size;
              }

            if (!bcm283x_i2c_get_rxd(pv))
              return;

            x = cpu_mem_read_32(pv->addr + BCM283X_I2C_FIFO_ADDR);
            ctr->data[ctr->size - pv->len] = endian_le32(x);

            pv->glen--;
            pv->len--;

            break;

          case DEV_I2C_BCM283X_END: /* 5 */
            if (!stop)
              return;
            /* Reset controller */
            bcm283x_i2c_reset(pv);
            /* remove request from timer queue */
            DEVICE_OP(&pv->timer, cancel, &pv->trq);
            pv->state = DEV_I2C_BCM283X_IDLE;
            return;
        } 
    }
}

static KROUTINE_EXEC(bcm283x_i2c_timeout)
{
   struct dev_timer_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, rq.kr);
   struct device_s *dev = rq->rq.pvdata;
   struct bcm283x_i2c_context_s *pv  = dev->drv_pv;
 
   LOCK_SPIN_IRQ(&dev->lock);

   if (pv->state == DEV_I2C_BCM283X_WAIT_END_WRITE)
     {
       /* get status */
       uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR));

       if (!(x & (BCM283X_I2C_S_ERR | BCM283X_I2C_S_CLKT)))
       /* No pending interrupt */
         bcm283x_i2c_fsm(dev, 0);
     }

   LOCK_RELEASE_IRQ(&dev->lock);
}

static bool_t bcm283x_i2c_timer_rq(struct device_s *dev)
{
   struct bcm283x_i2c_context_s    *pv  = dev->drv_pv;
   uint32_t len = bcm283x_i2c_get_dlen(pv);

   assert(pv->trq.rq.drvdata == NULL);
   while (1)
     {
       pv->trq.delay = (len + 1) * pv->timeout;
       switch (DEVICE_OP(&pv->timer, request, &pv->trq))
         {
         case -EAGAIN:
           if (!dev_timer_init_sec(&pv->timer, &pv->timeout, &pv->trq.rev,
                                   pv->bperiod, 1000000))
             continue;
         default:
           printk("BCM283X I2C driver: Timer error\n");
           return 1;
         case -ETIMEDOUT:
           return 1;
         case 0:
           return 0;
         }
     }
}

/***************************************** interrupt */

static DEV_IRQ_SRC_PROCESS(bcm283x_i2c_irq)
{
  struct device_s                *dev = ep->base.dev;
  struct bcm283x_i2c_context_s   *pv  = dev->drv_pv;

  bool_t stop = 0;
   
  lock_spin(&dev->lock);

  struct dev_request_s *base = dev_request_queue_head(&pv->queue);
  struct dev_i2c_rq_s *rq = dev_i2c_rq_s_cast(base);

  while (1)
    {
      /* get status */
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR));

      /* Reset interrupts flags */
      cpu_mem_write_32(pv->addr + BCM283X_I2C_S_ADDR, endian_le32(x & BCM283X_I2C_IRQ_CLR_ON_WRITE));

      if (!(x & BCM283X_I2C_IRQ_MASK) || rq == NULL)
        break;

      if (x & (BCM283X_I2C_S_DONE | BCM283X_I2C_S_ERR | BCM283X_I2C_S_CLKT))
        stop = 1;

      if (x & (BCM283X_I2C_S_ERR | BCM283X_I2C_S_CLKT))
        {
           rq->error = -EHOSTUNREACH;
           rq->error_transfer = pv->cnt;
           rq->error_offset = 0;
           pv->state = DEV_I2C_BCM283X_END;
        }

      bcm283x_i2c_fsm(dev, stop);

      if (stop)
        {
          dev_request_queue_pop(&pv->queue);
          break;
        }
    }

  if (stop)
    {
      kroutine_exec(&base->kr);

      base = dev_request_queue_head(&pv->queue);
      rq = dev_i2c_rq_s_cast(base);

      if (rq != NULL && (pv->state == DEV_I2C_BCM283X_IDLE))
        bcm283x_i2c_fsm(dev, 0);

    }

  lock_release(&dev->lock);
}

/***************************************** config */

static DEV_I2C_CONFIG(bcm283x_i2c_config)
{
  struct device_s               *dev = accessor->dev;
  struct bcm283x_i2c_context_s  *pv  = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t div = (BCM283X_I2C_CORE_CLK/config->bit_rate);
  cpu_mem_write_32(pv->addr + BCM283X_I2C_DIV_ADDR, endian_le32(div));
  /* Byte period in us */
  pv->bperiod = 8000000 / config->bit_rate;
  pv->trq.rev = 2;

  LOCK_RELEASE_IRQ(&dev->lock);

  return 0;
}

static DEV_I2C_REQUEST(bcm283x_i2c_request)
{
  struct device_s                 *dev = accessor->dev;
  struct bcm283x_i2c_context_s    *pv  = dev->drv_pv;

  assert(req->transfer_count);

  if (!bcm283x_i2c_valid_request(pv, req))
    {
      req->error = -ENOTSUP;
      kroutine_exec(&req->base.kr);
      return;
    }

  LOCK_SPIN_IRQ(&dev->lock);

  bool_t start = dev_request_queue_isempty(&pv->queue);
 
  dev_request_queue_pushback(&pv->queue, &req->base);

  if (start)
    bcm283x_i2c_fsm(dev, 0);

  LOCK_RELEASE_IRQ(&dev->lock);

}

static DEV_INIT(bcm283x_i2c_init);
static DEV_CLEANUP(bcm283x_i2c_cleanup);

#define bcm283x_i2c_use dev_use_generic

DRIVER_DECLARE(bcm283x_i2c_drv, 0, "BCM283X I2C Master", bcm283x_i2c,
               DRIVER_I2C_METHODS(bcm283x_i2c));

DRIVER_REGISTER(bcm283x_i2c_drv);

static DEV_INIT(bcm283x_i2c_init)
{
  struct bcm283x_i2c_context_s    *pv;


  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  /* retrieve device base address from device tree. */
  if(device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  dev_request_queue_init(&pv->queue);

  /* Set timeout to maximum value */
  cpu_mem_write_32(pv->addr + BCM283X_I2C_CLKT_ADDR, endian_le32(0xFFFF));

  /* Clear interrupts */
  cpu_mem_write_32(pv->addr + BCM283X_I2C_S_ADDR, endian_le32(BCM283X_I2C_IRQ_CLR_ON_WRITE));

  bcm283x_i2c_reset(pv);

  /* setup pinmux */
  if (device_iomux_setup(dev, ",scl ,sda", NULL, NULL, NULL))
    goto err_mem;

  /* Get accessor on timer */
  if (device_get_param_dev_accessor(dev, "timer", &pv->timer, DRIVER_CLASS_TIMER))
    device_init_accessor(&pv->timer);

  pv->trq.rq.pvdata = dev;
  pv->trq.rq.drvdata = NULL;
  pv->trq.deadline = 0;
  pv->trq.rev = 2;
  kroutine_init_immediate(&pv->trq.rq.kr, bcm283x_i2c_timeout);
  /* Set default timeout in counter cycle for 1 byte @ 100 KHz */
  pv->bperiod = 80;

  device_irq_source_init(dev, &pv->irq_ep, 1, &bcm283x_i2c_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_mem;


  return 0;

err_mem:
  mem_free(pv);
  return -EINVAL;
}

static DEV_CLEANUP(bcm283x_i2c_cleanup)
{
  struct bcm283x_i2c_context_s    *pv;

  pv = dev->drv_pv;

  /* disable I2C device. */
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, 0);  

  /* disable interrupt. */
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* Destroy request queue */
  dev_request_queue_destroy(&pv->queue);

  /* deallocate private driver context. */
  mem_free(pv);
}


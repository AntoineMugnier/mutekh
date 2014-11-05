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
    This driver can be used to generate most of I2C transfers. Transfers 
    with a START and a STOP condition are always processed. Transfers
    without START condition are not managed and return a -ENOTSUP error
    code. Transfers without STOP condition are only processed if a timer
    ressource is available in device tree. Otherwise -ENOTSUP is returned.
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
#include <device/class/iomux.h>
#include <device/class/timer.h>

#include <arch/bcm2835_i2c.h>

#define BCM2835_I2C_CORE_CLK         150000000
#define BCM2835_I2C_IRQ_MASK         0x30E
#define BCM2835_I2C_IRQ_CLR_ON_WRITE 0x302

enum bcm2835_i2c_state_e
{
  DEV_I2C_BCM2835_IDLE,
  DEV_I2C_BCM2835_DATA,
  DEV_I2C_BCM2835_ERROR,
  DEV_I2C_BCM2835_END,
};

struct bcm2835_i2c_context_s
{
  uintptr_t addr;
  /* current transfer */
  struct dev_i2c_ctrl_transfer_s *tr;
  /* Interrupt end-point */
  struct dev_irq_ep_s irq_ep;
  /* Initial buffer size */
  size_t count;
  /* Fsm state */
  enum bcm2835_i2c_state_e state;
  /* Timer accessor */
  struct device_timer_s timer;
  /* Timeout for tx fifo in timer cycles*/
  uint32_t timeout;
  /* timer request */
  struct dev_timer_rq_s rq;
  bool_t data;
};

static void bcm2835_i2c_timer_rq(struct device_s *dev);

static inline uint32_t bcm2835_i2c_get_rxd(struct bcm2835_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM2835_I2C_S_ADDR);
   return (endian_le32(x) & BCM2835_I2C_S_RXD);
}

static inline uint32_t bcm2835_i2c_get_txd(struct bcm2835_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM2835_I2C_S_ADDR);
   return (endian_le32(x) & BCM2835_I2C_S_TXD);
}

static inline uint32_t bcm2835_i2c_get_txe(struct bcm2835_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM2835_I2C_S_ADDR);
   return (endian_le32(x) & BCM2835_I2C_S_TXE);
}

static inline void bcm2835_i2c_reset(struct bcm2835_i2c_context_s *pv)
{
   uint32_t x = BCM2835_I2C_C_I2CEN |
                BCM2835_I2C_C_INTR  |
                BCM2835_I2C_C_INTD  |
                BCM2835_I2C_C_INTT  |
                BCM2835_I2C_C_CLEAR;
   
   cpu_mem_write_32(pv->addr + BCM2835_I2C_C_ADDR, endian_le32(x));  
}

static inline void bcm2835_i2c_fifo_irq(struct bcm2835_i2c_context_s *pv, bool_t enable)
{
   uint32_t x = cpu_mem_read_32(pv->addr + BCM2835_I2C_C_ADDR);
   if (enable)
     x |= BCM2835_I2C_C_INTR | BCM2835_I2C_C_INTT;
   else  
     x &= ~(BCM2835_I2C_C_INTR | BCM2835_I2C_C_INTT);
   cpu_mem_write_32(pv->addr + BCM2835_I2C_C_ADDR, endian_le32(x));  
}

static bool_t bcm2835_i2c_fsm(struct device_s *dev, bool_t stop)
{
  struct bcm2835_i2c_context_s   *pv  = dev->drv_pv;
  uint32_t x;

  while(1)
    {
      switch (pv->state)
        {
          case DEV_I2C_BCM2835_IDLE: /* 0 */

            /* Set DLEN */
            x = pv->tr->count;

            if (!(pv->tr->op & DEV_I2C_OP_STOP))
              x = pv->tr->count + 1;
            cpu_mem_write_32(pv->addr + BCM2835_I2C_DLEN_ADDR, endian_le32(x));

            /* Set address byte */
            x = pv->tr->saddr & 0x7F;
            cpu_mem_write_32(pv->addr + BCM2835_I2C_A_ADDR, endian_le32(x));

            /* Start transfer */
            x = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_I2C_C_ADDR));
            x |= BCM2835_I2C_C_ST | BCM2835_I2C_C_CLEAR;
            if (pv->tr->dir == DEV_I2C_TR_READ) 
              x |= BCM2835_I2C_C_READ;
            else
              x &= ~BCM2835_I2C_C_READ;
            cpu_mem_write_32(pv->addr + BCM2835_I2C_C_ADDR, endian_le32(x));

            pv->state = DEV_I2C_BCM2835_DATA;
            if (!pv->data)
              {
                pv->state = DEV_I2C_BCM2835_END;
                /* Start timer if no stop */
                if (!(pv->tr->op & DEV_I2C_OP_STOP))
                  bcm2835_i2c_timer_rq(dev);
              }
            else
              /* Enable fifo irq */
              bcm2835_i2c_fifo_irq(pv, 1);
            break;
          
          case DEV_I2C_BCM2835_DATA: /* 1 */
            if (!pv->tr->count)
              {
                /* disable fifo irq */
                bcm2835_i2c_fifo_irq(pv, 0);
                pv->state = DEV_I2C_BCM2835_END;
                /* Start timer if no stop */
                if (!(pv->tr->op & DEV_I2C_OP_STOP))
                  bcm2835_i2c_timer_rq(dev);
                break;
              }
            if (pv->tr->dir == DEV_I2C_TR_READ)
              {
                if (!bcm2835_i2c_get_rxd(pv))
                  return 0;

                 x = cpu_mem_read_32(pv->addr + BCM2835_I2C_FIFO_ADDR);
                 pv->tr->data[pv->count - pv->tr->count] = endian_le32(x);
                 pv->tr->count--;
              }
            else
              {
                if (!bcm2835_i2c_get_txd(pv))
                  return 0;

                x = pv->tr->data[pv->count - pv->tr->count];
                cpu_mem_write_32(pv->addr + BCM2835_I2C_FIFO_ADDR, endian_le32(x)); 
                pv->tr->count--;
              }
             break;

          case DEV_I2C_BCM2835_ERROR: /* 2 */
             bcm2835_i2c_reset(pv);
             if (!(pv->tr->op & DEV_I2C_OP_STOP))
             /* remove request from timer queue */
               DEVICE_OP(&pv->timer, cancel, &pv->rq);
             pv->tr = NULL;
             return 1;
          
          case DEV_I2C_BCM2835_END: /* 3 */
            if (!stop)
              return 0;
            
            if (!(pv->tr->op & DEV_I2C_OP_STOP))
            /* remove request from timer queue */
              DEVICE_OP(&pv->timer, cancel, &pv->rq);

            pv->tr = NULL;
            return 1;
        } 
    }
}

static KROUTINE_EXEC(bcm2835_i2c_timeout)
{
   struct dev_timer_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, kr);
   struct device_s *dev = rq->pvdata;
   struct bcm2835_i2c_context_s *pv  = dev->drv_pv;
   struct dev_i2c_ctrl_transfer_s *tr = pv->tr;
 
   bool_t done = 0;

   LOCK_SPIN_IRQ(&dev->lock);

   if (pv->tr != NULL)
     {
       /* get status */
       uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_I2C_S_ADDR));
       /* Reset interrupts flags */
       cpu_mem_write_32(pv->addr + BCM2835_I2C_S_ADDR, endian_le32(x & BCM2835_I2C_IRQ_CLR_ON_WRITE));
 
       bool_t stop = bcm2835_i2c_get_txe(pv) >> BCM2835_I2C_S_TXE_SHIFT; 
       
       if (x & BCM2835_I2C_S_ERR)
         {
            tr->error = -EAGAIN;
            pv->state = DEV_I2C_BCM2835_ERROR;
         }
       if ((x & BCM2835_I2C_S_CLKT) || !stop)
         {
            tr->error = -ETIMEDOUT;
            pv->state = DEV_I2C_BCM2835_ERROR;
         }
       done = bcm2835_i2c_fsm(dev, stop);
     }

   LOCK_RELEASE_IRQ(&dev->lock);
 
   if (done)
     kroutine_exec(&tr->kr, 0); 
}

static void bcm2835_i2c_timer_rq(struct device_s *dev)
{
   struct bcm2835_i2c_context_s    *pv  = dev->drv_pv;

   kroutine_init(&pv->rq.kr, bcm2835_i2c_timeout, KROUTINE_IMMEDIATE);

   pv->rq.delay = pv->timeout;
   pv->rq.pvdata = dev;

   DEVICE_OP(&pv->timer, request, &pv->rq);
}


/***************************************** interrupt */

static DEV_IRQ_EP_PROCESS(bcm2835_i2c_irq)
{
  struct device_s                *dev = ep->dev;
  struct bcm2835_i2c_context_s   *pv  = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  bool_t done = 0;
   
  lock_spin(&dev->lock);

  while (1)
    {
      /* get status */
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + BCM2835_I2C_S_ADDR));

      /* Reset interrupts flags */
      cpu_mem_write_32(pv->addr + BCM2835_I2C_S_ADDR, endian_le32(x & BCM2835_I2C_IRQ_CLR_ON_WRITE));

      if (!(x & BCM2835_I2C_IRQ_MASK) || (pv->tr == NULL))
        break;
    
      bool_t stop = (x & BCM2835_I2C_S_DONE) >> BCM2835_I2C_S_DONE_SHIFT;
      
      if (x & BCM2835_I2C_S_ERR)
        {
           tr->error = -EAGAIN;
           pv->state = DEV_I2C_BCM2835_ERROR;
        }
      if (x & BCM2835_I2C_S_CLKT)
        {
           tr->error = -ETIMEDOUT;
           pv->state = DEV_I2C_BCM2835_ERROR;
        }

      done = bcm2835_i2c_fsm(dev, stop);

      if (done)
        break;
    }

  lock_release(&dev->lock);
   
  if (done)
    kroutine_exec(&tr->kr, 0); 
}

/***************************************** config */

DEV_I2C_CTRL_CONFIG(bcm2835_i2c_config)
{
  struct device_s               *dev = i2cdev->dev;
  struct bcm2835_i2c_context_s    *pv  = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    err = -EBUSY;
  else
    {
       uint32_t div = 0;
       /* Fifo size is 16 * 8 bits */
       switch (cfg->bit_rate)
         {
           case DEV_I2C_SPEED_STD:
             div = (BCM2835_I2C_CORE_CLK/100000);
             dev_timer_init_sec(&pv->timer, &pv->timeout, 1920, 1000000);
             break;
           case DEV_I2C_SPEED_FAST:
             div = (BCM2835_I2C_CORE_CLK/400000);
             dev_timer_init_sec(&pv->timer, &pv->timeout, 480, 1000000);
             break;
           case DEV_I2C_SPEED_HIGH:
             div = (BCM2835_I2C_CORE_CLK/1000000);
             dev_timer_init_sec(&pv->timer, &pv->timeout, 192, 1000000);
             break;
         }
       cpu_mem_write_32(pv->addr + BCM2835_I2C_DIV_ADDR, endian_le32(div));
     }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

DEV_I2C_CTRL_TRANSFER(bcm2835_i2c_transfer)
{
//      printk("tr start : %d stop : %d\n", tr->op & 0x1, (tr->op & 0x2) >> 1);
  struct device_s                 *dev = i2cdev->dev;
  struct bcm2835_i2c_context_s    *pv  = dev->drv_pv;
  bool_t done = 1;

  assert(pv->tr == NULL);

  LOCK_SPIN_IRQ(&dev->lock);

  if (!(tr->op & DEV_I2C_OP_STOP) &&
     (!device_check_accessor(&pv->timer) || (tr->dir == DEV_I2C_TR_READ)))
    tr->error = -ENOTSUP;
  else if (!(tr->op & DEV_I2C_OP_START))
    tr->error = -ENOTSUP;
  else
    {
      tr->error = 0;
      pv->tr    = tr;
      pv->count = tr->count;
      pv->state = DEV_I2C_BCM2835_IDLE;
      pv->data  = ((tr->data != NULL) && tr->count);

      done = bcm2835_i2c_fsm(dev, 0);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr, 0); 

}

static const struct driver_i2c_ctrl_s bcm2835_i2c_drv_cls =
{
  .class_       = DRIVER_CLASS_I2C_CTRL,
  .f_config     = &bcm2835_i2c_config,
  .f_transfer   = &bcm2835_i2c_transfer,
};

static DEV_INIT(bcm2835_i2c_init);
static DEV_CLEANUP(bcm2835_i2c_cleanup);

const struct driver_s bcm2835_i2c_drv =
{
  .desc         = "BCM2835 Master Inter-Integrated Circuit Interface (I2C)",
  .f_init       = &bcm2835_i2c_init,
  .f_cleanup    = &bcm2835_i2c_cleanup,
  .classes      =
  {
    &bcm2835_i2c_drv_cls,
    0
  }
};

REGISTER_DRIVER(bcm2835_i2c_drv);

static DEV_INIT(bcm2835_i2c_init)
{
  struct bcm2835_i2c_context_s    *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  /* retreive the device base address from device tree. */
  if(device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  /* reset current transfer. */
  pv->tr = NULL;
  /* Set default timeout */
  pv->timeout = 1920;

  bcm2835_i2c_reset(pv);

  /* Set timeout to maximum value */
  cpu_mem_write_32(pv->addr + BCM2835_I2C_CLKT_ADDR, endian_le32(0xFFFF));

  /* setup pinmux */
  if (device_iomux_setup(dev, ",scl ,sda", NULL, NULL, NULL))
    goto err_mem;

  device_irq_source_init(dev, &pv->irq_ep, 1, &bcm2835_i2c_irq,
                         DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_mem;

  /* Get accessor on timer */
  if (device_get_param_dev_accessor(dev, "i2c-timer", &pv->timer, DRIVER_CLASS_TIMER))
    device_init_accessor(&pv->timer);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_mem:
  mem_free(pv);
  return -EINVAL;
}

static DEV_CLEANUP(bcm2835_i2c_cleanup)
{
  struct bcm2835_i2c_context_s    *pv;

  pv = dev->drv_pv;

  /* disable I2C device. */
  cpu_mem_write_32(pv->addr + BCM2835_I2C_C_ADDR, 0);  

  /* disable interrupt. */
  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* deallocate private driver context. */
  mem_free(pv);
}


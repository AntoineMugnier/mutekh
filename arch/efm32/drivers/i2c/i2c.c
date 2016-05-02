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
#include <device/clock.h>

#include <arch/efm32/i2c.h>

#define EFM32_I2C_IRQ_MASK 0x5A8 

  /* Interrupt Mask 
  EFM32_I2C_IEN_RXDATAV |
  EFM32_I2C_IEN_TXC     |
  EFM32_I2C_IEN_BUSERR  |
  EFM32_I2C_IEN_MSTOP   |
  EFM32_I2C_IEN_NACK; */


enum efm32_i2c_state_e
{
  DEV_I2C_EFM32_IDLE,                    
  DEV_I2C_EFM32_START,
  DEV_I2C_EFM32_WRITE_DATA,
  DEV_I2C_EFM32_READ_DATA,
  DEV_I2C_EFM32_STOP,
  DEV_I2C_EFM32_END,
};

struct efm32_i2c_context_s
{
  uintptr_t addr;
  /* Interrupt end-point */
  struct dev_irq_src_s irq_ep;
  /* request queue */
  dev_request_queue_root_t queue;
  /* transfert size */
  uint16_t len;
  /* transfert count */
  uint8_t cnt;
  /* Fsm state */
  enum efm32_i2c_state_e state;
  /* Bit rate */
  uint32_t bit_rate;
  /* Core frequency */
  struct dev_freq_s freq;

  struct dev_clock_sink_ep_s clk_ep;
};

static void efm32_i2c_update_rate(struct efm32_i2c_context_s *pv)
{
  /* NLOW + NHIGH */ 
  static const uint32_t clhr = 4 + 4;

  /* FSCL = FHFPERCLK /(((NLOW + NHIGH ) x (DIV + 1)) + 4) */ 
  uint32_t div = ((pv->freq.num / (pv->bit_rate * pv->freq.denom)) - 4) / clhr - 1;
  cpu_mem_write_32(pv->addr + EFM32_I2C_CLKDIV_ADDR, endian_le32(div));
}

static inline uint32_t efm32_i2c_get_txc(struct efm32_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + EFM32_I2C_STATUS_ADDR);
   return (endian_le32(x) & EFM32_I2C_STATUS_TXC);
}

static inline uint32_t efm32_i2c_get_txbl(struct efm32_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + EFM32_I2C_STATUS_ADDR);
   return (endian_le32(x) & EFM32_I2C_STATUS_TXBL);
}

static inline uint32_t efm32_i2c_get_rxdatav(struct efm32_i2c_context_s *pv)
{
   uint32_t x = cpu_mem_read_32(pv->addr + EFM32_I2C_STATUS_ADDR);
   return (endian_le32(x) & EFM32_I2C_STATUS_RXDATAV);
}

static void efm32_i2c_fsm(struct efm32_i2c_context_s *pv, bool_t stop)
{
  uint32_t x;

  while(1)
    {
        struct dev_request_s *base = dev_request_queue_head(&pv->queue);
        struct dev_i2c_rq_s *rq = dev_i2c_rq_s_cast(base);
        struct dev_i2c_transfer_s *ctr = rq->transfer + pv->cnt;
        
        if (rq == NULL)
          return;

//        printk("state: %d\n", pv->state);
        switch (pv->state)
          {
            case DEV_I2C_EFM32_IDLE: /* 0 */
              rq->error = 0;
              rq->error_transfer = 0;
              rq->error_offset = 0;
              pv->cnt = 0;
              pv->state = DEV_I2C_EFM32_START;
              break;

            case DEV_I2C_EFM32_START: /* 1 */

              /* TX buffer must be empty when restart */
              if (!efm32_i2c_get_txbl(pv))
                return;

              /** Send start condition */ 
              x = EFM32_I2C_CMD_START; 
              cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(x));

              pv->state = DEV_I2C_EFM32_STOP;
              pv->len = ctr->size;

              /* Prepare address byte */
              x = (rq->saddr << 1) & 0xFE;

              if (ctr->size)
                {
                  if (ctr->type == DEV_I2C_WRITE)
                    pv->state = DEV_I2C_EFM32_WRITE_DATA;
                  else
                    {
                      pv->state = DEV_I2C_EFM32_READ_DATA;
                      x |= 1; 
                    }
                }

              /* Send address byte */
              cpu_mem_write_32(pv->addr + EFM32_I2C_TXDATA_ADDR, endian_le32(x));

              break;
            
            case DEV_I2C_EFM32_WRITE_DATA:{ /* 2 */

              if (!efm32_i2c_get_txbl(pv))
                return;

              x = ctr->data[ctr->size - pv->len];
              cpu_mem_write_32(pv->addr + EFM32_I2C_TXDATA_ADDR, endian_le32(x)); 
              pv->len--;

              if (!pv->len)
                {
                  pv->cnt++;

                  if (pv->cnt == rq->transfer_count)
                    {
                      pv->state = DEV_I2C_EFM32_STOP;
                      break;
                    }

                  struct dev_i2c_transfer_s * next = ctr + 1;

                  if (next->type != ctr->type)
                    {
                      /* Restart must be send */
                      pv->state = DEV_I2C_EFM32_START;
                      break;
                    }

                  ctr = next;
                  pv->len = ctr->size;
                }
              
              break;}

            case DEV_I2C_EFM32_READ_DATA:{ /* 3 */

              if (!efm32_i2c_get_rxdatav(pv))
                return;

              x = cpu_mem_read_32(pv->addr + EFM32_I2C_RXDATA_ADDR);
              ctr->data[ctr->size - pv->len] = endian_le32(x);
              pv->len--;

              /* Acknoledge */
              x = EFM32_I2C_CMD_ACK;

              if (!pv->len)
                {
                  pv->cnt++;

                  if (pv->cnt == rq->transfer_count)
                    {
                      x = EFM32_I2C_CMD_NACK; 
                      pv->state = DEV_I2C_EFM32_STOP;
                    }
                  else
                    {
                      struct dev_i2c_transfer_s * next = ctr + 1;
                     
                      if (next->type != ctr->type)
                        {
                          x = EFM32_I2C_CMD_NACK; 
                          /* Restart must be send */
                          pv->state = DEV_I2C_EFM32_START;
                        }

                      pv->len = next->size;
                    }
                }

              cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(x));

              break;}

            case DEV_I2C_EFM32_STOP: /* 4 */
              /* TX buffer must be empty. */
              if (!efm32_i2c_get_txc(pv))
                return;

              /* Send stop condition. */
              cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(EFM32_I2C_CMD_STOP));
              pv->state = DEV_I2C_EFM32_END;
              break;

            
            case DEV_I2C_EFM32_END: /* 5 */
              if (stop)
                {
                  /* Clear controller. */
                  x = EFM32_I2C_CMD_ABORT |
                      EFM32_I2C_CMD_CLEARTX |
                      EFM32_I2C_CMD_CLEARPC;

                  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(x));

                  pv->state = DEV_I2C_EFM32_IDLE;
                }
              return;

          } 
    }
}

/***************************************** config */

static DEV_I2C_CONFIG(efm32_i2c_config)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_i2c_context_s    *pv  = dev->drv_pv;

  pv->bit_rate = config->bit_rate;
  efm32_i2c_update_rate(pv);

  return 0;
}

/***************************************** transfer */

static DEV_IRQ_SRC_PROCESS(efm32_i2c_irq)
{
  struct device_s             *dev = ep->base.dev;
  struct efm32_i2c_context_s  *pv  = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_request_s *base = dev_request_queue_head(&pv->queue);
  struct dev_i2c_rq_s *rq = dev_i2c_rq_s_cast(base);

  bool_t stop = 0;

  while (1)
    {
      /* get interurpt flags */
      uint32_t x = endian_le32(cpu_mem_read_32(pv->addr + EFM32_I2C_IF_ADDR));
      x &= EFM32_I2C_IRQ_MASK;

      /* Reset interrupts flags */
      cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, endian_le32(EFM32_I2C_IFC_MASK));

      if (!x || (rq == NULL))
        break;
     
      if (x & (EFM32_I2C_IF_MSTOP | EFM32_I2C_IF_BUSERR | EFM32_I2C_IF_NACK))
        stop = 1;
      
      if (x & (EFM32_I2C_IF_BUSERR | EFM32_I2C_IF_NACK))
        {
           rq->error = -EHOSTUNREACH;
           rq->error_transfer = pv->cnt;
           rq->error_offset = 0;
           pv->state = DEV_I2C_EFM32_END;
        }

      efm32_i2c_fsm(pv, stop);

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

      if (rq != NULL && (pv->state == DEV_I2C_EFM32_IDLE))
        efm32_i2c_fsm(pv, 0);
    }

  lock_release(&dev->lock);
}

static DEV_I2C_REQUEST(efm32_i2c_request)
{
  struct device_s               *dev = accessor->dev;
  struct efm32_i2c_context_s    *pv  = dev->drv_pv;

  assert(req->transfer_count);

  LOCK_SPIN_IRQ(&dev->lock);

  bool_t start = dev_request_queue_isempty(&pv->queue);
 
  dev_request_queue_pushback(&pv->queue, &req->base);

  if (start)
    efm32_i2c_fsm(pv, 0);

  LOCK_RELEASE_IRQ(&dev->lock);
}


static DEV_USE(efm32_i2c_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_NOTIFY: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct efm32_i2c_context_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      efm32_i2c_update_rate(pv);
      return 0;
    }
#endif

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(efm32_i2c_init)
{
  struct efm32_i2c_context_s    *pv;


  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  /* enable clock */
  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_SINK_NOTIFY |
                     DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_SINK_SYNC, &pv->freq))
    goto err_mem;

  dev_request_queue_init(&pv->queue);

  /* retreive the device base address from device tree. */
  if(device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_clk;

  /* Reset Device by disabling controller */
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, 0);  
  
  /* Send ABORT command as specified in reference manual*/
  uint32_t x = EFM32_I2C_CMD_ABORT |
               EFM32_I2C_CMD_CLEARTX |
               EFM32_I2C_CMD_CLEARPC;

  cpu_mem_write_32(pv->addr + EFM32_I2C_CMD_ADDR, endian_le32(x));

  /* Disable and clear interrupts */
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0);  

  /* setup pinmux */
  iomux_demux_t loc[2];
  if (device_iomux_setup(dev, ",scl ,sda", loc, NULL, NULL))
    goto err_clk;

  uint32_t route = EFM32_I2C_ROUTE_SCLPEN | EFM32_I2C_ROUTE_SDAPEN;

  EFM32_I2C_ROUTE_LOCATION_SETVAL(route, loc[0]);

  cpu_mem_write_32(pv->addr + EFM32_I2C_ROUTE_ADDR, endian_le32(route));

  device_irq_source_init(dev, &pv->irq_ep, 1, &efm32_i2c_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_clk;

  pv->bit_rate = 100000;
  efm32_i2c_update_rate(pv);

  /* Enable controller - Send stop on nack reception - clock stretching timeout */
  x = EFM32_I2C_CTRL_EN | EFM32_I2C_CTRL_AUTOSN;
       
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, endian_le32(x));  

  /* Clear interrupts flags */
  cpu_mem_write_32(pv->addr + EFM32_I2C_IFC_ADDR, endian_le32(EFM32_I2C_IFC_MASK));
  
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, endian_le32(EFM32_I2C_IRQ_MASK)); 


  return 0;

 err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
 err_mem:
  mem_free(pv);
  return -EINVAL;
}

static DEV_CLEANUP(efm32_i2c_cleanup)
{
  struct efm32_i2c_context_s    *pv;

  pv = dev->drv_pv;

  /* disable I2C device and clear interurpts. */
  cpu_mem_write_32(pv->addr + EFM32_I2C_CTRL_ADDR, 0);  
  cpu_mem_write_32(pv->addr + EFM32_I2C_IEN_ADDR, 0);  
  cpu_mem_write_32(pv->addr + EFM32_I2C_IF_ADDR, 0);  

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* Destroy request queue */
  dev_request_queue_destroy(&pv->queue);

  /* deallocate private driver context. */
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(efm32_i2c_drv, 0, "EFM32 i2c", efm32_i2c,
               DRIVER_I2C_METHODS(efm32_i2c));

DRIVER_REGISTER(efm32_i2c_drv);


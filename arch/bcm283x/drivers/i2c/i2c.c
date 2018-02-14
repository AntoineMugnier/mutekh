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
    Copyright Vincent Defilippi <vincentdefilippi@gmail.com> (c) 2016
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

#include <arch/bcm283x/i2c.h>

#define BCM283X_I2C_CORE_CLK  150000000

enum bcm283x_i2c_state_e
{
  BCM283X_I2C_STATE_IDLE,
  BCM283X_I2C_STATE_WRITE,
  BCM283X_I2C_STATE_READ,
  BCM283X_I2C_STATE_WAIT_TIMEOUT,
};

struct bcm283x_i2c_transfer_s
{
  uint16_t  saddr;
  uint8_t   *data;
  uint16_t  size;
  enum dev_i2c_op_e BITFIELD(type, 4);
};

DRIVER_PV(struct bcm283x_i2c_private_s
{
  uintptr_t                       addr;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s      clk_ep;
#endif
  struct dev_i2c_ctrl_context_s   i2c_ctrl_ctx;
  struct device_timer_s           dev_timer;
  struct dev_timer_rq_s           timer_rq;

  struct dev_irq_src_s            irq_ep;
  uint32_t                        bitrate;
  enum bcm283x_i2c_state_e        state;

  uint16_t                        byte_i;
  uint8_t                         tr_i;
  uint8_t                         tr_cnt;
  struct dev_i2c_ctrl_transfer_s  *last_tr;
  struct bcm283x_i2c_transfer_s   tr[CONFIG_DRIVER_BCM283X_I2C_MAX_TRANSFER];

  uint16_t                        total_size;
  uint16_t                        total_byte_i;
  bool_t                          restart;
});

static inline void
bcm283x_i2c_disable_tx_irq(struct bcm283x_i2c_private_s *pv)
{
  uint32_t x;
  x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_C_ADDR));
  x &= ~BCM283X_I2C_C_INTT;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));
}

static inline void
bcm283x_i2c_enable_tx_irq(struct bcm283x_i2c_private_s *pv)
{
  uint32_t x;
  x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_C_ADDR));
  x |= BCM283X_I2C_C_INTT;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));
}


static inline void
bcm283x_i2c_setup_next_transfer(struct bcm283x_i2c_private_s *pv)
{
  pv->total_byte_i = 0;
  pv->total_size = 0;
  pv->restart = 0;

  uint8_t i;
  for (i = pv->tr_i; i < pv->tr_cnt; i++)
    {
      pv->total_size += pv->tr[i].size;
      uint8_t ending = pv->tr[i].type & _DEV_I2C_ENDING_MASK;
      if (ending != _DEV_I2C_CONTINUOUS)
        {
          if (ending == _DEV_I2C_RESTART)
            pv->restart = 1;
          break;
        }
    }
}

static void
bcm283x_i2c_start_read(struct bcm283x_i2c_private_s *pv)
{
  uint32_t x;

  pv->state = BCM283X_I2C_STATE_READ;

  /* Set slave address */
  cpu_mem_write_32(pv->addr + BCM283X_I2C_A_ADDR, endian_le32(pv->tr[pv->tr_i].saddr & 0x7F));

  /* Set transfer size */
  bcm283x_i2c_setup_next_transfer(pv);
  cpu_mem_write_32(pv->addr + BCM283X_I2C_DLEN_ADDR, endian_le32(pv->total_size + pv->restart));

  /* Clear fifo and start transfer */
  x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_C_ADDR));
  x |= BCM283X_I2C_C_ST | BCM283X_I2C_C_CLEAR | BCM283X_I2C_C_READ;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));
}

static void
bcm283x_i2c_start_write(struct bcm283x_i2c_private_s *pv)
{

  uint32_t x;

  pv->state = BCM283X_I2C_STATE_WRITE;

  /* Set slave address */
  cpu_mem_write_32(pv->addr + BCM283X_I2C_A_ADDR, endian_le32(pv->tr[pv->tr_i].saddr & 0x7F));

  /* Set transfer size */
  bcm283x_i2c_setup_next_transfer(pv);
  cpu_mem_write_32(pv->addr + BCM283X_I2C_DLEN_ADDR, endian_le32(pv->total_size + pv->restart));

  /* Start transfer */
  x = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_C_ADDR));
  x |= BCM283X_I2C_C_ST | BCM283X_I2C_C_CLEAR;
  x &= ~BCM283X_I2C_C_READ;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));

  /* Load fifo */
  while (endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR) & BCM283X_I2C_S_TXD))
    {
      uint8_t data = pv->tr[pv->tr_i].data[pv->byte_i];
      cpu_mem_write_32(pv->addr + BCM283X_I2C_FIFO_ADDR, endian_le32(data));
      pv->total_byte_i++;
      pv->byte_i++;
      if (pv->byte_i == pv->tr[pv->tr_i].size)
        {
          pv->byte_i = 0;
          pv->tr_i++;
        }
      if (pv->total_byte_i == pv->total_size)
        break;
    }
}

static
KROUTINE_EXEC(bcm283x_i2c_timeout)
{
  struct dev_timer_rq_s         *rq = KROUTINE_CONTAINER(kr, *rq, rq.kr);
  struct device_s               *dev = rq->rq.pvdata;
  struct bcm283x_i2c_private_s  *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  uint32_t dlen = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_DLEN_ADDR));
  if (dlen > 1)
    {
      dev_timer_delay_t byte_delay_us = 8 * 1000000 * (dlen - 1) / pv->bitrate;
      dev_timer_init_sec(&pv->dev_timer, &pv->timer_rq.delay, 0, byte_delay_us, 1000000);
      DEVICE_OP(&pv->dev_timer, request, &pv->timer_rq);
      goto out;
    }

  if ((pv->tr[pv->tr_i].type & _DEV_I2C_READ_OP))
    bcm283x_i2c_start_read(pv);
  else
    bcm283x_i2c_start_write(pv);

  bcm283x_i2c_enable_tx_irq(pv);

out:
  LOCK_RELEASE_IRQ(&dev->lock);
}

static
DEV_IRQ_SRC_PROCESS(bcm283x_i2c_irq)
{
  struct device_s                  *dev = ep->base.dev;
  struct bcm283x_i2c_private_s     *pv  = dev->drv_pv;

  lock_spin(&dev->lock);

  while (1)
    {
      volatile uint32_t status = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR));
      uint32_t x = BCM283X_I2C_S_CLKT | BCM283X_I2C_S_ERR | BCM283X_I2C_S_DONE;
      cpu_mem_write_32(pv->addr + BCM283X_I2C_S_ADDR, endian_le32(x));

      status &= (BCM283X_I2C_S_ERR | BCM283X_I2C_S_RXR | BCM283X_I2C_S_TXW | BCM283X_I2C_S_DONE);

      if (pv->state == BCM283X_I2C_STATE_WAIT_TIMEOUT)
        break;

      if (!status)
        break;

      if (status & BCM283X_I2C_S_ERR)
        {
          if (pv->total_byte_i == 0)
            pv->last_tr->err = -EHOSTUNREACH;
          else
            pv->last_tr->err = -EAGAIN;
          kroutine_exec(&pv->last_tr->kr);
          pv->last_tr = NULL;
          pv->tr_cnt = 0;
          pv->state = BCM283X_I2C_STATE_IDLE;
        }
      else if (status & BCM283X_I2C_S_DONE)
        {
          if (pv->state == BCM283X_I2C_STATE_READ)
            {
              while (pv->total_byte_i < pv->total_size)
                {
                  uint8_t data = cpu_mem_read_32(pv->addr + BCM283X_I2C_FIFO_ADDR);
                  pv->tr[pv->tr_i].data[pv->byte_i] = data;
                  pv->total_byte_i++;
                  pv->byte_i++;
                  if (pv->byte_i == pv->tr[pv->tr_i].size)
                    {
                      pv->byte_i = 0;
                      pv->tr_i++;
                    }
                }
            }
          pv->last_tr->err = 0;
          kroutine_exec(&pv->last_tr->kr);
          pv->last_tr = NULL;
          pv->tr_cnt = 0;
          pv->state = BCM283X_I2C_STATE_IDLE;
        }
      else if (status & BCM283X_I2C_S_TXW)
        {
          if (pv->restart && pv->total_byte_i == pv->total_size)
            {
              pv->state = BCM283X_I2C_STATE_WAIT_TIMEOUT;
              uint32_t dlen = endian_le32(cpu_mem_read_32(pv->addr + BCM283X_I2C_DLEN_ADDR));
              dev_timer_delay_t byte_delay_us = 8 * 1000000 * (dlen - 1) / pv->bitrate;
              dev_timer_init_sec(&pv->dev_timer, &pv->timer_rq.delay, 0, byte_delay_us, 1000000);
              DEVICE_OP(&pv->dev_timer, request, &pv->timer_rq);
              bcm283x_i2c_disable_tx_irq(pv);
            }
          else
            {
              while (cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR) & BCM283X_I2C_S_TXW)
                {
                  uint8_t data = pv->tr[pv->tr_i].data[pv->byte_i];
                  cpu_mem_write_32(pv->addr + BCM283X_I2C_FIFO_ADDR, endian_le32(data));
                  pv->byte_i++;
                  if (pv->byte_i == pv->tr[pv->tr_i].size)
                    {
                      pv->byte_i = 0;
                      pv->tr_i++;
                    }
                }
            }
        }
      else if (status & BCM283X_I2C_S_RXR)
        {
          while (cpu_mem_read_32(pv->addr + BCM283X_I2C_S_ADDR) & BCM283X_I2C_S_RXR)
            {
              uint8_t data = cpu_mem_read_32(pv->addr + BCM283X_I2C_FIFO_ADDR);
              pv->tr[pv->tr_i].data[pv->byte_i] = data;
              pv->total_byte_i++;
              pv->byte_i++;
              if (pv->byte_i == pv->tr[pv->tr_i].size)
                {
                  pv->byte_i = 0;
                  pv->tr_i++;
                }
            }
        }
    }

  lock_release(&dev->lock);
}

/*----------------------------------------------------------------------------*/

static void
bcm283x_i2c_setup_controller(struct bcm283x_i2c_private_s *pv)
{
  uint32_t x;

  /* Disable controller, disbale interrupts, clear fifo */
  x = BCM283X_I2C_C_CLEAR;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));

  /* Clear status flags */
  x = BCM283X_I2C_S_CLKT | BCM283X_I2C_S_ERR | BCM283X_I2C_S_DONE;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_S_ADDR, endian_le32(x));

  /* Disable clock streching timeout */
  cpu_mem_write_32(pv->addr + BCM283X_I2C_CLKT_ADDR, endian_le32(0));

  /* Setup Bitrate */
  x = BCM283X_I2C_CORE_CLK / pv->bitrate;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_DIV_ADDR, endian_le32(x));

  /* Enable controller, clear fifo and enable interrupts */
  x = BCM283X_I2C_C_I2CEN | BCM283X_I2C_C_CLEAR |
      BCM283X_I2C_C_INTR | BCM283X_I2C_C_INTT | BCM283X_I2C_C_INTD;
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, endian_le32(x));
}

static
DEV_I2C_CTRL_TRANSFER(bcm283x_i2c_transfer)
{
  struct device_s               *dev = accessor->dev;
  struct bcm283x_i2c_private_s  *pv  = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (tr->type == DEV_I2C_RESET)
    {
      if (pv->state == BCM283X_I2C_STATE_WAIT_TIMEOUT)
        DEVICE_OP(&pv->dev_timer, cancel, &pv->timer_rq);
      if (pv->state != BCM283X_I2C_STATE_IDLE)
        bcm283x_i2c_setup_controller(pv);
      tr->err = 0;
      kroutine_exec(&tr->kr);
      pv->tr_cnt = 0;
      pv->state = BCM283X_I2C_STATE_IDLE;
      goto out;
    }

  if (pv->last_tr != NULL)
    {
      tr->err = -EBUSY;
      kroutine_exec(&tr->kr);
      goto out;
    }

  /* SYNC operations are not supported */
  if ((tr->type & _DEV_I2C_ENDING_MASK) != _DEV_I2C_STOP &&
      (tr->type & _DEV_I2C_SYNC))
    {
      if (pv->state != BCM283X_I2C_STATE_IDLE)
        bcm283x_i2c_setup_controller(pv);
      tr->err = -ENOTSUP;
      kroutine_exec(&tr->kr);
      pv->tr_cnt = 0;
      pv->state = BCM283X_I2C_STATE_IDLE;
      goto out;
    }

  /* Read RESTART is not supported */
  /* Write RESTART is supported only if a timer resource is defined */
  if ((tr->type & _DEV_I2C_ENDING_MASK) == _DEV_I2C_RESTART &&
      (tr->type & _DEV_I2C_READ_OP || !device_check_accessor(&pv->dev_timer.base)))
    {
      if (pv->state != BCM283X_I2C_STATE_IDLE)
        bcm283x_i2c_setup_controller(pv);
      tr->err = -ENOTSUP;
      kroutine_exec(&tr->kr);
      pv->tr_cnt = 0;
      pv->state = BCM283X_I2C_STATE_IDLE;
      goto out;
    }

  /* Check transfer limit */
  if (pv->tr_cnt >= CONFIG_DRIVER_BCM283X_I2C_MAX_TRANSFER)
    {
      if (pv->state != BCM283X_I2C_STATE_IDLE)
        bcm283x_i2c_setup_controller(pv);
      tr->err = -ENOTSUP;
      kroutine_exec(&tr->kr);
      pv->tr_cnt = 0;
      pv->state = BCM283X_I2C_STATE_IDLE;
      goto out;
    }

  /* Continuous transfers with differents directions are forbidden */
  if (pv->tr_cnt > 0 &&
      (pv->tr[pv->tr_cnt - 1].type & _DEV_I2C_ENDING_MASK) == _DEV_I2C_CONTINUOUS)
    {
      if ((pv->tr[pv->tr_cnt - 1].type & _DEV_I2C_READ_OP) != (tr->type & _DEV_I2C_READ_OP))
        {
          if (pv->state != BCM283X_I2C_STATE_IDLE)
            bcm283x_i2c_setup_controller(pv);
          tr->err = -ENOTSUP;
          kroutine_exec(&tr->kr);
          pv->tr_cnt = 0;
          pv->state = BCM283X_I2C_STATE_IDLE;
          goto out;
        }
    }

    pv->tr[pv->tr_cnt].saddr = tr->saddr;
    pv->tr[pv->tr_cnt].data = tr->data;
    pv->tr[pv->tr_cnt].size = tr->size;
    pv->tr[pv->tr_cnt].type = tr->type;
    pv->tr_cnt++;

  if ((tr->type & _DEV_I2C_ENDING_MASK) != _DEV_I2C_STOP)
    {
      tr->err = 0;
      kroutine_exec(&tr->kr);
      goto out;
    }

  pv->last_tr = tr;
  pv->byte_i = 0;
  pv->tr_i = 0;
  tr->err = 0;

  if ((pv->tr[pv->tr_i].type & _DEV_I2C_READ_OP))
    bcm283x_i2c_start_read(pv);
  else
    bcm283x_i2c_start_write(pv);

  out:
  LOCK_RELEASE_IRQ(&dev->lock);
}
/*----------------------------------------------------------------------------*/

#define bcm283x_i2c_use dev_use_generic

static
DEV_INIT(bcm283x_i2c_init)
{
  struct bcm283x_i2c_private_s    *pv;

  assert(CONFIG_DRIVER_BCM283X_I2C_MAX_TRANSFER <= 255);

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;
/* err_mem */

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if(device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_I2C_REQUEST
  if (dev_drv_i2c_ctrl_context_init(dev, &pv->i2c_ctrl_ctx))
    goto err_mem;
#endif
/* err_ctx */

  if (device_res_get_uint(dev, DEV_RES_I2C_BITRATE, 0, &pv->bitrate, NULL))
    goto err_ctx;

  bcm283x_i2c_setup_controller(pv);

  /* setup pinmux */
  if (device_iomux_setup(dev, ",scl ,sda", NULL, NULL, NULL))
    goto err_ctx;

  /* Link irq */
  device_irq_source_init(dev, &pv->irq_ep, 1, &bcm283x_i2c_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_ctx;
/* err_link */

  if (!device_get_param_dev_accessor(dev, "timer", &pv->dev_timer.base, DRIVER_CLASS_TIMER))
    {
      pv->timer_rq.rev = 0;
      pv->timer_rq.rq.pvdata = dev;
      kroutine_init_deferred(&pv->timer_rq.rq.kr, bcm283x_i2c_timeout);
    }

  pv->state = BCM283X_I2C_STATE_IDLE;

  return 0;

err_ctx:
#ifdef CONFIG_DEVICE_I2C_REQUEST
  dev_drv_i2c_ctrl_context_cleanup(&pv->i2c_ctrl_ctx);
#endif

err_mem:
  mem_free(pv);
  return -EINVAL;
}

static
DEV_CLEANUP(bcm283x_i2c_cleanup)
{
  struct bcm283x_i2c_private_s    *pv = dev->drv_pv;

  if (pv->tr)
    return -EBUSY;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* Disable controller and interrupts */
  cpu_mem_write_32(pv->addr + BCM283X_I2C_C_ADDR, 0);

#ifdef CONFIG_DEVICE_I2C_REQUEST
  dev_drv_i2c_ctrl_context_cleanup(&pv->i2c_ctrl_ctx);
#endif

  device_iomux_cleanup(dev);
  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(bcm283x_i2c_drv, 0, "BCM283X i2c", bcm283x_i2c,
               DRIVER_I2C_CTRL_METHODS(bcm283x_i2c));

DRIVER_REGISTER(bcm283x_i2c_drv);



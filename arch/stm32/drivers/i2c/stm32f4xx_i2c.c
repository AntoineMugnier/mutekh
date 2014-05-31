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

#include <arch/stm32f4xx_regs.h>

enum stm32f4xx_i2c_state_e
{
  DEV_I2C_ST_START   = 0,
  DEV_I2C_ST_ADDR    = 1,
  DEV_I2C_ST_RREADY  = 2,
  DEV_I2C_ST_RADDR   = 3,
  DEV_I2C_ST_WRITE_N = 4,
  DEV_I2C_ST_READ_2  = 5,
  DEV_I2C_ST_READ_N  = 6,
  DEV_I2C_ST_STOP    = 7,

  DEV_I2C_ST_NONE    = 8,
  DEV_I2C_ST_COUNT   = DEV_I2C_ST_NONE
};

/* the phase is used to detect first or second sending of slave
   address in the case of read transfer. */
enum stm32f4xx_i2c_phase_e
{
  DEV_I2C_PHASE_RW,
  DEV_I2C_PHASE_R,
};

struct stm32f4xx_i2c_context_s
{
  /* device base address. */
  uintptr_t                      addr;

  /* current transfer. */
  struct dev_i2c_ctrl_transfer_s *tr;

  /* current fsm state. */
  enum stm32f4xx_i2c_state_e     state;

  /* current fsm phase (rw = phase 1, r = phase 2). */
  enum stm32f4xx_i2c_phase_e     phase;

  /* byte count. */
  size_t                         nbytes;

#if defined(CONFIG_DEVICE_IRQ)
  /* interrupt end-point. */
  struct dev_irq_ep_s            irq_ep[2];
#endif
};


/***************************************** config */

DEVI2C_CTRL_CONFIG(stm32f4xx_i2c_config)
{
  return -ENOTSUP;
}


/***************************************** transfer */

#define DEVI2C_CTRL_FSM(n) void (n) (struct device_s *dev)

typedef DEVI2C_CTRL_FSM(stm32f4xx_i2c_fsm_callback_t);

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_start);
static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_slave_addr);
static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_slave_reg_addr0);
static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_slave_reg_addr1);
static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_writeN);
static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_read2);
static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_readN);

static stm32f4xx_i2c_fsm_callback_t * const stm32f4xx_i2c_fsm[] =
{
  [DEV_I2C_ST_START]   = &stm32f4xx_i2c_ev_start,
  [DEV_I2C_ST_ADDR]    = &stm32f4xx_i2c_ev_slave_addr,
  [DEV_I2C_ST_RREADY]  = &stm32f4xx_i2c_ev_slave_reg_addr0,
  [DEV_I2C_ST_RADDR]   = &stm32f4xx_i2c_ev_slave_reg_addr1,
  [DEV_I2C_ST_WRITE_N] = &stm32f4xx_i2c_ev_writeN,
  [DEV_I2C_ST_READ_2]  = &stm32f4xx_i2c_ev_read2,
  [DEV_I2C_ST_READ_N]  = &stm32f4xx_i2c_ev_readN,
};

static bool_t stm32f4xx_i2c_check_error(struct device_s *dev, uint32_t status)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;

  if (status & 0xff00)
    {
      if (status & STM32F4xx_I2C_SR1_TIMEOUT)
        pv->tr->error = ETIMEDOUT;
      else if (status & STM32F4xx_I2C_SR1_AF)
        pv->tr->error = EBADDATA;
      else if (status & STM32F4xx_I2C_SR1_BERR)
        pv->tr->error = EIO;
      else if (status & STM32F4xx_I2C_SR1_ARLO)
        pv->tr->error = EPERM;
      else
        pv->tr->error = EUNKNOWN;
      return 1;
    }
  return 0;
}

#if !defined(CONFIG_DEVICE_IRQ)

/* if the device library does not support the interrupts, the driver uses
   a polling-based mechanism for validating each protocol step. */
# define STM32F4xx_I2C_CHECK_ERROR(__dev, __pv, __flag)                      \
  do                                                                         \
    {                                                                        \
      while (STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, __flag) == 0) \
        {                                                                    \
           uint32_t status = STM32F4xx_REG_VALUE_DEV(I2C, __pv->addr, SR1);  \
           if (stm32f4xx_i2c_check_error(__dev, status))                     \
             {                                                               \
               /* clear errors. */                                           \
               STM32F4xx_REG_UPDATE_DEV(I2C, __pv->addr, SR1, 0);            \
                                                                             \
               return;                                                       \
             }                                                               \
        }                                                                    \
    } while (0)                                                              \
/**/

#endif


/****************************************** START SENT */

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_start)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr = pv->tr;

#if defined(CONFIG_DEVICE_IRQ)
  /* check for state consistency. */
  if (!STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, SB))
    {
      /* step to end state. */
      pv->state = DEV_I2C_ST_STOP;

      tr->error = EIO;
      return;
    }
#endif

  /* step to next phase. */
  pv->state = DEV_I2C_ST_ADDR;

  /* write the slave address. */
  uint8_t saddr = tr->saddr << 1;
  switch (pv->phase)
    {
    default: break;

    case DEV_I2C_PHASE_RW:
      //saddr |= 0x0; /* always do a write in phase 1. */
      break;

    case DEV_I2C_PHASE_R:
      saddr |= 0x1; /* do a read in phase 2. */
      break;
    }

  /* write the slave address. */
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, DR, DATA, saddr);

#if !defined(CONFIG_DEVICE_IRQ)
  /* wait for the address to be sent (and check for errors). */
  STM32F4xx_I2C_CHECK_ERROR(dev, pv, ADDRESS);
#endif
}


/****************************************** ADDR SENT (EV6) */

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_slave_addr)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr = pv->tr;

#if defined(CONFIG_DEVICE_IRQ)
  /* check for state consistency. */
  if (!STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, ADDRESS))
    return;
#endif

  /* step to next phase. */
  switch (pv->phase)
    {
    default:
      assert(0 && "non reachable.");
      break;

    /********************* Phase 1: read/write. */
    case DEV_I2C_PHASE_RW:
#if defined(CONFIG_DEVICE_IRQ)
      /* disable buffer interrupt. */
      STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITBUFEN);
#endif
      /* clear address sent interrupt. */
      (void) STM32F4xx_REG_VALUE_DEV(I2C, pv->addr, SR2);

      /* step to next phase. */
      pv->state = DEV_I2C_ST_RREADY;

#if !defined(CONFIG_DEVICE_IRQ)
      STM32F4xx_I2C_CHECK_ERROR(dev, pv, TXE);
#endif
      break;

    /********************* Phase 2: read only. */
    case DEV_I2C_PHASE_R:
#if defined(CONFIG_DEVICE_IRQ)
      /* disable buffer interrupt. */
      STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);
#endif

      /* 1-byte read. */
      if (tr->count == 1)
        {
          /* clear acknowledge to sent a NACK immediately. */
          STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, ACK);

          /* clear address sent interrupt. */
          (void) STM32F4xx_REG_VALUE_DEV(I2C, pv->addr, SR2);

          /* step to next phase. */
          pv->state = DEV_I2C_ST_STOP;

          /* send STOP condition. */
          STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);

          /* read the data. */
          tr->data[0] = STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, DR, DATA);
          --tr->count;
        }
      /* 2-byte read. */
      else if (tr->count == 2)
        {
          /* clear ACK and set POS. */
          STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, ACK);
          STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, POS);

          /* clear address sent interrupt. */
          (void) STM32F4xx_REG_VALUE_DEV(I2C, pv->addr, SR2);

          /* step to next phase. */
          pv->state = DEV_I2C_ST_READ_2;

#if !defined(CONFIG_DEVICE_IRQ)
          STM32F4xx_I2C_CHECK_ERROR(dev, pv, BTF);
#endif
        }
      else
        {
          /* clear address sent interrupt. */
          (void) STM32F4xx_REG_VALUE_DEV(I2C, pv->addr, SR2);

          /* set ACK. */
          STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, ACK);

          /* step to next phase. */
          pv->state = DEV_I2C_ST_READ_N;

#if defined(CONFIG_DEVICE_IRQ)
          /* enable buffer interrupt for data before the final sequence. */
          if (tr->count > 3)
            STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITBUFEN);
#endif

#if !defined(CONFIG_DEVICE_IRQ)
          if (tr->count == 3)
            STM32F4xx_I2C_CHECK_ERROR(dev, pv, BTF);
          else
            STM32F4xx_I2C_CHECK_ERROR(dev, pv, RXNE);
#endif
        }
      break;
    }
}


/****************************************** REG ADDR READY (EV8_1) */

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_slave_reg_addr0)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr = pv->tr;

#if defined(CONFIG_DEVICE_IRQ)
  /* check for state consistency. */
  if (!STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, TXE))
    return;
#endif

  /* step to next phase. */
  pv->state = DEV_I2C_ST_RADDR;

#if defined(CONFIG_DEVICE_IRQ)
  if (tr->count > 0 && tr->dir == DEV_I2C_TR_WRITE)
    /* enable buffer interrupt. */
    STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITBUFEN);
#endif

  /* write the slave register address. */
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, DR, DATA, tr->sraddr);

#if !defined(CONFIG_DEVICE_IRQ)
  if (tr->count == 0 || tr->dir == DEV_I2C_TR_READ)
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, BTF);
  else
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, TXE);
#endif
}


/****************************************** REG ADDR SENT (EV_8) */

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_slave_reg_addr1)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr = pv->tr;

#if defined(CONFIG_DEVICE_IRQ)
  /* check for state consistency. */
  if ((tr->count == 0 || tr->dir == DEV_I2C_TR_READ) &&
      !STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, BTF))
    {
      return;
    }
  if (tr->count > 0 && tr->dir == DEV_I2C_TR_WRITE &&
      !STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, TXE))
    {
      return;
    }
#endif

  /* if there is no data, stop here. */
  if (tr->count == 0)
    {
      /* step to next phase. */
      pv->state = DEV_I2C_ST_STOP;

      /* send STOP condition. */
      STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);

      return;
    }

  /* if this is a read transfer, then a restart is required. */
  if (tr->dir == DEV_I2C_TR_READ)
    {
      /* step to next phase. */
      pv->state = DEV_I2C_ST_START;
      pv->phase = DEV_I2C_PHASE_R;

      /* send START condition. */
      STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, START);

      return;
    }

  /* here we are sure the transfer is a write of at least 1 byte. */

  /* prepare. */
  pv->nbytes = 0;

#if defined(CONFIG_DEVICE_IRQ)
  if (tr->count == 1)
    /* disable buffer interrupt. */
    STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);
#endif

  /* step to next phase. */
  pv->state = DEV_I2C_ST_WRITE_N;

  /* write first byte. */
  STM32F4xx_REG_FIELD_UPDATE_DEV(
    I2C,
    pv->addr,
    DR,
    DATA,
    tr->data[pv->nbytes++]
  );

  --tr->count;

#if !defined(CONFIG_DEVICE_IRQ)
  if (tr->count == 0)
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, BTF);
  else
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, TXE);
#endif
}


/****************************************** WRITE N */

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_writeN)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr = pv->tr;

#if defined(CONFIG_DEVICE_IRQ)
  /* wait for BTF == 1. */
  if (tr->count == 0 &&
      !STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, BTF))
    {
      return;
    }
  /* wait for TXE == 1. */
  else if (tr->count > 0 &&
           !STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, TXE))
    {
      return;
    }
#endif

  /* we reached the end of the transfer. */
  if (tr->count == 0)
    {
      /* step to the next phase. */
      pv->state = DEV_I2C_ST_STOP;

      /* send STOP condition. */
      STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);

      return;
    }

#if defined(CONFIG_DEVICE_IRQ)
  /* disable buffer interrupt before writing the last byte. */
  if (tr->count == 1)
    STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);
#endif

  /* write the next byte to data register. */
  STM32F4xx_REG_FIELD_UPDATE_DEV(
    I2C,
    pv->addr,
    DR,
    DATA,
    tr->data[pv->nbytes++]
  );

  --tr->count;

#if !defined(CONFIG_DEVICE_IRQ)
  if (tr->count == 0)
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, BTF);
  else
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, TXE);
#endif
}


/****************************************** READ 2 bytes */

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_read2)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr = pv->tr;

#if defined(CONFIG_DEVICE_IRQ)
  /* wait for BTF == 1. */
  if (!STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, BTF))
    return;
#endif

  assert(pv->nbytes < tr->count);
  assert((tr->count - pv->nbytes) == 2);

  /* step to next phase. */
  pv->state = DEV_I2C_ST_STOP;

  /* send STOP condition. */
  STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);

  /* read the two last data. */
  while (pv->nbytes < tr->count)
    {
      tr->data[pv->nbytes++] = STM32F4xx_REG_FIELD_VALUE_DEV(
        I2C,
        pv->addr,
        DR,
        DATA
      );

      --tr->count;
    }
}


/****************************************** READ N bytes */

static DEVI2C_CTRL_FSM(stm32f4xx_i2c_ev_readN)
{
  struct stm32f4xx_i2c_context_s    *pv = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr = pv->tr;

#if defined(CONFIG_DEVICE_IRQ)
  /* if it remains only three bytes, wait for BTF == 1. */
  if (tr->count == 3 &&
      !STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, BTF))
    {
      return;
    }

  /* otherwise, wait for RXNE == 1. */
  else if (tr->count > 3 &&
           !STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, RXNE))
    {
      return;
    }
#endif

  /* read the data. */
  if (tr->count == 3)
    {
      /* clear ACK. */
      STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, ACK);

      /* step to next phase. */
      pv->state = DEV_I2C_ST_READ_2;
    }

#if defined(CONFIG_DEVICE_IRQ)
  /* clear buffer interrupt before writing the the N-4 byte as after only
     BTF is relevant. */
  if (tr->count == 4)
    STM32F4xx_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);
#endif

  /* read the data. */
  tr->data[pv->nbytes++] = STM32F4xx_REG_FIELD_VALUE_DEV(
    I2C,
    pv->addr,
    DR,
    DATA
  );

  --tr->count;

#if !defined(CONFIG_DEVICE_IRQ)
  if (tr->count == 3)
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, BTF);
  else
    STM32F4xx_I2C_CHECK_ERROR(dev, pv, RXNE);
#endif
}


/****************************************** transfer */

DEVI2C_CTRL_TRANSFER(stm32f4xx_i2c_transfer)
{
  struct device_s                   *dev = i2cdev->dev;
  struct stm32f4xx_i2c_context_s    *pv  = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL ||
      STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR2, BUSY) != 0)
    {
      tr->error = EBUSY;
    }
  else if (pv->tr->amode == DEV_I2C_ADDR_10_BITS)
    {
      tr->error = ENOTSUP;
    }
  else
    {
      /* start the transfer. */
      tr->error = 0;
      pv->tr    = tr;
      pv->state = DEV_I2C_ST_START;
      pv->phase = DEV_I2C_PHASE_RW;
      STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, ACK);
      STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, START);

#if !defined(CONFIG_DEVICE_IRQ)
      /* wait for the start condition to be sent (and check for errors). */
      STM32F4xx_I2C_CHECK_ERROR(dev, pv, SB);

      stm32f4xx_i2c_fsm_callback_t * trans;
      while (pv->state != DEV_I2C_ST_STOP)
        {
          assert (pv->state < DEV_I2C_ST_COUNT);

          if (tr->error)
            {
              /* step to end state. */
              enum stm32f4xx_i2c_state_e current = pv->state;
              pv->state = DEV_I2C_ST_STOP;

              /* send a stop condition if a transfer is active. */
              if (current > DEV_I2C_ST_START)
                STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);
            }
          else
            {
              /* apply the transition on the fsm. */
              trans = stm32f4xx_i2c_fsm[pv->state];
              trans(dev);
            }
        }

      /* we reached the end, then call the routine. */

      /* reset current transfer. */
      pv->tr = NULL;

      /* reset state. */
      pv->state = DEV_I2C_ST_NONE;

      /* wait to return in slave mode. */
      /* FIXME: this is a hack that is not mentioned in the datasheet.
         However, unless this, the controller looks to produce unexpected
         interrupts even after the STOP condition is sent. */
      while (STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR2, MSL));

      /* call routine. */
      LOCK_RELEASE_IRQ(&dev->lock);
      kroutine_exec(&tr->kr, cpu_is_interruptible());
      return;
#endif
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}


/****************************************** end-point process */

#if defined(CONFIG_DEVICE_IRQ)

static DEV_IRQ_EP_PROCESS(stm32f4xx_i2c_irq)
{
  struct device_s                   *dev = ep->dev;
  struct stm32f4xx_i2c_context_s    *pv  = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s    *tr  = pv->tr;

  lock_spin(&dev->lock);

  /* clear interrupt (read SR1 + read SR2). */
  uint32_t volatile status = STM32F4xx_REG_VALUE_DEV(I2C, pv->addr, SR1);

  assert(status);
  assert(pv->tr != NULL);
  assert(pv->state < DEV_I2C_ST_COUNT);

  /* check for errors. */
  if (stm32f4xx_i2c_check_error(dev, status))
    {
      /* clear errors. */
      STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, SR1, 0);

      /* step to end state. */
      enum stm32f4xx_i2c_state_e current = pv->state;
      pv->state = DEV_I2C_ST_STOP;

      /* send a stop condition if a transfer is active. */
      if (current > DEV_I2C_ST_START)
        STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);
    }
  else
    {
      /* apply the transition on the fsm. */
      stm32f4xx_i2c_fsm_callback_t * trans = stm32f4xx_i2c_fsm[pv->state];
      trans(dev);
    }

  /* if we reached the end, then call the routine (tail call). */
  if (pv->state == DEV_I2C_ST_STOP)
    {
      /* reset current transfer. */
      pv->tr = NULL;

      /* reset state. */
      pv->state = DEV_I2C_ST_NONE;

      /* wait to return in slave mode. */
      /* FIXME: this is a hack that is not mentioned in the datasheet.
         However, unless this, the controller looks to produce unexpected
         interrupts even after the STOP condition is sent. */
      while (STM32F4xx_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR2, MSL));

      /* call routine. */
      lock_release(&dev->lock);
      kroutine_exec(&tr->kr, cpu_is_interruptible());

      return;
    }

  lock_release(&dev->lock);
}

#endif


/****************************************** driver init */

static const struct driver_i2c_ctrl_s stm32f4xx_i2c_drv_cls =
{
  .class_       = DRIVER_CLASS_I2C,
  .f_config     = &stm32f4xx_i2c_config,
  .f_transfer   = &stm32f4xx_i2c_transfer,
};

static DEV_INIT(stm32f4xx_i2c_init);
static DEV_CLEANUP(stm32f4xx_i2c_cleanup);

const struct driver_s stm32f4xx_i2c_drv =
{
  .desc         = "STM32F4xx I2C",
  .f_init       = &stm32f4xx_i2c_init,
  .f_cleanup    = &stm32f4xx_i2c_cleanup,
  .classes      =
  {
    &stm32f4xx_i2c_drv_cls,
    0
  }
};

REGISTER_DRIVER(stm32f4xx_i2c_drv);

static DEV_INIT(stm32f4xx_i2c_init)
{
  struct stm32f4xx_i2c_context_s    *pv;
  uint16_t                          freq_in_mhz;

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

  /* reset the device. */
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR1, 0);
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR2, 0);

  /* configure GPIO. */
  if (device_iomux_setup(dev, ",scl ,sda", NULL, NULL, NULL))
    goto err_mem;

  /* configure input clock. */
  // XXX: will be fixed by clock tree. */
  switch (pv->addr)
  {
  default: assert(0 && "unknown I2C controller");

  case STM32F4xx_I2C1_ADDR:
    STM32F4xx_REG_FIELD_SET(RCC, , APB1ENR, I2C1EN);
    break;

  case STM32F4xx_I2C2_ADDR:
    STM32F4xx_REG_FIELD_SET(RCC, , APB1ENR, I2C2EN);
    break;

  case STM32F4xx_I2C3_ADDR:
    STM32F4xx_REG_FIELD_SET(RCC, , APB1ENR, I2C3EN);
    break;
  }

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_init(
    dev,
    pv->irq_ep,
    2,
    &stm32f4xx_i2c_irq,
    DEV_IRQ_SENSE_HIGH_LEVEL
  );

  if (device_irq_source_link(dev, pv->irq_ep, 2, -1))
    goto err_irq;

  STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITEVTEN);
  STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITERREN);
#endif

  /* initialize the input clock frequency. */
  extern uint32_t stm32f4xx_clock_freq_apb1;

  freq_in_mhz = stm32f4xx_clock_freq_apb1 / 1000000;
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CR2, FREQ, freq_in_mhz);

  /* set standard mode. */
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CCR, FS, SM);

  /* initialize the I2C bus speed (100kHz in standard mode). */
  STM32F4xx_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CCR, CCR, 5 * freq_in_mhz);

  /* initialize the rise time (p. 491 of Reference manual). */
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, TRISE, freq_in_mhz + 1);

  /* enable I2C device. */
  STM32F4xx_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, PE);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

#if defined(CONFIG_DEVICE_IRQ)
err_irq:
#endif

err_mem:
  mem_free(pv);
  return -EINVAL;
}

static DEV_CLEANUP(stm32f4xx_i2c_cleanup)
{
  struct stm32f4xx_i2c_context_s    *pv;

  pv = dev->drv_pv;

  // XXX: Wait for the current transmission.

  /* disable I2C device. */
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR1, 0);
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CR2, 0);
  STM32F4xx_REG_UPDATE_DEV(I2C, pv->addr, CCR, 0);

#if defined(CONFIG_DEVICE_IRQ)
  device_irq_source_unlink(dev, pv->irq_ep, 2);
#endif

  /* deallocate private driver context. */
  mem_free(pv);
}


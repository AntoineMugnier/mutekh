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
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/i2c.h>
#include <device/class/iomux.h>

#include <cpp/device/helpers.h>
#include <arch/stm32_memory_map.h>

#include <arch/stm32_i2c.h>
#include <arch/stm32_rcc.h>


#define STM32_I2C_IRQ_MASK \
  STM32_I2C_SR1_TIMEOUT |  \
  STM32_I2C_SR1_AF      |  \
  STM32_I2C_SR1_BERR    |  \
  STM32_I2C_SR1_SB      |  \
  STM32_I2C_SR1_ADDRESS |  \
  STM32_I2C_SR1_BTF     |  \
  STM32_I2C_SR1_RXNE    |  \
  STM32_I2C_SR1_TXE        \
/**/

enum stm32_i2c_fsm_state_e
{
  DEV_I2C_STM32_IDLE,
  DEV_I2C_STM32_START,
  DEV_I2C_STM32_ADDR,
  DEV_I2C_STM32_DATA,
  DEV_I2C_STM32_WRITE_RDY,
  DEV_I2C_STM32_WRITE_N,
  DEV_I2C_STM32_READ_2_AND_STOP,
  DEV_I2C_STM32_READ_N,
  DEV_I2C_STM32_ERROR,
  DEV_I2C_STM32_STOP,
  DEV_I2C_STM32_END,
};

struct stm32_i2c_private_s
{
  /* device address. */
  uintptr_t                      addr;

  /* current transfer */
  struct dev_i2c_ctrl_transfer_s *tr;

  /* current transfer processed bytes */
  size_t                         nbytes;

  /* Fsm state */
  enum stm32_i2c_fsm_state_e     state;

#if defined(CONFIG_DEVICE_I2C_REQUEST)
  struct dev_i2c_ctrl_sched_s    sched;
#endif

  /* interrupt end-point (event and error). */
  struct dev_irq_ep_s            irq_ep[2];
};


/***************************************** config */

DEV_I2C_CTRL_CONFIG(stm32_i2c_config)
{
  struct device_s *dev = accessor->dev;
  error_t err          = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  if (cfg->bit_rate != DEV_I2C_SPEED_STD)
    err = -ENOTSUP;

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}


/***************************************** transfer */

// FSM callbacks.
#define DEV_I2C_CTRL_FSM(n) bool_t (n) (struct stm32_i2c_private_s *pv)

/** @This function defines a callback for the i2c controller FSM. The
    return value is 0 if the callback as not been processed. */
typedef DEV_I2C_CTRL_FSM(stm32_i2c_fsm_callback_t);

static inline
bool_t stm32_i2c_check_error(struct dev_i2c_ctrl_transfer_s *tr,
                             uint32_t                       status)
{
  if (status & 0xff00)
    {
      if (status & STM32_I2C_SR1_TIMEOUT)
        tr->error = ETIMEDOUT;
      else if (status & STM32_I2C_SR1_AF)
        tr->error = EAGAIN;
      else if (status & STM32_I2C_SR1_BERR)
        tr->error = EIO;
      else if (status & STM32_I2C_SR1_ARLO)
        tr->error = EPERM;
      else
        tr->error = EUNKNOWN;
      return 1;
    }
  return 0;
}

static inline
DEV_I2C_CTRL_FSM(stm32_i2c_data_cb)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  /* check for state consistency. */
  if (!DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, ADDRESS))
    return 0;

  /* reset private state. */
  pv->nbytes = 0;

  /* handle direction. */
  switch (tr->dir)
    {
    default:
      assert(0 && "code non reachable.");
      break;

    case DEV_I2C_TR_WRITE:
      /* clear address sent interrupt. */
      (void) DEVICE_REG_VALUE_DEV(I2C, pv->addr, SR2);

      /* enable buffer interrupt. */
      DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITBUFEN);

      break;

    case DEV_I2C_TR_READ:
      /* 1-byte read. */
      if (tr->count == 1)
        {
          /* clear both ACK and POS (NACK on current byte). */
          DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, ACK);
          DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, POS);

          /* clear address sent interrupt. */
          (void) DEVICE_REG_VALUE_DEV(I2C, pv->addr, SR2);

          /* the STOP condition must be sent before check for data
             availability. */
          if (tr->op & DEV_I2C_OP_STOP)
            DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);

          break;
        }

      /* 2-byte read. */
      if (tr->count == 2)
        {
          /* clear ACK and set POS (NACK on next byte). */
          DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, ACK);
          DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, POS);

          /* clear address sent interrupt. */
          (void) DEVICE_REG_VALUE_DEV(I2C, pv->addr, SR2);

          break;
        }

      //if (tr->count > 2)
        {
          /* clear address sent interrupt. */
          (void) DEVICE_REG_VALUE_DEV(I2C, pv->addr, SR2);

          /* set ACK. */
          DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, ACK);
          DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, POS);

          /* enable buffer interrupt for data before the final sequence. */
          if (tr->count > 3)
            DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITBUFEN);

          break;
        }
    }

  return 1;
}


/****************************************** WRITE READY (EV8_1) */

static inline
DEV_I2C_CTRL_FSM(stm32_i2c_write_rdy_cb)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  /* check for state consistency. */
  if (!DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, TXE))
    return 0;

  /* disable buffer interrupt if only one byte. */
  if (tr->count == 1)
    DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);

  /* write the next byte to data register. */
  DEVICE_REG_FIELD_UPDATE_DEV(
    I2C,
    pv->addr,
    DR,
    DATA,
    tr->data[pv->nbytes++]
  );

  --tr->count;

  return 1;
}


/****************************************** WRITE N (EV_8) */

static inline
DEV_I2C_CTRL_FSM(stm32_i2c_write_n_cb)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  if (tr->count == 0)
    return 1;

  /* tr->count > 0 -> wait for TXE == 1. */
  if (!DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, TXE))
    return 0;

  /* disable buffer interrupt before writing the last byte. */
  if (tr->count == 1)
    DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);

  /* write the next byte to data register. */
  DEVICE_REG_FIELD_UPDATE_DEV(
    I2C,
    pv->addr,
    DR,
    DATA,
    tr->data[pv->nbytes++]
  );

  --tr->count;

  return 1;
}


/****************************************** READ 2 bytes */

static inline
DEV_I2C_CTRL_FSM(stm32_i2c_read_2_and_stop_cb)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  /* wait for RXNE == 1. */
  if (tr->count == 1 && !DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, RXNE))
    {
      DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITBUFEN);
      return 0;
    }

  /* wait for BTF == 1. */
  if (tr->count == 2 && !DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, BTF))
    return 0;

  /* transfer is for 1 or 2 bytes. */
  assert(tr->count < 3);

  /* clear buffer interrupt. */
  DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);

  /* send STOP condition before reading data (if count = 1, then the
     STOP condition is already sent, before checking BTF). */
  if (tr->count > 1)
    DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);

  /* read the two last data. */
  while (tr->count > 0)
    {
      tr->data[pv->nbytes++] = DEVICE_REG_FIELD_VALUE_DEV(
        I2C,
        pv->addr,
        DR,
        DATA
      );

      --tr->count;
    }

  return 1;
}


/****************************************** READ N bytes */

static inline
DEV_I2C_CTRL_FSM(stm32_i2c_read_n_cb)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  /* if it remains only three bytes, wait for BTF == 1. */
  if (tr->count == 3 && !DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, BTF))
    return 0;

  /* otherwise, wait for RXNE == 1. */
  else if (tr->count > 3 && !DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, RXNE))
    return 0;

  /* read the data. */
  if (tr->count == 3)
    /* clear ACK. */
    DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, ACK);

  /* clear buffer interrupt before writing the the N-4 byte as after only
     BTF is relevant. */
  if (tr->count == 4)
    DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITBUFEN);

  /* read the data. */
  tr->data[pv->nbytes++] = DEVICE_REG_FIELD_VALUE_DEV(
    I2C,
    pv->addr,
    DR,
    DATA
  );

  --tr->count;
  return 1;
}


static
bool_t stm32_i2c_fsm_exec(struct stm32_i2c_private_s *pv)
{
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  while (1)
    {
      /* compute next state. */
      switch (pv->state)
        {
        default:
          assert(0 && "invalid fsm state");
          break;

        /* *** IDLE (0) *** */
        case DEV_I2C_STM32_IDLE:
          if (tr->op & DEV_I2C_OP_START)
            pv->state = DEV_I2C_STM32_START;
          else if (tr->count > 0)
            pv->state = DEV_I2C_STM32_DATA;
          else if (tr->op & DEV_I2C_OP_STOP)
            pv->state = DEV_I2C_STM32_STOP;
          break;

        /* *** START (1) *** */
        case DEV_I2C_STM32_START:
          /* enable events. */
          DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITEVTEN);

          /* send START condition. */
          DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, START);

          pv->state = DEV_I2C_STM32_ADDR;
          break;

        /* *** ADDRESS (2) *** */
        case DEV_I2C_STM32_ADDR: {
          /* check that the START condition was sent. */
          if (!DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, SB))
            return 0;

          /* Send address byte */
          uint8_t const saddr = (tr->saddr << 1) | tr->dir;
          DEVICE_REG_FIELD_UPDATE_DEV(I2C, pv->addr, DR, DATA, saddr);

          if (tr->count > 0)
            pv->state = DEV_I2C_STM32_DATA;
          else 
            {
              if (tr->op & DEV_I2C_OP_STOP)
                pv->state = DEV_I2C_STM32_STOP;
              else
                pv->state = DEV_I2C_STM32_END;

              if (!DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, ADDRESS))
                return 0;
            }

          break;
        }

        /* *** DATA (3) *** */
        case DEV_I2C_STM32_DATA:
          if (!stm32_i2c_data_cb(pv))
            return 0;

          switch (tr->dir)
            {
            default:
              assert(0 && "unsupported transfer direction");
              break;

            case DEV_I2C_TR_WRITE:
              pv->state = DEV_I2C_STM32_WRITE_RDY;
              break;

            case DEV_I2C_TR_READ:
              if (tr->count < 3)
                pv->state = DEV_I2C_STM32_READ_2_AND_STOP;
              else
                pv->state = DEV_I2C_STM32_READ_N;

              break;
            }
          break;

        /* *** READY WRITING (4) *** */
        case DEV_I2C_STM32_WRITE_RDY:
          if (!stm32_i2c_write_rdy_cb(pv))
            return 0;

          pv->state = DEV_I2C_STM32_WRITE_N;
          break;

        /* *** WRITE N BYTES (5) *** */
        case DEV_I2C_STM32_WRITE_N:
          if (!stm32_i2c_write_n_cb(pv))
            return 0;

          /* end of writing. */
          if (tr->count == 0)
            {
              /* wait for BTF == 1. */
              if (!DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR1, BTF))
                return 0;

              if (tr->op & DEV_I2C_OP_STOP)
                pv->state = DEV_I2C_STM32_STOP;
              else
                pv->state = DEV_I2C_STM32_END;
            }
          break;

        /* *** READ 1 OR 2 BYTES (6) *** */
        case DEV_I2C_STM32_READ_2_AND_STOP:
          if (!(tr->op & DEV_I2C_OP_STOP))
            {
              tr->error = EINVAL;
              pv->state = DEV_I2C_STM32_ERROR;
              break;
            }

          if (!stm32_i2c_read_2_and_stop_cb(pv))
            return 0;

          pv->state = DEV_I2C_STM32_END;
          break;

        /* *** READ N BYTES (7) *** */
        case DEV_I2C_STM32_READ_N:
          if (!stm32_i2c_read_n_cb(pv))
            return 0;

          if (tr->count == 2)
            pv->state = DEV_I2C_STM32_READ_2_AND_STOP;
          break;

        /* *** STOP & ERROR (8) *** */
        case DEV_I2C_STM32_STOP:
          /* in case of a scan (no data and need to ack the address sent
             event. */
          (void) DEVICE_REG_VALUE_DEV(I2C, pv->addr, SR2);

        case DEV_I2C_STM32_ERROR:
          /* Send stop condition. */
          DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, STOP);
          pv->state = DEV_I2C_STM32_END;
          break;

        /* *** END OF TRANSMISSION (9) *** */
        case DEV_I2C_STM32_END:
          /* disable events. */
          DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR2, ITEVTEN);

          /* wait to return in slave mode. */
          /* FIXME: this is a hack that is not mentioned in the datasheet.
             However, unless this, the controller looks to produce unexpected
             interrupts even after the STOP condition is sent. */
          if (tr->op & DEV_I2C_OP_STOP)
            {
              while (DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR2, MSL));
            }

          pv->tr = NULL;
          return 1;
        }
    }
}


/***************************************** irq end-point process */

static DEV_IRQ_EP_PROCESS(stm32_i2c_irq)
{
  struct device_s                *dev = ep->dev;
  struct stm32_i2c_private_s     *pv  = dev->drv_pv;
  struct dev_i2c_ctrl_transfer_s *tr = pv->tr;

  lock_spin(&dev->lock);

  while (1)
    {
      /* get interurpt flags */
      uint32_t status = DEVICE_REG_VALUE_DEV(I2C, pv->addr, SR1);
      status &= STM32_I2C_IRQ_MASK;

      if (status == 0 || pv->tr == NULL)
        break;

      /* check for errors. */
      if (stm32_i2c_check_error(tr, status))
        {
          /* clear errors. */
          DEVICE_REG_UPDATE_DEV(I2C, pv->addr, SR1, 0);

          /* go to next state. */
          pv->state = DEV_I2C_STM32_ERROR;
        }

      /* compute protocol. */
      bool_t done = stm32_i2c_fsm_exec(pv);
      if (done)
        {
          lock_release(&dev->lock);
          kroutine_exec(&tr->kr, 0);
          return;
        }
    }
  lock_release(&dev->lock);
}

DEV_I2C_CTRL_TRANSFER(stm32_i2c_transfer)
{
  struct device_s            *dev = accessor->dev;
  struct stm32_i2c_private_s *pv  = dev->drv_pv;
  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->tr != NULL)
    tr->error = EBUSY;
#if 0
  else if (DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR2, BUSY))
    tr->error = EBUSY;
#endif
  else
    {
      tr->error = 0;
      pv->tr    = tr;
      pv->state = DEV_I2C_STM32_IDLE;

      done = stm32_i2c_fsm_exec(pv);
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    kroutine_exec(&tr->kr, cpu_is_interruptible());
}


#if defined(CONFIG_DEVICE_I2C_REQUEST)

/****************************************** scheduler */

static
DEV_I2C_CTRL_SCHED(stm32_i2c_sched)
{
  struct device_s            *dev = accessor->dev;
  struct stm32_i2c_private_s *pv  = dev->drv_pv;
  return &pv->sched;
}

#endif


static const struct driver_i2c_ctrl_s stm32_i2c_ctrl_cls =
{
  .class_     = DRIVER_CLASS_I2C_CTRL,
  .f_config   = &stm32_i2c_config,
  .f_transfer = &stm32_i2c_transfer,
#if defined(CONFIG_DEVICE_I2C_REQUEST)
  .f_sched    = &stm32_i2c_sched,
#endif
};

static DEV_INIT(stm32_i2c_init);
static DEV_CLEANUP(stm32_i2c_cleanup);

const struct driver_s stm32_i2c_ctrl_drv =
{
  .desc      = "STM32 I2C Master",
  .f_init    = &stm32_i2c_init,
  .f_cleanup = &stm32_i2c_cleanup,
  .classes   =
    {
      &stm32_i2c_ctrl_cls,
      0
    }
};

REGISTER_DRIVER(stm32_i2c_ctrl_drv);

static DEV_INIT(stm32_i2c_init)
{
  struct stm32_i2c_private_s *pv;
  uint16_t                   freq_in_mhz;

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

  /* if the bus is busy, fix blocked state using a soft reset. */
  if (DEVICE_REG_FIELD_VALUE_DEV(I2C, pv->addr, SR2, BUSY))
    {
      DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, SWRST);
      DEVICE_REG_FIELD_CLR_DEV(I2C, pv->addr, CR1, SWRST);
    }

  /* reset the device. */
  DEVICE_REG_UPDATE_DEV(I2C, pv->addr, CR1, 0);
  DEVICE_REG_UPDATE_DEV(I2C, pv->addr, CR2, 0);

  /* configure GPIO with pull-up on SCL/SDA lines. */
  if (device_iomux_setup(dev, ",scl ,sda", NULL, NULL, NULL))
    goto err_mem;

#if defined(CONFIG_DEVICE_I2C_REQUEST)
  if (dev_i2c_ctrl_sched_init(dev, &pv->sched))
    goto err_sched;
#endif

  /* configure input clock. */
  // XXX: will be fixed by clock tree. */
  switch (pv->addr)
  {
  default: assert(0 && "unknown I2C controller");

  case STM32_I2C1_ADDR:
    DEVICE_REG_FIELD_SET(RCC, , APB1ENR, I2C1EN);
    break;

  case STM32_I2C2_ADDR:
    DEVICE_REG_FIELD_SET(RCC, , APB1ENR, I2C2EN);
    break;

  case STM32_I2C3_ADDR:
    DEVICE_REG_FIELD_SET(RCC, , APB1ENR, I2C3EN);
    break;
  }

  device_irq_source_init(
    dev,
    pv->irq_ep,
    2,
    &stm32_i2c_irq,
    DEV_IRQ_SENSE_HIGH_LEVEL
  );

  if (device_irq_source_link(dev, pv->irq_ep, 2, -1))
    goto err_irq;

  /* enable error interrupts. */
  DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR2, ITERREN);

  /* initialize the input clock frequency. */
  extern uint32_t stm32f4xx_clock_freq_apb1;

  freq_in_mhz = stm32f4xx_clock_freq_apb1 / 1000000;
  DEVICE_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CR2, FREQ, freq_in_mhz);

  /* set standard mode. */
  DEVICE_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CCR, FS, SM);

  /* initialize the I2C bus speed (100kHz in standard mode). */
  DEVICE_REG_FIELD_UPDATE_DEV(I2C, pv->addr, CCR, CCR, 5 * freq_in_mhz);

  /* initialize the rise time (p. 491 of Reference manual). */
  DEVICE_REG_UPDATE_DEV(I2C, pv->addr, TRISE, freq_in_mhz + 1);

  /* enable I2C device. */
  DEVICE_REG_FIELD_SET_DEV(I2C, pv->addr, CR1, PE);

  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

err_irq:
#if defined(CONFIG_DEVICE_I2C_REQUEST)
  dev_i2c_ctrl_sched_destroy(&pv->sched);
#endif

#if defined(CONFIG_DEVICE_I2C_REQUEST)
err_sched:
#endif

err_mem:
  mem_free(pv);
  return -EINVAL;
}

static
DEV_CLEANUP(stm32_i2c_cleanup)
{
  struct stm32_i2c_private_s *pv;

  pv = dev->drv_pv;

  // XXX: Wait for the current transmission.

  /* disable I2C device. */
  DEVICE_REG_UPDATE_DEV(I2C, pv->addr, CR1, 0);
  DEVICE_REG_UPDATE_DEV(I2C, pv->addr, CR2, 0);
  DEVICE_REG_UPDATE_DEV(I2C, pv->addr, CCR, 0);

  device_irq_source_unlink(dev, pv->irq_ep, 2);

  /* deallocate private driver context. */
  mem_free(pv);
}


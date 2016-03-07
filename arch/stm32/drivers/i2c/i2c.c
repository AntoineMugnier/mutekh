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

#include <arch/stm32/memory_map.h>

#include <arch/stm32/i2c.h>
#include <arch/stm32/f4xx_rcc.h>


enum stm32_i2c_state_e
{
    STM32_I2C_S_IDLE,
    STM32_I2C_S_START,
    STM32_I2C_S_ADDR,
    STM32_I2C_S_PREPARE,
    STM32_I2C_S_READ_N_BYTES,
    STM32_I2C_S_READ_2_BYTES,
    STM32_I2C_S_READ_1_BYTE,
    STM32_I2C_S_WRITE_N_BYTES,
    STM32_I2C_S_END,
    STM32_I2C_S_ERROR,
    STM32_I2C_S_STOP,
    STM32_I2C_S_FINALIZE
};

struct stm32_i2c_private_s
{
    /* device address. */
    uintptr_t                addr;

    /* Fsm state */
    enum stm32_i2c_state_e   state;

    /* request queue. */
    dev_request_queue_root_t queue;

    /* the stop condition is already sent. */
    bool_t                   stopped:1;

    /* interrupt end-point (event and error). */
    struct dev_irq_src_s     irq_ep[2];

    struct dev_freq_s        busfreq;
};


/***************************************** config */

static DEV_I2C_CONFIG(stm32_i2c_config)
{
  return -ENOTSUP;
}


/***************************************** request */

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

// FSM callbacks.
#define STM32_I2C_FSM_CALLBACK(n) bool_t (n) (struct stm32_i2c_private_s *pv, \
                                              struct dev_i2c_rq_s        *rq) \
/**/

/** @This function defines a callback for the i2c controller FSM. The
    return value is 0 if the callback as not been processed. */
typedef STM32_I2C_FSM_CALLBACK(stm32_i2c_fsm_callback_t);

/*  I2C FSM:

    IDLE                : waiting for request
    START_AND_SEND_ADDR : start a new transfer and send the address of slave
    WAIT_FOR_ADDR_ACK   : wait for the address to be accepted by a slave
        - on ack: read or write
        - on nack: go to ERROR

    READ_N_BYTES        : read bytes
        - on RXEIE: read the next byte
    READ_2_BYTES        : read 2 bytes
        - on BTF: read the last 2 bytes
    READ_1_BYTE         : read 1 byte
        - on BTF: read the last byte

    WRITE_N_BYTES       : write n bytes
        - on TXEIE: send the next byte
        - on nack: go to error

    ERROR               : an error occured

    STOP                : Stop the transfer.
*/

static inline
void
stm32_i2c_enable_events(struct stm32_i2c_private_s *pv)
{
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ))); STM32_I2C_CR2_ITEVTEN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ), endian_le32(_reg) ); } while (0);
}

static inline
void
stm32_i2c_disable_events(struct stm32_i2c_private_s *pv)
{
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ))); STM32_I2C_CR2_ITEVTEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ), endian_le32(_reg) ); } while (0);
}

static inline
bool_t
stm32_i2c_is_last_same_way(struct dev_i2c_rq_s *rq)
{
    if (rq->error_transfer >= (rq->transfer_count-1))
        return 1;

    struct dev_i2c_transfer_s *tr   = &rq->transfer[rq->error_transfer];
    struct dev_i2c_transfer_s *next = &rq->transfer[rq->error_transfer+1];

    return tr->type != next->type;
}

static inline
uint16_t
stm32_i2c_bytes_left_same_way(struct dev_i2c_rq_s *rq)
{
    uint16_t pos                    = rq->error_transfer;
    struct dev_i2c_transfer_s *tr   = &rq->transfer[pos];
    struct dev_i2c_transfer_s *next = tr + 1;

    uint16_t size = 0;
    for ( ; pos < rq->transfer_count; ++pos)
    {
        size += tr->size;
        tr = next++;
    }

    return size;
}

static inline
struct dev_i2c_transfer_s *
stm32_i2c_next_transfer(struct dev_i2c_rq_s *rq)
{
    struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];
cont:
    /* find the first non-empty transfer. */
    if (++rq->error_transfer < rq->transfer_count)
    {
        /* reset the transfer size. */
        tr->size = rq->error_offset;

        /* go to next transfer. */
        rq->error_offset = 0;
        tr = &rq->transfer[rq->error_transfer];

        if (tr->size == 0)
            goto cont;
    }
    return rq->error_transfer == rq->transfer_count ? NULL : tr;
}

static inline
int
stm32_i2c_check_error(int state, struct dev_i2c_rq_s *rq, uint32_t status)
{
    if (!(status & 0xff00))
        return 0;

    if (status & STM32_I2C_SR1_TIMEOUT)
        return -ETIMEDOUT;

    if ((status & STM32_I2C_SR1_AF) && state == STM32_I2C_S_ADDR)
        return -EHOSTUNREACH;

    if (status & STM32_I2C_SR1_BERR)
        return -EIO;

    return -EIO;
}

static inline
STM32_I2C_FSM_CALLBACK(stm32_i2c_start)
{
    stm32_i2c_enable_events(pv);
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_START_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

    /* reset current transfered byte. */
    rq->error_offset = 0;

    return 1;
}

static inline
STM32_I2C_FSM_CALLBACK(stm32_i2c_address)
{
    struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];

    /* if the control of the bus is pending, then wait. */
    if (!STM32_I2C_SR1_SB_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
        return 0;

    /* send slave address. */
    uint8_t address = rq->saddr << 1;
    if (tr->type == DEV_I2C_READ)
      address |= 1;

    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_DR_ADDR) ))); STM32_I2C_DR_DATA_SET( (_reg), address ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_DR_ADDR) ), endian_le32(_reg) ); } while (0);

    return 1;
}

static inline
STM32_I2C_FSM_CALLBACK(stm32_i2c_read_n_bytes)
{
    struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];

    /* if it remains only 3 bytes to read, then set ack low. */
    if (tr->size <= 3)
    {
        /* look ahead for more data if any. */
        uint32_t left = stm32_i2c_bytes_left_same_way(rq);
        if (left == 3)
        {
            if (!STM32_I2C_SR1_BTF_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
                return 0;

            do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_ACK_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
        }
    }

    /* otherwise, process byte reading normally. */
    else if (!STM32_I2C_SR1_RXNE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
        return 0;

    /* read byte from register. */
    tr->data[rq->error_offset++] = STM32_I2C_DR_DATA_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_DR_ADDR) ))) )
                                ;

    --tr->size;

    return 1;
}

static inline
STM32_I2C_FSM_CALLBACK(stm32_i2c_read_2_bytes)
{
    struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];

    if (tr->size == 1 && !STM32_I2C_SR1_RXNE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
        return 0;

    if (tr->size > 1 && !STM32_I2C_SR1_BTF_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
        return 0;

    /* send stop condition. */
    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_STOP_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

    while (tr->size-- > 0)
    {
        tr->data[rq->error_offset++] = STM32_I2C_DR_DATA_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_DR_ADDR) ))) )
                                    ;

        if (tr->size == 0 && !stm32_i2c_is_last_same_way(rq))
        {
            tr = stm32_i2c_next_transfer(rq);
            assert(tr->size < 2);
        }
    }

    return 1;
}

static inline
STM32_I2C_FSM_CALLBACK(stm32_i2c_write_n_bytes)
{
    struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];

    if (!STM32_I2C_SR1_TXE_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
        return 0;

    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_DR_ADDR) ))); STM32_I2C_DR_DATA_SET( (_reg), tr->data[rq->error_offset++] ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_DR_ADDR) ), endian_le32(_reg) ); } while (0)
                                                              ;

    --tr->size;

    return 1;
}

static
bool_t
stm32_i2c_run(struct stm32_i2c_private_s *pv)
{
    struct dev_request_s *base = dev_request_queue_head(&pv->queue);
    struct dev_i2c_rq_s  *rq   = dev_i2c_rq_s_cast(base);

    struct dev_i2c_transfer_s *tr = &rq->transfer[rq->error_transfer];
    enum dev_i2c_way_e        way;

    while ( 1 )
    {
        switch (pv->state)
        {
        default:
            assert(!"undefined state.");
            break;

        case STM32_I2C_S_IDLE:
            rq->error          = 0;
            rq->error_transfer = 0;
            rq->error_offset   = 0;

            tr = &rq->transfer[0];

            pv->state = STM32_I2C_S_START;
            break;

        case STM32_I2C_S_START:
            if (!stm32_i2c_start(pv, rq))
                goto wait_event;

            stm32_i2c_enable_events(pv);
            pv->stopped = 0;

            pv->state = STM32_I2C_S_ADDR;
            break;

        case STM32_I2C_S_ADDR:
            if (!stm32_i2c_address(pv, rq))
                goto wait_event;

            pv->state = STM32_I2C_S_PREPARE;
            break;

        case STM32_I2C_S_PREPARE:
            /* check that the address is sent and accepted, otherwise wait. */
            if (!STM32_I2C_SR1_ADDRESS_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
                goto wait_event;

            /* check for next transfer. */
            while (tr != NULL && tr->size == 0)
                tr = stm32_i2c_next_transfer(rq);

            /* if no more transfer, then stop. */
            if (tr == NULL)
            {
                pv->state = STM32_I2C_S_STOP;
                break;
            }

            /* enable buffer events. */
            do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ))); STM32_I2C_CR2_ITBUFEN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ), endian_le32(_reg) ); } while (0);

            /* prepare read/write. */
            if (tr->type == DEV_I2C_READ)
            {
                switch (stm32_i2c_bytes_left_same_way(rq))
                {
                default:
                    /* ack next byte. */
                    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_ACK_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
                    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_POS_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
                    pv->state = STM32_I2C_S_READ_N_BYTES;
                    break;

                case 1:
                    /* nack next byte. */
                    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_ACK_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
                    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_POS_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
                    pv->state = STM32_I2C_S_READ_1_BYTE;
                    break;

                case 2:
                    /* nack next+1 byte. */
                    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_ACK_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
                    do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_POS_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
                    pv->state = STM32_I2C_S_READ_2_BYTES;
                    break;
                }
            }
            else
                pv->state = STM32_I2C_S_WRITE_N_BYTES;

            /* clear address flag. */
            (void) endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_I2C_SR2_ADDR) )));

            break;

        case STM32_I2C_S_READ_N_BYTES:
            if (!stm32_i2c_read_n_bytes(pv, rq))
                goto wait_event;

            if (tr->size == 2)
                pv->state = STM32_I2C_S_READ_2_BYTES;

            break;

        case STM32_I2C_S_READ_2_BYTES:
            if (!stm32_i2c_read_2_bytes(pv, rq))
                goto wait_event;

            /* already stopped. */
            pv->stopped = 1;

            /* be sure to keep the last processed tranfer. this is important
               in case the 2 bytes are in between two transfers. */
            tr = &rq->transfer[rq->error_transfer];

            pv->state = STM32_I2C_S_END;
            break;

        case STM32_I2C_S_READ_1_BYTE:
            if (!stm32_i2c_read_2_bytes(pv, rq))
                goto wait_event;

            /* already stopped. */
            pv->stopped = 1;

            pv->state = STM32_I2C_S_END;
            break;

        case STM32_I2C_S_WRITE_N_BYTES:
            if (!stm32_i2c_write_n_bytes(pv, rq))
                goto wait_event;

            if (tr->size == 0)
                pv->state = STM32_I2C_S_END;

            break;

        case STM32_I2C_S_END:
            /* reset the transfer size. */
            tr->size = rq->error_offset;

            /* save current type of transfer. */
            way = tr->type;

            /* disable buffer events. */
            do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ))); STM32_I2C_CR2_ITBUFEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ), endian_le32(_reg) ); } while (0);

            /* go next. */
            if ((tr = stm32_i2c_next_transfer(rq)) == NULL)
            {
                if (pv->stopped)
                    pv->state = STM32_I2C_S_FINALIZE;
                else
                    pv->state = STM32_I2C_S_STOP;
                break;
            }
            else if (pv->stopped)
            {
                rq->error = -ENOTSUP;
                pv->state = STM32_I2C_S_ERROR;
                break;
            }

            /* restart on way change. */
            if (tr->type != way)
            {
                do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ))); STM32_I2C_CR2_ITBUFEN_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ), endian_le32(_reg) ); } while (0);
                pv->state = STM32_I2C_S_START;
            }
            else
                pv->state = tr->type == DEV_I2C_READ ?
                    STM32_I2C_S_READ_N_BYTES : STM32_I2C_S_WRITE_N_BYTES;

            break;

        case STM32_I2C_S_ERROR:
            /* restore transfer size. */
            tr->size += rq->error_offset;

            pv->state = STM32_I2C_S_STOP;
            /* continue. */

        case STM32_I2C_S_STOP:
            /* if we have just written the last byte, wait for BTF=1. */
            tr = &rq->transfer[rq->error_transfer-1];
            if (rq->error_offset > 0 && tr->type == DEV_I2C_WRITE &&
                !STM32_I2C_SR1_BTF_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR1_ADDR) ))) ))
                return 0;

            do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_STOP_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

            pv->state = STM32_I2C_S_FINALIZE;
            /* continue. */

        case STM32_I2C_S_FINALIZE:
            stm32_i2c_disable_events(pv);

            /* active wait that the controller leaves the Master mode (i.e. MSL
               bit set). */
            while (STM32_I2C_SR2_MSL_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR2_ADDR) ))) ));

            pv->state = STM32_I2C_S_IDLE;
            goto transfer_done;
        }
    }

transfer_done:
    return 1;

wait_event:
    return 0;
}

/***************************************** irq end-point process */

static
DEV_IRQ_SRC_PROCESS(stm32_i2c_irq)
{
  struct device_s                *dev = ep->base.dev;
  struct stm32_i2c_private_s     *pv  = dev->drv_pv;

  lock_spin(&dev->lock);

  struct dev_request_s *base = dev_request_queue_head(&pv->queue);
  struct dev_i2c_rq_s  *rq   = dev_i2c_rq_s_cast(base);
  uint32_t err = 0;

  while (1)
    {
      /* get interurpt flags */
      uint32_t status = endian_le32(cpu_mem_read_32(( (((pv->addr))) + (STM32_I2C_SR1_ADDR) )));
      status &= STM32_I2C_IRQ_MASK;

      /* clear errors. */
      cpu_mem_write_32( ( (((pv->addr))) + (STM32_I2C_SR1_ADDR) ), endian_le32(0) );

      if (status == 0 || rq == NULL)
        break;

      /* check for errors. */
      if ((err = stm32_i2c_check_error(pv->state, rq, status)))
        {
          /* set request error. */
          rq->error = err;

          /* go to error state. */
          pv->state = STM32_I2C_S_ERROR;
        }

      /* compute protocol. */
      bool_t done = stm32_i2c_run(pv);
      if (done)
        {
          dev_request_queue_pop(&pv->queue);

          kroutine_exec(&base->kr);

          base = dev_request_queue_head(&pv->queue);
          rq   = dev_i2c_rq_s_cast(base);
        }
    }

  lock_release(&dev->lock);
}

static
DEV_I2C_REQUEST(stm32_i2c_request)
{
  struct device_s            *dev = accessor->dev;
  struct stm32_i2c_private_s *pv  = dev->drv_pv;

  assert(req->transfer_count != 0);

  bool_t done = 0;
  LOCK_SPIN_IRQ(&dev->lock);

  bool_t empty = dev_request_queue_isempty(&pv->queue);
  dev_request_queue_pushback(&pv->queue, &req->base);

  if (empty)
    done = stm32_i2c_run(pv);

  LOCK_RELEASE_IRQ(&dev->lock);

  /* if the request was pushed in an empty queue and the request already
     terminates, then execute the kroutine.
   */
  if (done)
    kroutine_exec(&req->base.kr);
}

static DEV_INIT(stm32_i2c_init);
static DEV_CLEANUP(stm32_i2c_cleanup);

#define stm32_i2c_use dev_use_generic

DRIVER_DECLARE(stm32_i2c_ctrl_drv, 0, "STM32 I2C Master", stm32_i2c,
               DRIVER_I2C_METHODS(stm32_i2c));

DRIVER_REGISTER(stm32_i2c_ctrl_drv);

static DEV_INIT(stm32_i2c_init)
{
  struct stm32_i2c_private_s *pv;
  uint16_t                   freq_in_mhz;


  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  /* retreive the device base address from device tree. */
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  if (device_get_res_freq(dev, &pv->busfreq, 0))
    goto err_mem;

  /* if the bus is busy, fix blocked state using a soft reset. */
  if (STM32_I2C_SR2_BUSY_GET( endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_SR2_ADDR) ))) ))
    {
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_SWRST_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
      do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_SWRST_SET( (_reg), 0 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);
    }

  /* reset the device. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_I2C_CR1_ADDR) ), endian_le32(0) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_I2C_CR2_ADDR) ), endian_le32(0) );

  /* configure GPIO with pull-up on SCL/SDA lines. */
  if (device_iomux_setup(dev, ",scl ,sda", NULL, NULL, NULL))
    goto err_mem;

  device_irq_source_init(dev, pv->irq_ep, 2, &stm32_i2c_irq);
  if (device_irq_source_link(dev, pv->irq_ep, 2, -1))
    goto err_mem;

  /* intialize the request queue. */
  dev_request_queue_init(&pv->queue);

  /* enable error interrupts. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ))); STM32_I2C_CR2_ITERREN_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ), endian_le32(_reg) ); } while (0);

  freq_in_mhz = (uint64_t)pv->busfreq.num / (1000000ULL * pv->busfreq.denom);
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ))); STM32_I2C_CR2_FREQ_SET( (_reg), freq_in_mhz ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR2_ADDR) ), endian_le32(_reg) ); } while (0);

  /* set standard mode. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CCR_ADDR) ))); STM32_I2C_CCR_FS_SET( (_reg), SM ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CCR_ADDR) ), endian_le32(_reg) ); } while (0);

  /* initialize the I2C bus speed (100kHz in standard mode). */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CCR_ADDR) ))); STM32_I2C_CCR_CCR_SET( (_reg), 5 * freq_in_mhz ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CCR_ADDR) ), endian_le32(_reg) ); } while (0);

  /* initialize the rise time (p. 491 of Reference manual). */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_I2C_TRISE_ADDR) ), endian_le32(freq_in_mhz + 1) );

  /* enable I2C device. */
  do { uint32_t register _reg = endian_le32(cpu_mem_read_32(( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ))); STM32_I2C_CR1_PE_SET( (_reg), 1 ); cpu_mem_write_32( ( ((((pv->addr)))) + (STM32_I2C_CR1_ADDR) ), endian_le32(_reg) ); } while (0);

  dev->drv    = &stm32_i2c_ctrl_drv;
  dev->drv_pv = pv;

  return 0;

err_mem:
  mem_free(pv);
  return -1;
}

static
DEV_CLEANUP(stm32_i2c_cleanup)
{
  struct stm32_i2c_private_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  /* disable I2C device. */
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_I2C_CR1_ADDR) ), endian_le32(0) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_I2C_CR2_ADDR) ), endian_le32(0) );
  cpu_mem_write_32( ( (((pv->addr))) + (STM32_I2C_CCR_ADDR) ), endian_le32(0) );

  /* clean up irqs. */
  device_irq_source_unlink(dev, pv->irq_ep, 2);

  /* destroy the request queue. */
  dev_request_queue_destroy(&pv->queue);

  /* deallocate private driver context. */
  mem_free(pv);

  return 0;
}


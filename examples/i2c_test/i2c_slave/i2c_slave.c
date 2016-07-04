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

    Copyright Vincent DEFILIPPI <vincentdefilippi@gmail.com> (c) 2016
*/

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/kroutine.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <device/class/gpio.h>
#include <device/class/timer.h>
#include <device/clock.h>

#include "i2c_slave.h"

enum i2c_slave_pin_mode_e
{
  I2C_SLAVE_PIN_MODE_NONE = 0,
  I2C_SLAVE_PIN_MODE_INPUT,
  I2C_SLAVE_PIN_MODE_OUTPUT,
};

enum i2c_slave_state_e
{
  I2C_SLAVE_STATE_IDLE = 0,
  I2C_SLAVE_STATE_START,
  I2C_SLAVE_STATE_RECV_ADDR,
  I2C_SLAVE_STATE_RECV_DATA,
  I2C_SLAVE_STATE_SEND_DATA,
  I2C_SLAVE_STATE_SEND_ACK,
  I2C_SLAVE_STATE_RECV_ACK
};

enum i2c_slave_mode_e
{
  I2C_SLAVE_MODE_WRITE = 0,
  I2C_SLAVE_MODE_READ,
  I2C_SLAVE_MODE_DEBUG,
};

enum i2c_slave_debug_state_e
{
  I2C_SLAVE_DEBUG_STATE_SIZE = 0,
  I2C_SLAVE_DEBUG_STATE_LAST_CHECKSUM,
  I2C_SLAVE_DEBUG_STATE_LOG,
  I2C_SLAVE_DEBUG_STATE_CHECKSUM,
};

struct i2c_slave_device_s
{
  uint8_t   addr;
  uint8_t   checksum[I2C_SLAVE_BUFFER_SIZE];
  uint8_t   log[I2C_SLAVE_BUFFER_SIZE];
  uint8_t   wptr;
  uint8_t   rptr;
  uint16_t  last_checksum;
  enum i2c_slave_debug_state_e debug_state;
};

struct i2c_slave_private_s
{
  struct device_gpio_s        dev_gpio;

  gpio_id_t                   clk_pin;
  gpio_id_t                   sda_pin;

  enum i2c_slave_pin_mode_e   clk_mode;
  enum i2c_slave_pin_mode_e   sda_mode;

  bool_t                      last_clk;
  bool_t                      last_sda;
  bool_t                      new_clk;
  bool_t                      new_sda;

  uint8_t                     bit_cnt;
  uint8_t                     shift_buffer;

  struct i2c_slave_device_s   *current_slave;
  enum i2c_slave_mode_e      current_mode;

  enum i2c_slave_state_e      state;

  uint8_t                     slaves_cnt;
  struct i2c_slave_device_s   slaves[0];
};

/*----------------------------------------------------------------------------*/

inline error_t
dev_gpio_set_mode(
    const struct device_gpio_s *accessor,
    gpio_id_t pin,
    enum dev_pin_driving_e mode)
{
  return DEVICE_OP(accessor, set_mode, pin, pin, dev_gpio_mask1, mode);
}

inline error_t
dev_gpio_set_pin(
    const struct device_gpio_s *accessor,
    gpio_id_t pin,
    bool_t val)
{
    if (val)
      return DEVICE_OP(accessor, set_output, pin, pin, dev_gpio_mask1, dev_gpio_mask1);
    return DEVICE_OP(accessor, set_output, pin, pin, dev_gpio_mask0, dev_gpio_mask0);
}

inline error_t
dev_gpio_get_pin(

    const struct device_gpio_s *accessor,
    gpio_id_t pin,
    bool_t *val)
{
    uint8_t data[8];
    error_t error = DEVICE_OP(accessor, get_input, pin, pin, data);
    if (!error)
      *val = data[0] & 1;
    return error;
}

/*----------------------------------------------------------------------------*/

static inline void
clk_as_input(struct i2c_slave_private_s *pv)
{
  if (pv->clk_mode == I2C_SLAVE_PIN_MODE_INPUT)
    return;

  if (dev_gpio_set_mode(&pv->dev_gpio, pv->clk_pin, DEV_PIN_INPUT))
    {
      printk("error while setting clk as input\n");
      abort();
    }
  pv->clk_mode = I2C_SLAVE_PIN_MODE_INPUT;
}

static inline void
sda_as_input(struct i2c_slave_private_s *pv)
{
  if (pv->sda_mode == I2C_SLAVE_PIN_MODE_INPUT)
    return;

  if (dev_gpio_set_mode(&pv->dev_gpio, pv->sda_pin, DEV_PIN_INPUT))
    {
      printk("error while setting sda as input\n");
      abort();
    }
  pv->sda_mode = I2C_SLAVE_PIN_MODE_INPUT;
}

static inline void
sda_as_output(struct i2c_slave_private_s *pv)
{
  if (pv->sda_mode == I2C_SLAVE_PIN_MODE_OUTPUT)
    return;

  if (dev_gpio_set_mode(&pv->dev_gpio, pv->sda_pin, DEV_PIN_OPENDRAIN))
    {
      printk("error while setting sda as output\n");
      abort();
    }
  pv->sda_mode = I2C_SLAVE_PIN_MODE_OUTPUT;
}

static inline bool_t
get_clk(struct i2c_slave_private_s *pv)
{
  bool_t val = 0;
  if (dev_gpio_get_pin(&pv->dev_gpio, pv->clk_pin, &val))
    {
      printk("error while getting clk value\n");
      abort();
    }
  return val;
}

static inline bool_t
get_sda(struct i2c_slave_private_s *pv)
{
  bool_t val = 0;
  if (dev_gpio_get_pin(&pv->dev_gpio, pv->sda_pin, &val))
    {
      printk("error while getting sda value\n");
      abort();
    }
  return val;
}

static inline void
set_sda(struct i2c_slave_private_s *pv, bool_t val)
{
  if (dev_gpio_set_pin(&pv->dev_gpio, pv->sda_pin, val))
    {
      printk("error while setting sda value\n");
      abort();
    }
}

/*----------------------------------------------------------------------------*/

static inline bool_t
clk_rise(struct i2c_slave_private_s *pv)
{
  return ((!pv->last_clk) && pv->new_clk);
}

static inline bool_t
clk_fall(struct i2c_slave_private_s *pv)
{
  return (pv->last_clk && (!pv->new_clk));
}

static inline bool_t
sda_rise(struct i2c_slave_private_s *pv)
{
  return ((!pv->last_sda) && pv->new_sda);
}

static inline bool_t
sda_fall(struct i2c_slave_private_s *pv)
{
  return (pv->last_sda && (!pv->new_sda));
}

/*----------------------------------------------------------------------------*/

static inline void
reset_slave(struct i2c_slave_private_s *pv)
{
  assert(pv->current_slave);
  struct i2c_slave_device_s *slave = pv->current_slave;

  memset(slave->log, 0, I2C_SLAVE_BUFFER_SIZE);
  memset(slave->checksum, 0, I2C_SLAVE_BUFFER_SIZE);
  slave->wptr = 0;
  slave->rptr = 0;
  slave->last_checksum = 0;
  slave->debug_state = I2C_SLAVE_DEBUG_STATE_SIZE;
}

static inline void
update_slave(struct i2c_slave_private_s *pv, uint8_t value)
{
  assert(pv->current_slave);
  struct i2c_slave_device_s *slave = pv->current_slave;

  if (pv->current_mode == I2C_SLAVE_MODE_DEBUG)
    return;

  slave->last_checksum = (slave->last_checksum << 1) | (slave->last_checksum >> 15);
  slave->last_checksum ^= value;
  slave->checksum[slave->wptr] = (uint8_t)(slave->last_checksum & 0xff);

  slave->log[slave->wptr] = value;
  slave->wptr++;
  if (slave->wptr == I2C_SLAVE_BUFFER_SIZE)
    slave->wptr = 0;
}

/*----------------------------------------------------------------------------*/

static inline bool_t
select_slave(struct i2c_slave_private_s *pv, uint8_t value)
{
  enum i2c_slave_mode_e mode = I2C_SLAVE_MODE_WRITE;
  if (value & 1)
    mode = I2C_SLAVE_MODE_READ;

  uint8_t addr = value >> 1;
  if (addr & I2C_SLAVE_DEBUG_MODE_MASK)
    {
      addr &= ~I2C_SLAVE_DEBUG_MODE_MASK;
      if (mode == I2C_SLAVE_MODE_WRITE)
        return 0;
      mode = I2C_SLAVE_MODE_DEBUG;
    }

  for (uint8_t i = 0; i < pv->slaves_cnt; i++)
    {
      if (pv->slaves[i].addr == addr)
        {
          pv->current_slave = &pv->slaves[i];
          pv->current_mode = mode;
          return 1;
        }
    }
  pv->current_slave = NULL;
  return 0;
}

static inline void
go_to_idle(struct i2c_slave_private_s *pv)
{
  sda_as_input(pv);
  pv->state = I2C_SLAVE_STATE_IDLE;
}

static inline void
go_to_start(struct i2c_slave_private_s *pv)
{
  sda_as_input(pv);
  pv->state = I2C_SLAVE_STATE_START;
}

static inline void
go_to_recv_addr(struct i2c_slave_private_s *pv)
{
  sda_as_input(pv);
  pv->bit_cnt = 0;
  pv->shift_buffer = 0;
  pv->state = I2C_SLAVE_STATE_RECV_ADDR;
}

static inline void
go_to_recv_data(struct i2c_slave_private_s *pv)
{
  sda_as_input(pv);
  pv->bit_cnt = 0;
  pv->shift_buffer = 0;
  pv->state = I2C_SLAVE_STATE_RECV_DATA;
}

static inline void
go_to_send_data(struct i2c_slave_private_s *pv)
{
  assert(pv->current_slave);
  struct i2c_slave_device_s *slave = pv->current_slave;

  pv->bit_cnt = 0;


  if (pv->current_mode == I2C_SLAVE_MODE_DEBUG)
    {
      switch (slave->debug_state)
        {
          case I2C_SLAVE_DEBUG_STATE_SIZE:
            pv->shift_buffer = slave->wptr;
            slave->debug_state = I2C_SLAVE_DEBUG_STATE_LAST_CHECKSUM;
            break;

          case I2C_SLAVE_DEBUG_STATE_LAST_CHECKSUM:
            pv->shift_buffer = slave->last_checksum;
            slave->debug_state = I2C_SLAVE_DEBUG_STATE_LOG;
            break;

          case I2C_SLAVE_DEBUG_STATE_LOG:
            pv->shift_buffer = slave->log[slave->rptr];
            slave->rptr++;
            if (slave->rptr > I2C_SLAVE_BUFFER_SIZE)
              {
                slave->rptr = 0;
                slave->debug_state = I2C_SLAVE_DEBUG_STATE_CHECKSUM;
              }
            break;

          case I2C_SLAVE_DEBUG_STATE_CHECKSUM:
            pv->shift_buffer = slave->checksum[slave->rptr];
            slave->rptr++;
            if (slave->rptr > I2C_SLAVE_BUFFER_SIZE)
              {
                slave->rptr = 0;
                slave->debug_state = I2C_SLAVE_DEBUG_STATE_LOG;
              }
            break;
        }
    }
  else
    {
      pv->shift_buffer = slave->last_checksum;
      update_slave(pv, I2C_SLAVE_BASIC_OP_SEND_DATA);
      update_slave(pv, pv->shift_buffer);
    }


  sda_as_output(pv);
  set_sda(pv, (pv->shift_buffer >> 7) & 1);
  pv->state = I2C_SLAVE_STATE_SEND_DATA;
}

static inline void
go_to_send_ack(struct i2c_slave_private_s *pv)
{
  sda_as_output(pv);
  set_sda(pv, 0);
  pv->state = I2C_SLAVE_STATE_SEND_ACK;
}

static inline void
go_to_recv_ack(struct i2c_slave_private_s *pv)
{
  sda_as_input(pv);
  pv->state = I2C_SLAVE_STATE_RECV_ACK;
}

static inline void
step(struct i2c_slave_private_s *pv)
{
  if ((pv->sda_mode == I2C_SLAVE_PIN_MODE_INPUT) && pv->new_clk)
    {
      if (sda_fall(pv))
        return go_to_start(pv);
      else if (sda_rise(pv))
        {
          if (pv->current_slave)
            {
              if (pv->current_mode == I2C_SLAVE_MODE_DEBUG)
                reset_slave(pv);
              update_slave(pv, I2C_SLAVE_BASIC_OP_STOP);
            }
          pv->current_slave = NULL;
          return go_to_idle(pv);
        }
    }

  switch (pv->state)
    {
      case I2C_SLAVE_STATE_IDLE:
        return;

      case I2C_SLAVE_STATE_START:
        if (clk_fall(pv))
          go_to_recv_addr(pv);
        return;

      case I2C_SLAVE_STATE_RECV_ADDR:
        if (clk_fall(pv))
          {
            if (pv->bit_cnt < 7)
              pv->bit_cnt++;
            else
              {
                if (select_slave(pv, pv->shift_buffer))
                  {
                    update_slave(pv, I2C_SLAVE_BASIC_OP_START);
                    update_slave(pv, I2C_SLAVE_BASIC_OP_RECV_ADDR);
                    update_slave(pv, pv->shift_buffer);
                    update_slave(pv, I2C_SLAVE_BASIC_OP_SEND_ACK);
                    go_to_send_ack(pv);
                  }
                else
                  go_to_idle(pv);
              }
          }
        else if (clk_rise(pv))
          {
            if (pv->new_sda)
              pv->shift_buffer |= (1 << (7 - pv->bit_cnt));
          }
        return;

      case I2C_SLAVE_STATE_RECV_DATA:
        assert(pv->current_slave);
        if (clk_fall(pv))
          {
            if (pv->bit_cnt < 7)
              pv->bit_cnt++;
            else
              {
                update_slave(pv, I2C_SLAVE_BASIC_OP_RECV_DATA);
                update_slave(pv, pv->shift_buffer);
                if (pv->shift_buffer != I2C_SLAVE_INVALID_BYTE)
                  {
                    update_slave(pv, I2C_SLAVE_BASIC_OP_SEND_ACK);
                    go_to_send_ack(pv);
                  }
                else
                  {
                    update_slave(pv, I2C_SLAVE_BASIC_OP_SEND_NACK);
                    go_to_idle(pv);
                  }
              }
          }
        else if (clk_rise(pv))
          {
            if (pv->new_sda)
              pv->shift_buffer |= (1 << (7 - pv->bit_cnt));
          }
        return;

      case I2C_SLAVE_STATE_SEND_DATA:
        assert(pv->current_slave);
        if (clk_fall(pv))
          {
            if (pv->bit_cnt < 7)
              {
                pv->bit_cnt++;
                set_sda(pv, (pv->shift_buffer >> (7 - pv->bit_cnt)) & 1);
              }
            else
              go_to_recv_ack(pv);
          }
        return;

      case I2C_SLAVE_STATE_SEND_ACK:
        assert(pv->current_slave);
        if (clk_fall(pv))
          {
            if (pv->current_mode == I2C_SLAVE_MODE_READ ||
                pv->current_mode == I2C_SLAVE_MODE_DEBUG)
              go_to_send_data(pv);
            else
              go_to_recv_data(pv);
          }
        return;

      case I2C_SLAVE_STATE_RECV_ACK:
        assert(pv->current_slave);
        if (clk_fall(pv))
          {
            update_slave(pv, I2C_SLAVE_BASIC_OP_RECV_ACK);
            go_to_send_data(pv);
          }
        else if (clk_rise(pv))
          {
            if (get_sda(pv))
              {
                update_slave(pv, I2C_SLAVE_BASIC_OP_RECV_NACK);
                go_to_idle(pv);
              }
          }
        return;
    }
}

static inline void
init_slaves(struct i2c_slave_private_s *pv,
            const uint8_t *slaves_addr, uint32_t slaves_cnt)
{
  assert(slaves_cnt > 0);
  assert(slaves_cnt < 64);

  pv->slaves_cnt = (uint8_t)slaves_cnt;
  for (uint8_t i = 0; i < slaves_cnt; i++)
    {
      if (slaves_addr[i] >= I2C_SLAVE_DEBUG_MODE_MASK ||
          slaves_addr[i] == I2C_SLAVE_INVALID_ADDR)
        {
          printk("init slave: invalid address [0x%x]\n", slaves_addr[i]);
          abort();
        }
      pv->slaves[i].addr = slaves_addr[i];
      pv->current_slave = &pv->slaves[i];
      reset_slave(pv);
    }
    pv->current_slave = NULL;
}

void
app_start(void)
{
  printk(">>\n");
  printk("i2c slave\n");

  /* Check that the buffer size could be contained in an uint8_t */
  assert(I2C_SLAVE_BUFFER_SIZE <= 255);

  /* Must be between 0x00 and 0x3f and different of 0x42 */
  static const uint8_t slaves_addr[] =
  {
    0x00,
    0x07,
    0x11,
    0x1c,
    0x25,
    0x2a,
    0x30,
    0x3f,
  };

  uint32_t slaves_cnt = sizeof(slaves_addr) / sizeof(slaves_addr[0]);
  uint32_t slaves_size = sizeof(struct i2c_slave_device_s) * slaves_cnt;
  uint32_t pv_size = sizeof(struct i2c_slave_private_s) + slaves_size;

  struct i2c_slave_private_s *pv = mem_alloc(pv_size, (mem_scope_sys));
  if (!pv)
    {
      printk("error cannot allocate memory\n");
      abort();
    }
  memset(pv, 0, pv_size);

  if (device_get_accessor_by_path(&pv->dev_gpio.base, NULL, "gpio", DRIVER_CLASS_GPIO))
    {
      printk("error cannot get [ gpio ] accessor\n");
      abort();
    }

  init_slaves(pv, slaves_addr, slaves_cnt);

  pv->state = I2C_SLAVE_STATE_IDLE;

  pv->clk_pin = SCL_PIN;
  pv->sda_pin = SDA_PIN;

  clk_as_input(pv);
  sda_as_input(pv);

  pv->last_clk = get_clk(pv);
  pv->last_sda = get_sda(pv);
  while (1)
    {
      pv->new_clk = get_clk(pv);
      pv->new_sda = get_sda(pv);

      step(pv);

      pv->last_clk = pv->new_clk;
      pv->last_sda = pv->new_sda;
    }
}


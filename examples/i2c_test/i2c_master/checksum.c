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

#include "i2c_master.h"

static inline uint8_t
calcul_checksum(uint8_t *seq, uint8_t len)
{
  uint16_t checksum = 0;
  bool_t flag = 0;

  for (uint8_t i = 0; i < len; i++)
    {
      if (!flag && seq[i] == I2C_SLAVE_BASIC_OP_SEND_DATA)
      {
        flag = 1;
        seq[i + 1] = checksum & 0xff;
      }
      else
        flag = 0;

      checksum = (checksum << 1) | (checksum >> 15);
      checksum ^= seq[i];

      //printk("%d 0x%02x  [ 0x%02X ]\n", i, seq[i], checksum & 0xff);
    }
  return checksum & 0xff;
}

uint8_t
get_test_bc_1_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_bc_2_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_bc_3_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_bc_4_checksum(uint8_t slave_addr)
{
  uint8_t seq[] =
  {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint16_t checksum = 0;
  uint8_t len = sizeof(seq) / sizeof(seq[0]);

  uint8_t data[8];

  bool_t flag = 0;
  uint8_t i_send = 0;
  uint8_t i_recv = 0;

  for (uint8_t i = 0; i < len; i++)
    {
      if (!flag)
        {
          if (seq[i] == I2C_SLAVE_BASIC_OP_SEND_DATA)
            {
              flag = 1;
              data[i_send++] = checksum & 0xff;
              seq[i + 1] = checksum & 0xff;
              assert(i_send < 9);
            }
          else if (seq[i] == I2C_SLAVE_BASIC_OP_RECV_DATA)
            {
              flag = 1;
              seq[i + 1] = data[i_recv++];
              assert(i_recv < 9);
            }
        }
      else
        flag = 0;

      checksum = (checksum << 1) | (checksum >> 15);
      checksum ^= seq[i];

      //printk("%d 0x%02x  [ 0x%02X ]\n", i, seq[i], checksum & 0xff);
    }
  return checksum & 0xff;
}

uint8_t
get_test_bc_5_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    I2C_SLAVE_INVALID_BYTE,
    I2C_SLAVE_BASIC_OP_SEND_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_bc_7_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_bc_8_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    I2C_SLAVE_INVALID_BYTE,
    I2C_SLAVE_BASIC_OP_SEND_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_bc_9_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_tr_1_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_tr_2_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_tr_3_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x06,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x07,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_tr_4_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    (slave_addr << 1) | 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_ACK,
    I2C_SLAVE_BASIC_OP_SEND_DATA,
    0x00, /* last checksum (slave dependant) */
    I2C_SLAVE_BASIC_OP_RECV_NACK,
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00, /* data received previously (slave dependant) */
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint16_t checksum = 0;
  uint8_t len = sizeof(seq) / sizeof(seq[0]);

  uint8_t data[8];

  bool_t flag = 0;
  uint8_t i_send = 0;
  uint8_t i_recv = 0;

  for (uint8_t i = 0; i < len; i++)
    {
      if (!flag)
        {
          if (seq[i] == I2C_SLAVE_BASIC_OP_SEND_DATA)
            {
              flag = 1;
              data[i_send++] = checksum & 0xff;
              seq[i + 1] = checksum & 0xff;
              assert(i_send < 9);
            }
          else if (seq[i] == I2C_SLAVE_BASIC_OP_RECV_DATA)
            {
              flag = 1;
              seq[i + 1] = data[i_recv++];
              assert(i_recv < 9);
            }
        }
      else
        flag = 0;

      checksum = (checksum << 1) | (checksum >> 15);
      checksum ^= seq[i];

      //printk("%d 0x%02x  [ 0x%02X ]\n", i, seq[i], checksum & 0xff);
    }
  return checksum & 0xff;
}

uint8_t
get_test_tr_6_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x42,
    I2C_SLAVE_BASIC_OP_SEND_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}

uint8_t
get_test_tr_7_checksum(uint8_t slave_addr)
{
  uint8_t seq[] = {
    I2C_SLAVE_BASIC_OP_START,
    I2C_SLAVE_BASIC_OP_RECV_ADDR,
    slave_addr << 1,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x00,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x01,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x02,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x03,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x04,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x05,
    I2C_SLAVE_BASIC_OP_SEND_ACK,
    I2C_SLAVE_BASIC_OP_RECV_DATA,
    0x42,
    I2C_SLAVE_BASIC_OP_SEND_NACK,
    I2C_SLAVE_BASIC_OP_STOP,
  };

  uint8_t len = sizeof(seq) / sizeof(seq[0]);
  return calcul_checksum(seq, len);
}




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

#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__


#define I2C_MASTER_I2C_DEV        "i2c*"
#define I2C_MASTER_MAX_THREADS    3  /* without the main thread */

#define I2C_MASTER_READ_RESTART_SUPPORT 0  /* controller read restart support */

/*
-----------------------------------------------------------------
|   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
-----------------------------------------------------------------
| DEBUG |                                               |       |
|                     SLAVE ADDRESS                     | R / W |
-----------------------------------------------------------------
*/

#define I2C_SLAVE_BUFFER_SIZE     255

#define I2C_SLAVE_DEBUG_MODE_MASK 0x40
#define I2C_SLAVE_INVALID_ADDR    0x21
#define I2C_SLAVE_INVALID_BYTE    0x42

enum i2c_slave_basic_op_e
{
  I2C_SLAVE_BASIC_OP_START      = 0xf0,
  I2C_SLAVE_BASIC_OP_STOP       = 0xf1,
  I2C_SLAVE_BASIC_OP_RECV_ADDR  = 0xf2,
  I2C_SLAVE_BASIC_OP_RECV_DATA  = 0xf3,
  I2C_SLAVE_BASIC_OP_SEND_DATA  = 0xf4,
  I2C_SLAVE_BASIC_OP_RECV_ACK   = 0xf5,
  I2C_SLAVE_BASIC_OP_RECV_NACK  = 0xf6,
  I2C_SLAVE_BASIC_OP_SEND_ACK   = 0xf7,
  I2C_SLAVE_BASIC_OP_SEND_NACK  = 0xf8,
};

void display_memory(void);

uint8_t get_test_bc_1_checksum(uint8_t slave_addr);
uint8_t get_test_bc_2_checksum(uint8_t slave_addr);
uint8_t get_test_bc_3_checksum(uint8_t slave_addr);
uint8_t get_test_bc_4_checksum(uint8_t slave_addr);
uint8_t get_test_bc_5_checksum(uint8_t slave_addr);
uint8_t get_test_bc_7_checksum(uint8_t slave_addr);
uint8_t get_test_bc_8_checksum(uint8_t slave_addr);
uint8_t get_test_bc_9_checksum(uint8_t slave_addr);

uint8_t get_test_tr_1_checksum(uint8_t slave_addr);
uint8_t get_test_tr_2_checksum(uint8_t slave_addr);
uint8_t get_test_tr_3_checksum(uint8_t slave_addr);
uint8_t get_test_tr_4_checksum(uint8_t slave_addr);
uint8_t get_test_tr_6_checksum(uint8_t slave_addr);
uint8_t get_test_tr_7_checksum(uint8_t slave_addr);

#endif


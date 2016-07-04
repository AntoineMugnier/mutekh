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

#include <pthread.h>

#include <mutek/mem_alloc.h>

#include <mutek/printk.h>
#include <mutek/kroutine.h>
#include <mutek/semaphore.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>


#include <device/class/i2c.h>
#include <device/class/timer.h>
#include <device/class/cmu.h>
#include <device/clock.h>

#include "i2c_master.h"

#define I2C_MASTER_DISPLAY_DEBUG  0

struct i2c_master_debug_s
{
  uint8_t   size;
  uint8_t   last_checksum;
  uint8_t   log[I2C_SLAVE_BUFFER_SIZE];
  uint8_t   checksum[I2C_SLAVE_BUFFER_SIZE];
};

struct i2c_master_test_ctx_s
{
  struct device_i2c_ctrl_s              i2c_ctrl;
  struct dev_i2c_ctrl_bytecode_rq_s     i2c_bc_rq;
  struct dev_i2c_ctrl_transaction_rq_s  i2c_tr_rq;
  struct i2c_master_debug_s             debug_info;
  uint8_t                               buffer[8];
};

struct i2c_master_slaves_s
{
  uint8_t             addr[64];
  uint8_t             count;
  uint8_t             current;
  pthread_mutex_t     lock;
  struct semaphore_s  sem;
};

typedef error_t i2c_master_test_t(struct i2c_master_test_ctx_s *);

#define TEST_BAD_CHECKSUM 1
#define TEST_FAILED 2

#if I2C_MASTER_DISPLAY_DEBUG
static void
i2c_master_display_debug_info(struct i2c_master_test_ctx_s *ctx)
{
  static const char * const i2c_slave_basic_op_str[] =
  {
    "START     ",
    "STOP      ",
    "RECV_ADDR ",
    "RECV_DATA ",
    "SEND_DATA ",
    "RECV_ACK  ",
    "RECV_NACK ",
    "SEND_ACK  ",
    "SEND_NACK "
  };

  printk("\n");
  bool_t data = 0;
  for (uint8_t i = 0; i < ctx->debug_info.size; i++)
    {
      printk("%3d: ", i);

      /* Display log */
      uint8_t log = ctx->debug_info.log[i];
      printk("0x%02x ", log);
      if (!data && log >= I2C_SLAVE_BASIC_OP_START && log <= I2C_SLAVE_BASIC_OP_SEND_NACK)
        printk("%s ", i2c_slave_basic_op_str[log - I2C_SLAVE_BASIC_OP_START]);
      else
        printk("0x%02x       ", log);

      if (!data && (log == I2C_SLAVE_BASIC_OP_RECV_ADDR ||
                    log == I2C_SLAVE_BASIC_OP_RECV_DATA ||
                    log == I2C_SLAVE_BASIC_OP_SEND_DATA))
       {
          data = 1;
       }
       else
        data = 0;

      /* Display checksum */
      printk("0x%02x\n", ctx->debug_info.checksum[i]);
    }
}
#endif

static void
i2c_master_init_buffer(struct i2c_master_test_ctx_s *ctx)
{
  for (uint8_t i = 0; i < 8; i++)
    ctx->buffer[i] = i;
}

static void
i2c_master_init_mem(struct i2c_master_test_ctx_s *ctx)
{
  i2c_master_init_buffer(ctx);

  bc_set_reg(&ctx->i2c_bc_rq.vm, 0, (bc_reg_t)ctx->buffer);
  bc_set_reg(&ctx->i2c_bc_rq.vm, 1, 8);
}



/*------------------------------------------------------------------------------
  TEST BC 1
  Write CONTINUOUS
  Write RESTART
  Write STOP                                                                  */

static error_t
i2c_master_test_bc_1(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_1_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_1_reg,
                        0xff, 0, 1, 2, 3, 4, 5, 6, 7);

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_1_mem, 0);
  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 2
  Read CONTINUOUS
  Read RESTART
  Read STOP                                                                   */

static error_t
i2c_master_test_bc_2(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_2_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_2_reg,
                        0xff, 0, 1, 2, 3, 4, 5, 6, 7);

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_2_mem, 0);
  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 3
  Write RESTART
  Read  STOP                                                                  */

static error_t
i2c_master_test_bc_3(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_3_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_3_reg,
                        0xff, 0, 1, 2, 3, 4, 5, 6, 7);

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_3_mem, 0);
  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 4
  Read  RESTART
  Write STOP                                                                  */

static error_t
i2c_master_test_bc_4(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_4_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_4_reg,
                        0xff, 0, 1, 2, 3, 4, 5, 6, 7);

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_4_mem, 0);
  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 5
  Write STOP (with byte 3 invalid)                                            */

static error_t
i2c_master_test_bc_5(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_5_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_5_reg,
                        0xff, 0, 1, 2, I2C_SLAVE_INVALID_BYTE, 4, 5, 6, 7);

  if (ctx->i2c_bc_rq.base.err != -EAGAIN)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  ctx->buffer[3] = I2C_SLAVE_INVALID_BYTE;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_5_mem, 0);
  if (ctx->i2c_bc_rq.base.err != -EAGAIN)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 6
  Read STOP (with address invalid)                                            */

static error_t
i2c_master_test_bc_6(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  ctx->i2c_bc_rq.base.saddr = I2C_SLAVE_INVALID_ADDR;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_6_reg,
                        0xff, 0, 1, 2, 3, 4, 5, 6, 7);

  ctx->i2c_bc_rq.base.saddr = slave_addr;

  if (ctx->i2c_bc_rq.base.err != -EHOSTUNREACH)
    return TEST_FAILED;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  ctx->i2c_bc_rq.base.saddr = I2C_SLAVE_INVALID_ADDR;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_6_mem, 0);

  ctx->i2c_bc_rq.base.saddr = slave_addr;

  if (ctx->i2c_bc_rq.base.err != -EHOSTUNREACH)
    return TEST_FAILED;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 7
  Write conditionnal STOP (valid)
  Set REG 8 to 0x0 (should be skipped)                                        */

static error_t
i2c_master_test_bc_7(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_7_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_7_reg,
                        0x1ff, 0, 1, 2, 3, 4, 5, 6, 7, 8);

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;
  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 1)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  bc_set_reg(&ctx->i2c_bc_rq.vm, 8, 8);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_7_mem, 0);
  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;
  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 1)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 8
  Write conditionnal STOP (with byte 3 invalid)
  Set REG 8 to 0x0 (should be executed)                                       */

static error_t
i2c_master_test_bc_8(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_8_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_8_reg,
                        0x1ff, 0, 1, 2, I2C_SLAVE_INVALID_BYTE, 4, 5, 6, 7, 8);

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;
  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 0)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  ctx->buffer[3] = I2C_SLAVE_INVALID_BYTE;
  bc_set_reg(&ctx->i2c_bc_rq.vm, 8, 8);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_8_mem, 0);
  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;
  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 0)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 9
  Read conditionnal STOP (valid)
  Set REG 8 to 0x0 (should be skipped)                                        */

static error_t
i2c_master_test_bc_9(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;
  uint8_t checksum = get_test_bc_9_checksum(slave_addr);

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_9_reg,
                        0x1ff, 0, 1, 2, 3, 4, 5, 6, 7, 8);

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;
  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 1)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  bc_set_reg(&ctx->i2c_bc_rq.vm, 8, 8);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_9_mem, 0);
  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;
  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 1)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST BC 10
  Read conditionnal STOP (with address invalid)
  Set REG 8 to 0x0 (should be executed)                                       */

static error_t
i2c_master_test_bc_10(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_bc_rq.base.saddr;

  /* reg */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);

  ctx->i2c_bc_rq.base.saddr = I2C_SLAVE_INVALID_ADDR;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_10_reg,
                        0x1ff, 0, 1, 2, 3, 4, 5, 6, 7, 8);

  ctx->i2c_bc_rq.base.saddr = slave_addr;

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 0)
    return TEST_FAILED;

  /* mem */
  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_mem(ctx);

  bc_set_reg(&ctx->i2c_bc_rq.vm, 8, 8);

  ctx->i2c_bc_rq.base.saddr = I2C_SLAVE_INVALID_ADDR;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_test_10_mem, 0);
  ctx->i2c_bc_rq.base.saddr = slave_addr;

  if (ctx->i2c_bc_rq.base.err)
    return ctx->i2c_bc_rq.base.err;

  if ((bc_get_reg(&ctx->i2c_bc_rq.vm, 8) & 0xff) != 0)
    return TEST_FAILED;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST TR 1
  Write
  Write                                                                       */

static error_t
i2c_master_test_tr_1(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_tr_rq.base.saddr;
  uint8_t checksum = get_test_tr_1_checksum(slave_addr);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_buffer(ctx);

  struct dev_i2c_ctrl_transaction_data_s tr[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
  };

  ctx->i2c_tr_rq.transfer = tr;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr) / sizeof(tr[0]);

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err)
    return ctx->i2c_tr_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST TR 2
  Read
  Read                                                                        */

static error_t
i2c_master_test_tr_2(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_tr_rq.base.saddr;
  uint8_t checksum = get_test_tr_2_checksum(slave_addr);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_buffer(ctx);

  struct dev_i2c_ctrl_transaction_data_s tr[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_READ,
    },
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_READ,
    },
  };

  ctx->i2c_tr_rq.transfer = tr;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr) / sizeof(tr[0]);

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err)
    return ctx->i2c_tr_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST TR 3
  Write
  Read                                                                        */

static error_t
i2c_master_test_tr_3(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_tr_rq.base.saddr;
  uint8_t checksum = get_test_tr_3_checksum(slave_addr);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_buffer(ctx);

  struct dev_i2c_ctrl_transaction_data_s tr[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_READ,
    },
  };

  ctx->i2c_tr_rq.transfer = tr;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr) / sizeof(tr[0]);

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err)
    return ctx->i2c_tr_rq.base.err;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST TR 4
  Read
  Write                                                                       */

static error_t
i2c_master_test_tr_4(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_tr_rq.base.saddr;
  uint8_t checksum = get_test_tr_4_checksum(slave_addr);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_buffer(ctx);


#if I2C_MASTER_READ_RESTART_SUPPORT

  struct dev_i2c_ctrl_transaction_data_s tr[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_READ,
    },
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
  };

  ctx->i2c_tr_rq.transfer = tr;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr) / sizeof(tr[0]);

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err)
    return ctx->i2c_tr_rq.base.err;

#else

  struct dev_i2c_ctrl_transaction_data_s tr1[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_READ,
    },
  };

  ctx->i2c_tr_rq.transfer = tr1;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr1) / sizeof(tr1[0]);

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err)
    return ctx->i2c_tr_rq.base.err;

  struct dev_i2c_ctrl_transaction_data_s tr2[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
  };

  ctx->i2c_tr_rq.transfer = tr2;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr2) / sizeof(tr2[0]);

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err)
    return ctx->i2c_tr_rq.base.err;

#endif

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST TR 5
  Read (with address invalid)                                                 */

static error_t
i2c_master_test_tr_5(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_tr_rq.base.saddr;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_buffer(ctx);

  struct dev_i2c_ctrl_transaction_data_s tr[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_READ,
    },
  };

  ctx->i2c_tr_rq.transfer = tr;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr) / sizeof(tr[0]);

  ctx->i2c_tr_rq.base.saddr = I2C_SLAVE_INVALID_ADDR;

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err != -EHOSTUNREACH)
    return TEST_FAILED;

  ctx->i2c_tr_rq.base.saddr = slave_addr;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST TR 6
  Write (with byte 3 invalid)                                                 */

static error_t
i2c_master_test_tr_6(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_tr_rq.base.saddr;
  uint8_t checksum = get_test_tr_6_checksum(slave_addr);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_buffer(ctx);

  struct dev_i2c_ctrl_transaction_data_s tr[] =
  {
    {
      .data = ctx->buffer,
      .size = 8,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
  };

  ctx->i2c_tr_rq.transfer = tr;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr) / sizeof(tr[0]);

  ctx->buffer[3] = I2C_SLAVE_INVALID_BYTE;

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err != -EAGAIN)
    return TEST_FAILED;
  if (ctx->i2c_tr_rq.transfer_index != 0)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}



/*------------------------------------------------------------------------------
  TEST TR 7
  Write
  Write (with byte 3 invalid)                                                 */

static error_t
i2c_master_test_tr_7(struct i2c_master_test_ctx_s *ctx)
{
  uint8_t slave_addr = ctx->i2c_tr_rq.base.saddr;
  uint8_t checksum = get_test_tr_7_checksum(slave_addr);

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_init_slave, 0);
  i2c_master_init_buffer(ctx);

  struct dev_i2c_ctrl_transaction_data_s tr[] =
  {
    {
      .data = ctx->buffer,
      .size = 4,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
    {
      .data = ctx->buffer + 4,
      .size = 4,
      .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
    },
  };

  ctx->i2c_tr_rq.transfer = tr;
  ctx->i2c_tr_rq.transfer_count = sizeof(tr) / sizeof(tr[0]);

  ctx->buffer[6] = I2C_SLAVE_INVALID_BYTE;

  dev_i2c_wait_transaction(&ctx->i2c_ctrl, &ctx->i2c_tr_rq);
  if (ctx->i2c_tr_rq.base.err != -EAGAIN)
    return TEST_FAILED;
  if (ctx->i2c_tr_rq.transfer_index != 1)
    return TEST_FAILED;

  dev_i2c_wait_bytecode(&ctx->i2c_ctrl, &ctx->i2c_bc_rq, &i2c_master_bc_read_debug_info, 0);
#if I2C_MASTER_DISPLAY_DEBUG
  i2c_master_display_debug_info(ctx);
#endif
  if (ctx->debug_info.last_checksum != checksum)
    return TEST_BAD_CHECKSUM;

  return 0;
}


/*----------------------------------------------------------------------------*/


static void
i2c_master_init_test_ctx(struct i2c_master_test_ctx_s *ctx, uint8_t slave_addr)
{
  memset(ctx, 0, sizeof(*ctx));

  if (device_get_accessor_by_path(
      &ctx->i2c_ctrl.base, NULL, I2C_MASTER_I2C_DEV, DRIVER_CLASS_I2C_CTRL))
    {
      printk("error cannot get accessor\n");
      abort();
    }

  ctx->i2c_bc_rq.base.saddr = slave_addr;
  bc_init(&ctx->i2c_bc_rq.vm, &i2c_master_bytecode);

  ctx->i2c_tr_rq.base.saddr = slave_addr;
}

static void
i2c_master_run_tests(struct i2c_master_test_ctx_s *ctx, pthread_mutex_t *lock,
                     const char *prefix,
                     i2c_master_test_t *const *test, size_t test_count)
{
  for (uint8_t i = 0; i < test_count; i++)
  {
    pthread_mutex_lock(lock);
    //printk("[0x%02x] %s %02d starting...\n", ctx->i2c_bc_rq.base.saddr, prefix, i + 1);
    pthread_mutex_unlock(lock);

    error_t err = test[i](ctx);
    const char *status = "OK";

    if (err == TEST_BAD_CHECKSUM)
      status = "bad checksum";
    else if (err == TEST_FAILED)
      status = "failed";
    else if (err)
      status = strerror(err);

    pthread_mutex_lock(lock);
    printk("[0x%02x] %s %02d done: %s\n", ctx->i2c_bc_rq.base.saddr, prefix, i + 1, status);
    pthread_mutex_unlock(lock);
  }

  pthread_mutex_lock(lock);
  //printk("[0x%02x] All %s tests done\n", ctx->i2c_bc_rq.base.saddr, prefix);
  pthread_mutex_unlock(lock);
}

static void
i2c_master_test_bc(struct i2c_master_test_ctx_s *ctx, pthread_mutex_t *lock)
{
  static i2c_master_test_t * const tests_bc[] =
  {
    i2c_master_test_bc_1,
    i2c_master_test_bc_2,
    i2c_master_test_bc_3,
    i2c_master_test_bc_4,
    i2c_master_test_bc_5,
    i2c_master_test_bc_6,
    i2c_master_test_bc_7,
    i2c_master_test_bc_8,
    i2c_master_test_bc_9,
    i2c_master_test_bc_10,
  };

  i2c_master_run_tests(ctx, lock, "bc", tests_bc, ARRAY_SIZE(tests_bc));
}

static void
i2c_master_test_tr(struct i2c_master_test_ctx_s *ctx, pthread_mutex_t *lock)
{
  static i2c_master_test_t * const tests_tr[] =
  {
    i2c_master_test_tr_1,
    i2c_master_test_tr_2,
    i2c_master_test_tr_3,
    i2c_master_test_tr_4,
    i2c_master_test_tr_5,
    i2c_master_test_tr_6,
    i2c_master_test_tr_7,
  };

  i2c_master_run_tests(ctx, lock, "tr", tests_tr, ARRAY_SIZE(tests_tr));
}

static void *
i2c_master_test(void *arg)
{
  struct i2c_master_slaves_s *slaves = (struct i2c_master_slaves_s *)arg;
  uint8_t slave_addr;

  while (1)
    {
      pthread_mutex_lock(&slaves->lock);
      if (slaves->current >= slaves->count)
        {
          pthread_mutex_unlock(&slaves->lock);
          semaphore_give(&slaves->sem, 1);
          return 0;
        }
      slave_addr = slaves->addr[slaves->current++];
      pthread_mutex_unlock(&slaves->lock);

      struct i2c_master_test_ctx_s ctx;
      i2c_master_init_test_ctx(&ctx, slave_addr);

      bc_set_reg(&ctx.i2c_bc_rq.vm, 11, (bc_reg_t)&ctx.debug_info.size);
      bc_set_reg(&ctx.i2c_bc_rq.vm, 12, (bc_reg_t)&ctx.debug_info.last_checksum);
      bc_set_reg(&ctx.i2c_bc_rq.vm, 13, (bc_reg_t)ctx.debug_info.log);
      bc_set_reg(&ctx.i2c_bc_rq.vm, 14, (bc_reg_t)ctx.debug_info.checksum);

      i2c_master_test_bc(&ctx, &slaves->lock);
      i2c_master_test_tr(&ctx, &slaves->lock);

      pthread_mutex_lock(&slaves->lock);
      //printk("[0x%02x] All done\n", slave_addr);
      pthread_mutex_unlock(&slaves->lock);
  }
}

static void
i2c_master_scan(struct i2c_master_slaves_s *slaves)
{
  struct i2c_master_test_ctx_s ctx;
  i2c_master_init_test_ctx(&ctx, 0);

  bc_set_reg(&ctx.i2c_bc_rq.vm, 0, (bc_reg_t)slaves->addr);
  dev_i2c_wait_bytecode(&ctx.i2c_ctrl, &ctx.i2c_bc_rq, &i2c_master_bc_scan, 0);

  slaves->count = (uint8_t *)bc_get_reg(&ctx.i2c_bc_rq.vm, 0) - slaves->addr;
}

void
main(void)
{
  struct i2c_master_slaves_s slaves;
  memset(&slaves, 0, sizeof(slaves));

  printk("\n--- SCAN ---\n\n");

  i2c_master_scan(&slaves);
  if (slaves.count == 0)
    {
      printk("ERROR: No slave found\n");
      abort();
    }

  printk("\n%d slaves found:\n", slaves.count);
  for (uint8_t i = 0; i < slaves.count; i++)
    printk("  [0x%02x]\n", slaves.addr[i]);

  printk("\n--- TESTS ---\n\n");

  pthread_t threads[I2C_MASTER_MAX_THREADS];

  pthread_mutex_init(&slaves.lock, NULL);
  semaphore_init(&slaves.sem, 0);

  for (uint8_t i = 0; i < I2C_MASTER_MAX_THREADS; i++)
    {
      error_t err;
      if ((err = pthread_create(&threads[i], NULL, i2c_master_test, &slaves)))
        {
          printk("error cannot create thread [%d]\n", err);
          abort();
        }
    }

  semaphore_take(&slaves.sem, I2C_MASTER_MAX_THREADS);
  semaphore_destroy(&slaves.sem);
  printk("DONE\n");
}

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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/driver.h>
#include <device/request.h>
#include <device/class/timer.h>
#include <device/class/valio.h>
#include <device/valio/motion_sensor.h>
#include <device/irq.h>

#include <device/class/i2c.h>

#include "mpu6505_regs.h"

#define dprintk(k...) do {} while (0)
//#define dprintk printk

__attribute__((packed))
struct mpu6505_sensor_block_s
{
  // All big-endian
  uint8_t tmp;
  uint8_t it;
  uint16_t accel[3];
  uint16_t temp;
  uint16_t gyro[3];
};

enum mpu6505_power_mode_e
{
  MPU6505_POWER_OFF,
  MPU6505_POWER_ON,
  MPU6505_ACCEL_CALIBRATED,
  MPU6505_GYRO_CALIBRATED,
  MPU6505_WOM,
  MPU6505_WOM_NOTIFIED,
  MPU6505_STREAMING,
};

#define STREAMING_FPS 16
#define GYRO_AUTO (STREAMING_FPS)
#define GYRO_AUTO_TIME (GYRO_AUTO * 4)
#define WOM_AUTO (STREAMING_FPS * 10)

struct mpu6505_private_s
{
  struct device_i2c_s i2c;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_req;
  struct dev_irq_src_s irq;
  struct dev_i2c_rq_s i2c_req;
  struct dev_i2c_transfer_s i2c_transfer[2];
  struct dev_valio_rq_s *running;
  struct mpu6505_sensor_block_s last_data;
  dev_request_queue_root_t queue;

  int16_t offset[VALIO_MS_GYRO_Z + 1];
  int32_t gyro_auto[3];
  uint8_t gyro_auto_left;
  uint8_t wom_auto_left;

  enum mpu6505_power_mode_e power_mode : 4;
  enum mpu6505_power_mode_e next_mode : 4;
  bool_t read_pending : 1;
  uint8_t i2c_saddr;

  uint8_t tmp[11];

  union {
    uint8_t wdata[6];
    struct {
      const uint8_t *data;
      uint8_t size;
    } sequence;
  };
};

static KROUTINE_EXEC(mpu6505_tick);

static
void mpu6505_request_run_first(struct device_s *dev);

static bool_t rq_is_running(struct device_s *dev)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  return pv->i2c_req.base.pvdata != NULL || pv->running;
}

static void rq_start(struct device_s *dev, kroutine_exec_t *kr)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  assert(!pv->i2c_req.base.pvdata);
  pv->i2c_req.base.pvdata = dev;

  kroutine_init_sched_switch(&pv->i2c_req.base.kr, kr);
  DEVICE_OP(&pv->i2c, request, &pv->i2c_req);
}

static void rq_done_(struct device_s *dev, struct dev_valio_rq_s *rq, error_t err)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  if (rq) {
    rq->error = err;

    dev_request_queue_remove(&pv->queue, &rq->base);

    kroutine_exec(&rq->base.kr);
  }

  pv->i2c_req.base.pvdata = NULL;
  mpu6505_request_run_first(dev);
}

static void rq_done(struct device_s *dev, struct dev_valio_rq_s *rq, error_t err)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  assert(pv->i2c_req.base.pvdata);

  rq_done_(dev, rq, err);
}

static KROUTINE_EXEC(mpu6505_write_sequence_done);
static void write_sequence_next(struct device_s *dev);

static
void mpu6505_do_write_sequence(struct device_s *dev,
                               const uint8_t *data, uint8_t size)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  dprintk("%s %P\n", __FUNCTION__, data, size);

  pv->sequence.data = data;
  pv->sequence.size = size;

  write_sequence_next(dev);
};

static
void write_sequence_next(struct device_s *dev)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  if (pv->i2c_req.error) {
    dprintk("i2c req error: %d after transfer %d offset %d\n",
           pv->i2c_req.error, pv->i2c_req.error_transfer, pv->i2c_req.error_offset);
  }

  pv->i2c_req.transfer = pv->i2c_transfer;
  pv->i2c_req.saddr = pv->i2c_saddr;
  pv->i2c_req.transfer_count = 1;

  pv->i2c_transfer[0].type = DEV_I2C_WRITE;
  pv->i2c_transfer[0].size = *pv->sequence.data;
  pv->i2c_transfer[0].data = (uint8_t *)pv->sequence.data + 1;

  assert(pv->sequence.size >= *pv->sequence.data + 1);

  pv->sequence.size -= *pv->sequence.data + 1;
  pv->sequence.data += *pv->sequence.data + 1;

  dprintk("%s %P\n", __FUNCTION__, pv->i2c_transfer[0].data, pv->i2c_transfer[0].size);

  rq_start(dev, mpu6505_write_sequence_done);
}

static
KROUTINE_EXEC(mpu6505_write_sequence_done)
{
  struct device_s *dev;
  struct mpu6505_private_s *pv;
  struct dev_valio_rq_s *rq;

  pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
  rq = pv->running;
  dev = pv->i2c_req.base.pvdata;

  LOCK_SPIN_IRQ(&dev->lock);

  if (pv->sequence.size) {
    pv->i2c_req.base.pvdata = NULL;
    write_sequence_next(dev);
  } else {
    rq_done(dev, rq, pv->i2c_req.error);
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static KROUTINE_EXEC(mpu6505_sensor_read_done);

static
void mpu6505_do_sensor_read(struct device_s *dev,
                            struct dev_valio_rq_s *rq)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  pv->i2c_req.transfer = pv->i2c_transfer;
  pv->i2c_req.saddr = pv->i2c_saddr;

  pv->i2c_transfer[0].data = pv->wdata;
  pv->i2c_transfer[0].size = 1;
  pv->i2c_transfer[0].type = DEV_I2C_WRITE;
  pv->wdata[0] = REG_SENSOR_BLOCK_BEGIN - 1;

  pv->i2c_transfer[1].data = (uint8_t*)&pv->last_data + 1;
  pv->i2c_transfer[1].size = sizeof(pv->last_data) - 1;
  pv->i2c_transfer[1].type = DEV_I2C_READ;

  pv->i2c_req.transfer_count = 2;

  pv->running = rq;

  rq_start(dev, mpu6505_sensor_read_done);
}

static
KROUTINE_EXEC(mpu6505_sensor_read_done)
{
  struct device_s *dev;
  struct mpu6505_private_s *pv;
  struct valio_ms_state_s *val;
  struct dev_valio_rq_s *rq;

  pv = KROUTINE_CONTAINER(kr, *pv, i2c_req.base.kr);
  rq = pv->running;
  pv->running = NULL;
  dev = pv->i2c_req.base.pvdata;

  pv->read_pending = 0;

  dprintk("%s i2c req error: %d after transfer %d offset %d rq %p\n", __FUNCTION__,
         pv->i2c_req.error, pv->i2c_req.error_transfer, pv->i2c_req.error_offset,
         rq);

  dprintk("IRQ: %02x\n", pv->last_data.it);
  dprintk("data read: %P\n", &pv->last_data, 14);

  if (!(pv->last_data.it & REG_INT_ENABLE_WOM)) {
    if (pv->gyro_auto_left) {
      pv->gyro_auto[0] += (int16_t)endian_be16(pv->last_data.gyro[0]) - pv->gyro_auto[0] / GYRO_AUTO;
      pv->gyro_auto[1] += (int16_t)endian_be16(pv->last_data.gyro[1]) - pv->gyro_auto[1] / GYRO_AUTO;
      pv->gyro_auto[2] += (int16_t)endian_be16(pv->last_data.gyro[2]) - pv->gyro_auto[2] / GYRO_AUTO;
      pv->gyro_auto_left--;
      if (!pv->gyro_auto_left) {
        pv->offset[VALIO_MS_GYRO_X] += pv->gyro_auto[0] / GYRO_AUTO;
        pv->offset[VALIO_MS_GYRO_Y] += pv->gyro_auto[1] / GYRO_AUTO;
        pv->offset[VALIO_MS_GYRO_Z] += pv->gyro_auto[2] / GYRO_AUTO;
        pv->power_mode = MPU6505_ACCEL_CALIBRATED;

        dprintk("Gyro offsets: %d %d %d\n",
               pv->offset[VALIO_MS_GYRO_X],
               pv->offset[VALIO_MS_GYRO_Y],
               pv->offset[VALIO_MS_GYRO_Z]);
      }
    }

    if (pv->wom_auto_left) {
      pv->wom_auto_left--;
      if (!pv->wom_auto_left) {
        pv->next_mode = MPU6505_WOM;
      }
    }
  } else {
    pv->wom_auto_left = WOM_AUTO;
    pv->next_mode = MPU6505_STREAMING;
  }

  if (rq) {
    val = rq->data;

    val->axis[VALIO_MS_ACCEL_X] = - (int16_t)endian_be16(pv->last_data.accel[0]) / 4;
    val->axis[VALIO_MS_ACCEL_Y] = - (int16_t)endian_be16(pv->last_data.accel[1]) / 4;
    val->axis[VALIO_MS_ACCEL_Z] = - (int16_t)endian_be16(pv->last_data.accel[2]) / 4;

    val->axis[VALIO_MS_GYRO_X] = (int16_t)endian_be16(pv->last_data.gyro[0]) / 2;
    val->axis[VALIO_MS_GYRO_Y] = (int16_t)endian_be16(pv->last_data.gyro[1]) / 2;
    val->axis[VALIO_MS_GYRO_Z] = (int16_t)endian_be16(pv->last_data.gyro[2]) / 2;
  }

  LOCK_SPIN_IRQ(&dev->lock);
  rq_done(dev, rq, pv->i2c_req.error);
  LOCK_RELEASE_IRQ(&dev->lock);
}

static void mpu6505_read_static(struct device_s *dev, struct valio_ms_state_s *val)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  val->axis[VALIO_MS_ACCEL_X] = - (int16_t)endian_be16(pv->last_data.accel[0]) / 4;
  val->axis[VALIO_MS_ACCEL_Y] = - (int16_t)endian_be16(pv->last_data.accel[1]) / 4;
  val->axis[VALIO_MS_ACCEL_Z] = - (int16_t)endian_be16(pv->last_data.accel[2]) / 4;
  val->axis[VALIO_MS_GYRO_X] = 0;
  val->axis[VALIO_MS_GYRO_Y] = 0;
  val->axis[VALIO_MS_GYRO_Z] = 0;
}

static
bool_t mpu6505_switch_mode(struct device_s *dev, enum mpu6505_power_mode_e mode)
{
  struct mpu6505_private_s *pv = dev->drv_pv;
  static const uint8_t poweron[] = {
    2, REG_PWR_MGMT_1,
    REG_PWR_MGMT_1_H_RESET, // Reset all

    6, REG_SIGNAL_PATH_RESET,
    0x7, // Reset all
    0,
    REG_USER_FIFO_RST,
    REG_PWR_MGMT_1_TEMP_DIS,
    REG_PWR_MGMT_2_LP_WAKE_CTRL_20_HZ,

    2, REG_INT_PIN_CFG,
    REG_INT_PIN_CFG_ACTIVE_HIGH | REG_INT_PIN_CFG_PUSHPULL
       | REG_INT_PIN_CFG_LEVEL | REG_INT_PIN_CLEAR_READ_ANY,

    7, REG_CONFIG,
    REG_CONFIG_DLPF_250HZ,
    REG_GYRO_CONFIG_2000DPS | REG_GYRO_CONFIG_DLPF,
    REG_ACCEL_CONFIG_8G,
    REG_ACCEL_CONFIG2_184HZ,
    REG_LP_ACCEL_ODR_15_63HZ,
    REG_WOM_THR_MG(40),

    2, REG_FIFO_EN,
    0,
  };
  static const uint8_t poweroff[] = {
    2, REG_PWR_MGMT_1,
    REG_PWR_MGMT_1_SLEEP,
  };
  static const uint8_t wom[] = {
    2, REG_INT_ENABLE,
    REG_INT_ENABLE_WOM,

    5, REG_ACCEL_INTEL_CTRL,
    REG_ACCEL_INTEL_EN | REG_ACCEL_INTEL_MODE_COMPARE,
    0,
    REG_PWR_MGMT_1_CYCLE | REG_PWR_MGMT_1_GYRO_STANDBY | REG_PWR_MGMT_1_TEMP_DIS,
    //REG_PWR_MGMT_2_LP_WAKE_CTRL_20_HZ | 
    REG_PWR_MGMT_2_DISABLE_XG | REG_PWR_MGMT_2_DISABLE_YG | REG_PWR_MGMT_2_DISABLE_ZG,
  };
  static const uint8_t streaming[] = {
    2, REG_INT_ENABLE,
    0,

    5, REG_ACCEL_INTEL_CTRL,
    REG_ACCEL_INTEL_EN | REG_ACCEL_INTEL_MODE_COMPARE,
    0,
    REG_PWR_MGMT_1_TEMP_DIS,
    0,
  };

  dprintk("%s %d -> %d\n", __FUNCTION__, pv->power_mode, mode);

  if (mode == pv->power_mode)
    return 0;

  if (mode == MPU6505_POWER_OFF) {
    mpu6505_do_write_sequence(dev, poweroff, sizeof(poweroff));
    pv->power_mode = MPU6505_POWER_OFF;
    return 1;
  }

  switch (pv->power_mode) {
  case MPU6505_POWER_OFF:
    mpu6505_do_write_sequence(dev, poweron, sizeof(poweron));
    pv->power_mode = MPU6505_POWER_ON;
    return 1;

  case MPU6505_POWER_ON:
    pv->tmp[0] = 7;
    pv->tmp[1] = REG6050_XA_OFFSET_H;
    pv->tmp[2] = (pv->offset[VALIO_MS_ACCEL_X] >> 7) & 0xff;
    pv->tmp[3] = (pv->offset[VALIO_MS_ACCEL_X] << 1) & 0xff;
    pv->tmp[4] = (pv->offset[VALIO_MS_ACCEL_Y] >> 7) & 0xff;
    pv->tmp[5] = (pv->offset[VALIO_MS_ACCEL_Y] << 1) & 0xff;
    pv->tmp[6] = (pv->offset[VALIO_MS_ACCEL_Z] >> 7) & 0xff;
    pv->tmp[7] = (pv->offset[VALIO_MS_ACCEL_Z] << 1) & 0xff;
    mpu6505_do_write_sequence(dev, pv->tmp, 8);
    pv->power_mode = MPU6505_ACCEL_CALIBRATED;

    pv->gyro_auto_left = GYRO_AUTO_TIME;
    pv->gyro_auto[0] = 0;
    pv->gyro_auto[1] = 0;
    pv->gyro_auto[2] = 0;

    return 1;

  case MPU6505_ACCEL_CALIBRATED:
    pv->tmp[0] = 7;
    pv->tmp[1] = REG_XG_OFFSET_H;
    pv->tmp[2] = ((-pv->offset[VALIO_MS_GYRO_X] * 2) >> 8) & 0xff;
    pv->tmp[3] = ((-pv->offset[VALIO_MS_GYRO_X] * 2)) & 0xff;
    pv->tmp[4] = ((-pv->offset[VALIO_MS_GYRO_Y] * 2) >> 8) & 0xff;
    pv->tmp[5] = ((-pv->offset[VALIO_MS_GYRO_Y] * 2)) & 0xff;
    pv->tmp[6] = ((-pv->offset[VALIO_MS_GYRO_Z] * 2) >> 8) & 0xff;
    pv->tmp[7] = ((-pv->offset[VALIO_MS_GYRO_Z] * 2)) & 0xff;
    mpu6505_do_write_sequence(dev, pv->tmp, 8);
    pv->power_mode = MPU6505_GYRO_CALIBRATED;
    return 1;

  case MPU6505_WOM:
  case MPU6505_WOM_NOTIFIED:
    switch (mode) {
    case MPU6505_WOM:
    case MPU6505_WOM_NOTIFIED:
      pv->power_mode = mode;
      return 0;
    default:
      break;
    }
    // Fallthrough
  case MPU6505_GYRO_CALIBRATED:
  case MPU6505_STREAMING:
    switch (mode) {
    case MPU6505_WOM:
    case MPU6505_WOM_NOTIFIED:
      mpu6505_do_write_sequence(dev, wom, sizeof(wom));
      pv->power_mode = MPU6505_WOM;
      return 1;

    case MPU6505_STREAMING:
      mpu6505_do_write_sequence(dev, streaming, sizeof(streaming));
      pv->power_mode = MPU6505_STREAMING;
      pv->wom_auto_left = WOM_AUTO;
      if (pv->timer_req.rq.drvdata)
        DEVICE_OP(&pv->timer, cancel, &pv->timer_req);

      dev_timer_init_sec(&pv->timer, &pv->timer_req.delay, &pv->timer_req.rev, 1, STREAMING_FPS);
      DEVICE_OP(&pv->timer, request, &pv->timer_req);
      return 1;

    default:
      break;
    }
  }

  assert(0);

  return 0;
}

static
void mpu6505_request_run_first(struct device_s *dev)
{
  struct mpu6505_private_s *pv = dev->drv_pv;
  struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue));

  assert(pv);

  if (rq_is_running(dev))
    return;

  dprintk("%s %p %d\n", __FUNCTION__, rq, rq->type);

  if (!rq) {
    if (pv->read_pending)
      mpu6505_switch_mode(dev, MPU6505_POWER_OFF);
    return;
  }

  switch (rq->attribute) {
  case VALIO_MS_CALIBRATE:
    switch (rq->type) {
    case DEVICE_VALIO_WRITE:
      dprintk("%s write %d\n", __FUNCTION__, rq->attribute);
      memcpy(pv->offset, rq->data, 6 * 2);
      // Trigger calibration values update
      if (pv->power_mode > MPU6505_POWER_ON)
        pv->power_mode = MPU6505_POWER_ON;
      break;

    case DEVICE_VALIO_READ:
      memcpy(rq->data, pv->offset, 6 * 2);
      break;
    }

    rq_done_(dev, rq, 0);
    return;
  }

  switch (rq->type) {
  case DEVICE_VALIO_READ:
  case DEVICE_VALIO_WAIT_EVENT:
    if (pv->power_mode <= MPU6505_GYRO_CALIBRATED)
      pv->next_mode = MPU6505_STREAMING;
    break;
  }

  if (pv->power_mode != pv->next_mode &&
      mpu6505_switch_mode(dev, pv->next_mode))
    return;

  switch (rq->type) {
  case DEVICE_VALIO_READ:
    dprintk("%s read\n", __FUNCTION__);

    mpu6505_do_sensor_read(dev, rq);
    return;

  case DEVICE_VALIO_WAIT_EVENT:
    dprintk("%s wait update\n", __FUNCTION__);

    if (pv->read_pending)
      mpu6505_do_sensor_read(dev, rq);
    else if (pv->power_mode == MPU6505_WOM) {
      mpu6505_read_static(dev, rq->data);
      pv->next_mode = MPU6505_WOM_NOTIFIED;
      rq_done_(dev, rq, 0);
    }
    return;
  }
}

static
DEV_VALIO_REQUEST(mpu6505_request)
{
  struct device_s *dev = accessor->dev;
  struct mpu6505_private_s *pv = dev->drv_pv;
  error_t err = -EINVAL;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (req->attribute) {
  case VALIO_MS_STATE:
    if (!((1 << req->type) & ((1 << DEVICE_VALIO_READ) | (1 << DEVICE_VALIO_WAIT_EVENT))))
      break;
    err = 0;
    break;

  case VALIO_MS_CALIBRATE:
    if (!((1 << req->type) & ((1 << DEVICE_VALIO_READ) | (1 << DEVICE_VALIO_WRITE))))
      break;
    err = 0;
    break;

  default:
    break;
  }

  if (err == 0) {
    dev_request_queue_pushback(&pv->queue, &req->base);
    mpu6505_request_run_first(dev);
  }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (err < 0) {
    req->error = err;
    kroutine_exec(&req->base.kr);
  }
}

static DEV_IRQ_SRC_PROCESS(mpu6505_irq)
{
  struct device_s *dev = ep->base.dev;
  struct mpu6505_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  dprintk("%s\n", __FUNCTION__);

  switch (pv->power_mode) {
  case MPU6505_WOM:
  case MPU6505_WOM_NOTIFIED:
    pv->read_pending = 1;
    mpu6505_request_run_first(dev);
    break;
  }

  lock_release(&dev->lock);
}

static KROUTINE_EXEC(mpu6505_tick)
{
  struct mpu6505_private_s *pv = KROUTINE_CONTAINER(kr, *pv, timer_req.rq.kr);
  struct device_s *dev = pv->timer_req.rq.pvdata;

  dprintk("%s\n", __FUNCTION__);

  lock_spin(&dev->lock);

  if (pv->power_mode == MPU6505_STREAMING) {
    pv->read_pending = 1;

    mpu6505_request_run_first(dev);

    DEVICE_OP(&pv->timer, request, &pv->timer_req);
  }

  lock_release(&dev->lock);
}


#define mpu6505_cancel (dev_valio_cancel_t*)&dev_driver_notsup_fcn

static DEV_INIT(mpu6505_init)
{
  struct mpu6505_private_s *pv;


  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  memset(pv, 0, sizeof(*pv));
  if (!pv)
    return -ENOMEM;

  if (device_get_param_dev_accessor(dev, "i2c", &pv->i2c, DRIVER_CLASS_I2C)) {
    dprintk("Bus not found");
    goto err_pv;
  }

  if (device_get_param_dev_accessor(dev, "timer", &pv->timer, DRIVER_CLASS_TIMER)) {
    dprintk("Timer not found");
    goto err_pv;
  }

  if (dev_i2c_res_get_addr(dev, &pv->i2c_saddr, 0)) {
    dprintk("Slave address not found");
    goto err_pv;
  }

  kroutine_init_sched_switch(&pv->timer_req.rq.kr, mpu6505_tick);

  device_irq_source_init(dev, &pv->irq, 1, &mpu6505_irq);

  if (device_irq_source_link(dev, &pv->irq, 1, -1))
    goto err_pv;

  dev_request_queue_init(&pv->queue);

  dev->drv_pv = pv;
  pv->timer_req.rq.pvdata = dev;
  pv->power_mode = -1;

  mpu6505_switch_mode(dev, MPU6505_POWER_OFF);

  return 0;

 err_pv:
  mem_free(pv);
  return 1;
}

static DEV_CLEANUP(mpu6505_cleanup)
{
  struct mpu6505_private_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  device_put_accessor(&pv->i2c.base);
  device_irq_source_unlink(dev, &pv->irq, 1);
  dev_request_queue_destroy(&pv->queue);
  mem_free(pv);

  return 0;
}

static DEV_USE(mpu6505_use)
{
  struct device_accessor_s *accessor = param;

  switch (op) {

  case DEV_USE_START: {
    struct device_s *dev = accessor->dev;
    struct mpu6505_private_s *pv = dev->drv_pv;
    if (dev->start_count == 0)
      return device_start(&pv->timer.base);
  }

  case DEV_USE_STOP: {
    struct device_s *dev = accessor->dev;
    struct mpu6505_private_s *pv = dev->drv_pv;
    if (dev->start_count == 0)
      device_stop(&pv->timer.base);
    return 0;
  }

  default:
    return dev_use_generic(param, op);
  }
}

DRIVER_DECLARE(mpu6505_drv, 0, "MPU6505 motion", mpu6505,
               DRIVER_VALIO_METHODS(mpu6505));

DRIVER_REGISTER(mpu6505_drv);


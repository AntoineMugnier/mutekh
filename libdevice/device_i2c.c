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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

    Synchronous read and write functions for i2c devices.

*/

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/i2c.h>

#include <hexo/lock.h>
#include <hexo/interrupt.h>

#if defined(CONFIG_DEVICE_I2C_REQUEST)
# include <mutek/bytecode.h>
#endif

#if defined(CONFIG_MUTEK_SCHEDULER)
# include <mutek/scheduler.h>
#endif

#if defined(CONFIG_DEVICE_I2C_REQUEST)
GCT_CONTAINER_PROTOTYPES(dev_i2c_ctrl_queue, extern inline, dev_i2c_ctrl_queue,
                   init, destroy, pop, remove, push, push_back, isempty);
#endif

/* structure that is used for blocking calls. */
struct dev_i2c_ctrl_wait_rq_s
{
  struct kroutine_s      kr;
#if defined(CONFIG_MUTEK_SCHEDULER)
  lock_t                 lock;
  struct sched_context_s *ctx;
#endif
  bool_t                 done;
};


#if defined(CONFIG_DEVICE_I2C_REQUEST)

/****************************** scheduler api ****/

static
enum dev_i2c_sched_state_e
dev_i2c_ctrl_sched_transfer(struct dev_i2c_ctrl_request_s    *rq,
                            enum dev_i2c_ctrl_transfer_op_e  op,
                            uint8_t                          saddr,
                            enum dev_i2c_ctrl_transfer_dir_e dir,
                            uint8_t                          *buffer,
                            size_t                           size);

static
void dev_i2c_ctrl_sched_end(struct dev_i2c_ctrl_request_s *rq, error_t err);

static
enum dev_i2c_sched_state_e
dev_i2c_ctrl_sched_exec(struct dev_i2c_ctrl_sched_s *sched);

static
enum dev_i2c_sched_state_e
dev_i2c_ctrl_sched_next(struct dev_i2c_ctrl_sched_s *sched);

static
void dev_i2c_ctrl_sched_run(struct dev_i2c_ctrl_sched_s *sched);


error_t dev_i2c_ctrl_sched_init(struct device_s             *dev,
                                struct dev_i2c_ctrl_sched_s *sched)
{
  sched->config  = NULL;
  sched->curr_rq = NULL;
  sched->running = 0;

  dev_i2c_ctrl_queue_init(&sched->queue);
  lock_init_irq(&sched->lock);
  memset(&sched->transfer, 0, sizeof(sched->transfer));

  return 0;
}

void dev_i2c_ctrl_sched_destroy(struct dev_i2c_ctrl_sched_s *sched)
{
  lock_destroy_irq(&sched->lock);
  dev_i2c_ctrl_queue_destroy(&sched->queue);
}

error_t dev_i2c_request_init(struct device_s               *dev,
                             struct dev_i2c_ctrl_request_s *rq)
{
  memset(rq, 0, sizeof(*rq));

  if (device_get_param_dev_accessor(dev, "i2c", &rq->i2cdev, DRIVER_CLASS_I2C_CTRL))
    return -ENOENT;

  rq->sched = DEVICE_OP(&rq->i2cdev, sched);
  return 0;
}

void dev_i2c_request_destroy(struct dev_i2c_ctrl_request_s *rq)
{
  device_put_accessor(&rq->i2cdev);
}

/** @This enumeration defines the different states the I2C scheduler. */
enum dev_i2c_sched_state_e
{
  DEV_I2C_SCHED_IDLE,
  DEV_I2C_SCHED_CONTINUE,
  DEV_I2C_SCHED_WAIT_TRANSFER,
};

/** @This function terminates the scheduling of a given request. It espacially
    calls the termination routine. */
static
void dev_i2c_ctrl_sched_end(struct dev_i2c_ctrl_request_s *rq, error_t err)
{
  struct dev_i2c_ctrl_sched_s *sched = rq->sched;

  if (rq == sched->curr_rq)
    {
      sched->config  = NULL;
      sched->curr_rq = NULL;
    }
  else
    {
      assert(sched->curr_rq == NULL);
      dev_i2c_ctrl_queue_remove(&sched->queue, rq);
    }

  rq->error    = err;
  rq->enqueued = 0;

  lock_release_irq(&sched->lock);
  kroutine_exec(&rq->kr, cpu_is_interruptible());
  lock_spin_irq(&sched->lock);
}

/** @This routine is call upon the end of an I2C transfer for a given request.
    The routine sets the request as done and terminates the call chain of
    subsequent routines. */
static
KROUTINE_EXEC(dev_i2c_ctrl_sched_transfer_end)
{
  struct dev_i2c_ctrl_transfer_s *tr    = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_request_s  *rq    = tr->pvdata;
  struct dev_i2c_ctrl_sched_s    *sched = rq->sched;

  lock_spin_irq(&sched->lock);

  /* check for errors. */
  if (tr->error != 0)
    dev_i2c_ctrl_sched_end(rq, tr->error);

  /* if succeeded, check if direct call was used and then mark as done. */
  else if (rq->type == DEV_I2C_REQ_INFO)
    rq->u.info.done = 1;

  /* if succeeded, check if bytecode was used and then consume one
     instruction. */
  else if (rq->type == DEV_I2C_REQ_VM)
    bc_skip(&rq->u.vm);

  /* if we were in the scheduler, relaunch it. */
  if (kroutine_triggered_1st(kr))
    dev_i2c_ctrl_sched_run(sched);

  /* otherwise, simply quit. */
  else
    lock_release_irq(&sched->lock);
}

/** @This does an actual I2C transfer for the given request using information
    provided in arguments. These information may come from either direct
    call or bytecode undifferently.

    @returns the new state of the scheduler.
 */
static
enum dev_i2c_sched_state_e
dev_i2c_ctrl_sched_transfer(struct dev_i2c_ctrl_request_s    *rq,
                            enum dev_i2c_ctrl_transfer_op_e  op,
                            uint8_t                          saddr,
                            enum dev_i2c_ctrl_transfer_dir_e dir,
                            uint8_t                          *buffer,
                            size_t                           size)
{
  struct dev_i2c_ctrl_sched_s    *sched = rq->sched;
  struct dev_i2c_ctrl_transfer_s *tr    = &sched->transfer;
  error_t err;

  if (sched->config != &rq->config)
    {
      if ((err = DEVICE_OP(&rq->i2cdev, config, &rq->config)))
        goto err_config;
      sched->config = &rq->config;
    }

  tr->op    = op;
  tr->saddr = saddr;
  tr->dir   = dir;
  tr->data  = buffer;
  tr->count = size;

  tr->error = 0;

  kroutine_init(&tr->kr, &dev_i2c_ctrl_sched_transfer_end, KROUTINE_TRIGGER);
  tr->pvdata = rq;

  DEVICE_OP(&rq->i2cdev, transfer, tr);

  lock_spin_irq(&sched->lock);
  return DEV_I2C_SCHED_WAIT_TRANSFER;

err_config:
  lock_spin_irq(&sched->lock);
  dev_i2c_ctrl_sched_end(rq, -err /* reverse negative error. */);
  return DEV_I2C_SCHED_CONTINUE;
}

static
enum dev_i2c_sched_state_e
dev_i2c_ctrl_sched_exec_bytecode(struct dev_i2c_ctrl_sched_s *sched)
{
  struct dev_i2c_ctrl_request_s *rq = sched->curr_rq;
  assert(rq != NULL);

  error_t  err;
  uint16_t op;

  for (err = 0; err == 0; bc_skip(&rq->u.vm))
    {
      /* run indefinitely. */
      op = bc_run(&rq->u.vm, -1);

      if (!(op & 0x8000)) /* bytecode end. */
        {
          if (op)
            err = -EINVAL;
          break;
        }

      switch (op & 0x4000)
        {
        default:
          continue;

        case 0x4000: /* special. */
          switch (op & 0x1000)
            {
            default:
              continue;

            case 0x0000: /* configure bit rate. */
              {
                rq->config.bit_rate = op & 0xf;
                sched->config       = NULL;
                continue;
              }

            case 0x1000: /* yield. */
              lock_spin_irq(&sched->lock);
              dev_i2c_ctrl_queue_pushback(&sched->queue, rq);
              sched->curr_rq = NULL;
              bc_skip(&rq->u.vm);
              return DEV_I2C_SCHED_CONTINUE;
            }
          break;

        case 0x0000: { /* read/write. */
          /* I2C operation (START, STOP, START/STOP). */
          enum dev_i2c_ctrl_transfer_op_e i2cop = (op >> 12) & 0x3;

          /* I2C device address (slave device). */
          uint_fast8_t sreg  = (op >> 4) & 0xf;
          uint_fast8_t saddr = bc_get_reg(&rq->u.vm, sreg) & 0xf;

          /* data. */
          uint8_t *buffer = (uint8_t *) bc_get_reg(&rq->u.vm, op & 0xf);
          size_t  size    = ((op >> 8) & 0x7) + 1;

          /* I2C transfer direction. */
          enum dev_i2c_ctrl_transfer_dir_e dir = (op >> 11) & 0x1;

          /* launch the transfer. */
          return dev_i2c_ctrl_sched_transfer(rq, i2cop, saddr, dir, buffer, size);
        }
        }
    }

  lock_spin_irq(&sched->lock);
  dev_i2c_ctrl_sched_end(rq, err);
  return DEV_I2C_SCHED_CONTINUE;
}

/** @This executes the current request depending if the request comes from
    direct call or comes from bytecode.

    @returns the new state of the scheduler.
 */
static
enum dev_i2c_sched_state_e
dev_i2c_ctrl_sched_exec(struct dev_i2c_ctrl_sched_s *sched)
{
  struct dev_i2c_ctrl_request_s *rq = sched->curr_rq;
  assert (rq != NULL);

  lock_release_irq(&sched->lock);

  //printk("i2c: execute request %p.\n", rq);

  error_t err = 0;
  switch (rq->type)
    {
    default:
      err = ENOTSUP;
      break;

    /* use direct C function call. */
    case DEV_I2C_REQ_INFO:
      /* If the request is done, this this turn is for handling termination.
         So break and execute termination function @see dev_i2c_ctrl_sched_end.
       */
      if (rq->u.info.done)
        break;

      return dev_i2c_ctrl_sched_transfer(rq, rq->u.info.devrq->op,
        rq->u.info.saddr, rq->u.info.devrq->dir, rq->u.info.devrq->data,
        rq->u.info.devrq->size);

    /* use I2C bytecode. */
    case DEV_I2C_REQ_VM:
      return dev_i2c_ctrl_sched_exec_bytecode(sched);
    }

  lock_spin_irq(&sched->lock);
  dev_i2c_ctrl_sched_end(rq, err);
  return DEV_I2C_SCHED_CONTINUE;
}

/** @This pop the next available request from the scheduler queue. If the
    queue is empty, the scheduler goes idle. Otherwise, the next available
    request is executed.

    @returns the new state of the scheduler.
 */
static
enum dev_i2c_sched_state_e
dev_i2c_ctrl_sched_next(struct dev_i2c_ctrl_sched_s *sched)
{
  struct dev_i2c_ctrl_request_s *rq = NULL;

  //printk("i2c: look for next request.\n");

  assert(sched->curr_rq == NULL);
  rq = dev_i2c_ctrl_queue_pop(&sched->queue);
  if (rq == NULL)
    return DEV_I2C_SCHED_IDLE;

  sched->curr_rq = rq;

  /* prepare request for being scheduled. */
  if (rq->type == DEV_I2C_REQ_INFO)
    rq->u.info.done = 0;

  //printk("i2c: schedule next request %p.\n", rq);

  return DEV_I2C_SCHED_CONTINUE;
}

/** @This runs the scheduler until there is no new request available in the
    queue. A push of a new request will restart the scheduler.
    @see dev_i2c_request_sched */
static
void dev_i2c_ctrl_sched_run(struct dev_i2c_ctrl_sched_s *sched)
{
  assert(!sched->running);
  sched->running = 1;

  while (1)
    {
      enum dev_i2c_sched_state_e s = DEV_I2C_SCHED_IDLE;

      /* if a request is being processed. */
      if (sched->curr_rq != NULL)
        s = dev_i2c_ctrl_sched_exec(sched);

      /* otherwise, schedule next request if the queue is not empty. */
      else if (!dev_i2c_ctrl_queue_isempty(&sched->queue))
        s = dev_i2c_ctrl_sched_next(sched);

      switch (s)
        {
        default:
          assert(0 && "unknown scheduler state");
          break;

        case DEV_I2C_SCHED_CONTINUE:
          break;

        case DEV_I2C_SCHED_IDLE:
          sched->running = 0;
          lock_release_irq(&sched->lock);
          return;

        case DEV_I2C_SCHED_WAIT_TRANSFER:
          sched->running = 0;
          lock_release_irq(&sched->lock);
          /* if the kroutine is not executed, return from function. */
          if (kroutine_trigger(&sched->transfer.kr, 0) == 0)
            return;
          lock_spin_irq(&sched->lock);
          sched->running = 1;
          break;
        }
    }
}

/** @This schedules a new request, pushing it into the scheduler queue. If the
    request has a high priority, it is being pushed in the front of the queue.
    Otherwise, it is being pushed at the back of the queue. If the scheduler
    was not running, the push of a new request will restart it. */
void dev_i2c_request_sched(struct dev_i2c_ctrl_request_s *rq)
{
  struct dev_i2c_ctrl_sched_s *sched = rq->sched;

  lock_spin_irq(&sched->lock);

  /* prepare request. */
  rq->error    = 0;
  rq->enqueued = 1;

  //printk("i2c: thread %p schedules a new request.\n", pthread_self());

  bool_t empty = dev_i2c_ctrl_queue_isempty(&sched->queue);

  /* enqueue the request depending on its priority. */
  if (rq->priority)
    dev_i2c_ctrl_queue_push(&sched->queue, rq);
  else
    dev_i2c_ctrl_queue_pushback(&sched->queue, rq);

  /* restart the scheduler if it was not running with an active request. */
  if (sched->curr_rq == NULL && empty && !sched->running)
    dev_i2c_ctrl_sched_run(sched);
  else
    lock_release_irq(&sched->lock);
}

static
KROUTINE_EXEC(dev_i2c_ctrl_wait_rq_end)
{
  struct dev_i2c_dev_request_s  *rq     = KROUTINE_CONTAINER(kr, *rq, kr);
  struct dev_i2c_ctrl_wait_rq_s *status = rq->pvdata;

#if defined(CONFIG_MUTEK_SCHEDULER)
  lock_spin(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  lock_release(&status->lock);
#else
  status->done = 1;
#endif
}

ssize_t dev_i2c_request_helper(struct device_i2c_dev_s      *slave,
                               struct dev_i2c_dev_request_s *rq)
{
  struct dev_i2c_ctrl_wait_rq_s status;

  lock_init(&status.lock);
  status.ctx  = NULL;
  status.done = 0;

  kroutine_init(&rq->kr, &dev_i2c_ctrl_wait_rq_end, KROUTINE_IMMEDIATE);
  rq->pvdata = &status;

  rq->error = 0;

  size_t count = rq->size;

  /* push a request and wait for it to complete. Then kr is executed. */
  error_t err = DEVICE_OP(slave, request, rq);
  if (err)
    return err;

#if defined(CONFIG_MUTEK_SCHEDULER)
  /* ensure callback doesn't occur here */

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&status.lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      sched_stop_unlock(&status.lock);
    }
  else
    lock_release(&status.lock);

  CPU_INTERRUPT_RESTORESTATE;
  lock_destroy(&status.lock);

#else

# if defined(CONFIG_DEVICE_IRQ)
  assert(cpu_is_interruptible());
# endif

  while (!status.done)
    order_compiler_mem();

#endif

  assert (rq->error >= 0);
  return rq->error > 0 ? -rq->error : count - rq->size;
}

#endif // CONFIG_DEVICE_I2C_REQUEST


error_t dev_i2c_config(struct device_i2c_ctrl_s     *i2cdev,
                       struct dev_i2c_ctrl_config_s *cfg)
{
  return DEVICE_OP(i2cdev, config, cfg);
}

ssize_t dev_i2c_read(struct device_i2c_dev_s        *slave,
                     enum dev_i2c_ctrl_transfer_op_e op,
                     uint8_t                         *data,
                     size_t                          size)
{
#if defined(CONFIG_DEVICE_I2C_REQUEST)
  struct dev_i2c_dev_request_s rq = {
    .op   = op,
    .dir  = DEV_I2C_TR_READ,
    .data = data,
    .size = size
  };

  return dev_i2c_request_helper(slave, &rq);
#else
  return -ENOTSUP;
#endif
}

ssize_t dev_i2c_write(struct device_i2c_dev_s         *slave,
                      enum dev_i2c_ctrl_transfer_op_e op,
                      uint8_t const                   *data,
                      size_t                          size)
{
#if defined(CONFIG_DEVICE_I2C_REQUEST)
  struct dev_i2c_dev_request_s rq = {
    .op   = op,
    .dir  = DEV_I2C_TR_WRITE,
    .data = (uint8_t *)data,
    .size = size
  };

  return dev_i2c_request_helper(slave, &rq);
#else
  return -ENOTSUP;
#endif
}


/********************************** very low-level api. */

error_t dev_i2c_wait_scan(struct device_i2c_ctrl_s *i2cdev,
                          uint_fast16_t            saddr,
                          bool_t                   saddr_10_bits)
{
  ssize_t nbytes;

  if (saddr_10_bits)
    return -ENOTSUP;

  nbytes = dev_i2c_wait_write(
    i2cdev,
    DEV_I2C_OP_START_STOP,
    saddr & (saddr_10_bits ? 0xff : 0x7f),
    NULL,
    0
  );

  if (nbytes == -EAGAIN)
    return -EADDRNOTAVAIL;
  return nbytes;
}

error_t dev_i2c_spin_scan(struct device_i2c_ctrl_s *i2cdev,
                          uint_fast16_t            saddr,
                          bool_t                   saddr_10_bits)
{
  ssize_t nbytes;

  if (saddr_10_bits)
    return -ENOTSUP;

  nbytes = dev_i2c_spin_write(
    i2cdev,
    DEV_I2C_OP_START_STOP,
    saddr & (saddr_10_bits ? 0xff : 0x7f),
    NULL,
    0
  );

  if (nbytes == -EAGAIN)
    return -EADDRNOTAVAIL;
  return nbytes;
}

static
KROUTINE_EXEC(dev_i2c_ctrl_lock_transfer_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  status->done = 1;
}

static
KROUTINE_EXEC(dev_i2c_ctrl_lock_transfer_whole_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  if ( tr->count == 0 || tr->error )
    status->done = 1;
}

static ssize_t dev_i2c_lock_transfer(const struct device_i2c_ctrl_s *i2cdev,
                                     struct dev_i2c_ctrl_transfer_s *tr,
                                     kroutine_exec_t                *kr)
{
  struct dev_i2c_ctrl_wait_rq_s status;
  size_t                        size;

  /* setup the kroutine. */
  kroutine_init(&tr->kr, kr, KROUTINE_IMMEDIATE);
  tr->pvdata = &status;

  /* clear error. */
  tr->error = 0;

  /* setup the associated device. */
  tr->i2cdev = (struct device_i2c_ctrl_s *)i2cdev;

  status.done = 0;

  /* save requested byte count. */
  size = tr->count;

  /* launch the transfer and jump in the driver. */
  DEVICE_OP(i2cdev, transfer, tr);

#if defined(CONFIG_DEVICE_IRQ)
  assert(cpu_is_interruptible());
#endif

  while (!status.done)
    order_compiler_mem();

  assert (tr->error >= 0);
  return tr->error ? -tr->error : size - tr->count;
}

#if defined(CONFIG_MUTEK_SCHEDULER)

static
KROUTINE_EXEC(dev_i2c_ctrl_wait_transfer_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  lock_spin(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  lock_release(&status->lock);
}

static
KROUTINE_EXEC(dev_i2c_ctrl_wait_transfer_whole_end)
{
  struct dev_i2c_ctrl_transfer_s *tr     = KROUTINE_CONTAINER(kr, *tr, kr);
  struct dev_i2c_ctrl_wait_rq_s  *status = tr->pvdata;

  if (tr->count == 0 || tr->error)
    {
      lock_spin(&status->lock);
      if (status->ctx != NULL)
        sched_context_start(status->ctx);
      status->done = 1;
      lock_release(&status->lock);
    }
}

static ssize_t dev_i2c_wait_transfer(const struct device_i2c_ctrl_s *i2cdev,
                                     struct dev_i2c_ctrl_transfer_s *tr,
                                     kroutine_exec_t                *kr)
{
  struct dev_i2c_ctrl_wait_rq_s status;
  size_t                        size;

  /* setup the kroutine. */
  kroutine_init(&tr->kr, kr, KROUTINE_IMMEDIATE);
  tr->pvdata = &status;

  /* clear error. */
  tr->error = 0;

  /* setup associated device. */
  tr->i2cdev = (struct device_i2c_ctrl_s *)i2cdev;

  lock_init(&status.lock);
  status.ctx  = NULL;
  status.done = 0;

  /* save requested byte count. */
  size = tr->count;

  /* launch the transfer and jump in the driver. */
  DEVICE_OP(i2cdev, transfer, tr);

  /* ensure callback doesn't occur here */

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&status.lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      sched_stop_unlock(&status.lock);
    }
  else
    lock_release(&status.lock);

  CPU_INTERRUPT_RESTORESTATE;
  lock_destroy(&status.lock);

  assert (tr->error >= 0);
  return tr->error ? -tr->error : size - tr->count;
}

#endif

ssize_t dev_i2c_wait_read(const struct device_i2c_ctrl_s  *i2cdev,
                          enum dev_i2c_ctrl_transfer_op_e op,
                          uint8_t                         saddr,
                          uint8_t                         *data,
                          size_t                          size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .op     = op,
    .saddr  = saddr,
    .dir    = DEV_I2C_TR_READ,
    .count  = size,
    .data   = data,
  };

#if defined(CONFIG_MUTEK_SCHEDULER)
  return dev_i2c_wait_transfer(
    i2cdev,                         /* device. */
    &tr,                            /* transfer. */
    &dev_i2c_ctrl_wait_transfer_end /* kroutine. */
  );
#else
  return dev_i2c_lock_transfer(
    i2cdev,                         /* device. */
    &tr,                            /* transfer. */
    &dev_i2c_ctrl_lock_transfer_end /* kroutine. */
  );
#endif
}

ssize_t dev_i2c_spin_read(const struct device_i2c_ctrl_s  *i2cdev,
                          enum dev_i2c_ctrl_transfer_op_e op,
                          uint8_t                         saddr,
                          uint8_t                         *data,
                          size_t                          size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .op     = op,
    .saddr  = saddr,
    .dir    = DEV_I2C_TR_READ,
    .count  = size,
    .data   = data,
  };

  return dev_i2c_lock_transfer(
    i2cdev,                         /* device. */
    &tr,                            /* transfer. */
    &dev_i2c_ctrl_lock_transfer_end /* kroutine. */
  );
}

ssize_t dev_i2c_wait_write(const struct device_i2c_ctrl_s  *i2cdev,
                           enum dev_i2c_ctrl_transfer_op_e op,
                           uint8_t                         saddr,
                           const uint8_t                   *data,
                           size_t                          size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .op     = op,
    .saddr  = saddr,
    .dir    = DEV_I2C_TR_WRITE,
    .count  = size,
    .data   = (uint8_t*)data,
  };

#if defined(CONFIG_MUTEK_SCHEDULER)
  return dev_i2c_wait_transfer(
    i2cdev,                                 /* device. */
    &tr,                                    /* transfer. */
    &dev_i2c_ctrl_wait_transfer_whole_end   /* kroutine. */
  );
#else
  return dev_i2c_lock_transfer(
    i2cdev,                                 /* device. */
    &tr,                                    /* transfer. */
    &dev_i2c_ctrl_lock_transfer_whole_end   /* kroutine. */
  );
#endif
}

ssize_t dev_i2c_spin_write(const struct device_i2c_ctrl_s  *i2cdev,
                           enum dev_i2c_ctrl_transfer_op_e op,
                           uint8_t                         saddr,
                           const uint8_t                   *data,
                           size_t                          size)
{
  /* prepare the I2C transfer. */
  struct dev_i2c_ctrl_transfer_s tr =
  {
    .op     = op,
    .saddr  = saddr,
    .dir    = DEV_I2C_TR_WRITE,
    .count  = size,
    .data   = (uint8_t*)data,
  };

  return dev_i2c_lock_transfer(
    i2cdev,                                 /* device. */
    &tr,                                    /* transfer. */
    &dev_i2c_ctrl_lock_transfer_whole_end   /* kroutine. */
  );
}


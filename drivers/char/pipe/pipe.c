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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2015

*/

/*
  This driver implements two virtual char devices. Data written on one
  of the sub-device can be read on the other sub-device.

  When the CONFIG_DRIVER_CHAR_PIPE_FIFO token is defined, an
  additional integer array resource must be present which specifies
  the size of the buffer fifos for each direction.

  Here is a sample static char_pipe device declaration:

  DEV_DECLARE_STATIC(char_pipe_dev, "pipe", 0, char_pipe_drv,
                     DEV_STATIC_RES_UINT_ARRAY_PARAM("fifos", 128, 128)
                    );
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>

#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
#include <gct_platform.h>
#include <gct/container_dring.h>
#define GCT_CONTAINER_ALGO_pipe_fifo DRING
GCT_CONTAINER_TYPES(pipe_fifo, uint8_t, 0);
GCT_CONTAINER_FCNS(pipe_fifo, static inline, pipe_fifo,
                   init, destroy, storage,
                   isempty, isfull, pop_array, pushback_array);
#endif

DRIVER_PV(struct char_pipe_context_s
{
  dev_request_queue_root_t rq_q[2];
#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
  pipe_fifo_root_t fifo[2];
#endif
});


static DEV_CHAR_CANCEL(char_pipe_cancel)
{
  struct device_s *dev = accessor->dev;
  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);
  dev_request_queue_root_t *q = rq->base.drvdata;

  if (q != NULL)
    {
      dev_request_queue_remove(q, dev_char_rq_s_base(rq));
      rq->base.drvdata = NULL;
      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static bool_t char_pipe_rq2rq(struct device_s *dev,
                              struct dev_char_rq_s *rq0,
                              struct dev_char_rq_s *rq1,
                              dev_request_queue_root_t *q)
{
  while (rq1 != NULL)
    {
      bool_t done0, done1;

      if ((rq1->type | rq0->type) & _DEV_CHAR_POLL)
        {
          done0 = rq0->type & _DEV_CHAR_POLL;
          done1 = rq1->type & _DEV_CHAR_POLL;
        }
      else
        {
          size_t l = __MIN(rq0->size, rq1->size);
          assert(l > 0);

          if (rq0->type & _DEV_CHAR_WRITE)
            memcpy(rq1->data, rq0->data, l);
          else
            memcpy(rq0->data, rq1->data, l);
          rq0->size -= l;
          rq0->data += l;
          rq1->size -= l;
          rq1->data += l;
          done0 = !rq0->size || (rq0->type & (_DEV_CHAR_PARTIAL | _DEV_CHAR_NONBLOCK));
          done1 = !rq1->size || (rq1->type & (_DEV_CHAR_PARTIAL | _DEV_CHAR_NONBLOCK));
        }

      if (done1)
        {
          dev_request_queue_pop(q);
          rq1->base.drvdata = NULL;
          kroutine_exec(&rq1->base.kr);
          rq1 = dev_char_rq_s_cast(dev_request_queue_head(q));
        }

      if (done0)
        {
          kroutine_exec(&rq0->base.kr);
          return 0;
        }
    }

  return 1;
}

#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
static bool_t char_pipe_fifo2rq(struct device_s *dev,
                                struct dev_char_rq_s *rq,
                                pipe_fifo_root_t *fifo)
{
  if (!pipe_fifo_isempty(fifo))
    {
      if (rq->type & _DEV_CHAR_POLL)
        goto done;

      size_t r = pipe_fifo_pop_array(fifo, rq->data, rq->size);
      rq->data += r;
      rq->size -= r;

      if (!rq->size || (rq->type & (_DEV_CHAR_PARTIAL | _DEV_CHAR_NONBLOCK)))
        {
        done:
          kroutine_exec(&rq->base.kr);
          return 0;
        }
    }
  return 1;
}

static bool_t char_pipe_rq2fifo(struct device_s *dev,
                                struct dev_char_rq_s *rq,
                                pipe_fifo_root_t *fifo)
{
  if (!pipe_fifo_isfull(fifo))
    {
      if (rq->type & _DEV_CHAR_POLL)
        goto done;

      size_t r = pipe_fifo_pushback_array(fifo, rq->data, rq->size);
      rq->data += r;
      rq->size -= r;

      if (!rq->size || (rq->type & (_DEV_CHAR_PARTIAL | _DEV_CHAR_NONBLOCK)))
        {
        done:
          kroutine_exec(&rq->base.kr);
          return 0;
        }
    }
  return 1;
}
#endif

static DEV_CHAR_REQUEST(char_pipe_request)
{
  struct device_s *dev = accessor->dev;
  struct char_pipe_context_s *pv = dev->drv_pv;
  uint_fast8_t num = accessor->number & 1;
  error_t err = -ENOTSUP;
  bool_t done = 1;
  dev_request_queue_root_t *q;

  assert(rq->size);

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type & ~_DEV_CHAR_FLUSH)
    {
    case DEV_CHAR_READ_POLL:
      if (rq->size != 1)
        break;
    case DEV_CHAR_READ_PARTIAL:
    case DEV_CHAR_READ_NONBLOCK:
    case DEV_CHAR_READ: {
      q = &pv->rq_q[num ^ 0];
      struct dev_char_rq_s *wr_rq = dev_char_rq_s_cast(dev_request_queue_head(q));
      if (
#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
          char_pipe_fifo2rq(dev, rq, &pv->fifo[num ^ 0]) &&
#endif
          (wr_rq == NULL || !(wr_rq->type & _DEV_CHAR_WRITE) ||
           char_pipe_rq2rq(dev, rq, wr_rq, q)))
        goto blocking;
      done = 0;
      break;
    }

    case DEV_CHAR_WRITE_POLL:
      if (rq->size != 1)
        break;
    case DEV_CHAR_WRITE_PARTIAL:
    case DEV_CHAR_WRITE_NONBLOCK:
    case DEV_CHAR_WRITE: {
      q = &pv->rq_q[num ^ 1];
      struct dev_char_rq_s *rd_rq = dev_char_rq_s_cast(dev_request_queue_head(q));
      if ((rd_rq == NULL || (rd_rq->type & _DEV_CHAR_WRITE) ||
           char_pipe_rq2rq(dev, rq, rd_rq, q))
#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
          && char_pipe_rq2fifo(dev, rq, &pv->fifo[num ^ 1])
#endif
          )
        goto blocking;
      done = 0;
      break;
    }
    blocking:
      if (rq->type & _DEV_CHAR_NONBLOCK)
        {
          err = 0;
          break;
        }
      done = 0;
      dev_request_queue_pushback(q, dev_char_rq_s_base(rq));
      rq->base.drvdata = q;
    default:
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    {
      rq->error = err;
      kroutine_exec(&rq->base.kr);
    }
}

static DEV_USE(char_pipe_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      if (accessor->number > 1)
        return -ENOTSUP;
    }
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_LAST_NUMBER: {
      accessor->number = 1;
      return 0;
    }
    default:
      return -ENOTSUP;
    }
}

static DEV_INIT(char_pipe_init)
{
  struct char_pipe_context_s	*pv;
  size_t fsize = 0;


#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
  const uintptr_t *fsizes;
  uint16_t fcnt;
  if (device_get_param_uint_array(dev, "fifos", &fcnt, &fsizes) || fcnt != 2)
    return -EINVAL;
  if (!is_pow2(fsizes[0]) || !is_pow2(fsizes[1]))
    return -EINVAL;
  fsize = fsizes[0] + fsizes[1];
#endif

  pv = mem_alloc(sizeof(*pv) + fsize, mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  dev_request_queue_init(&pv->rq_q[0]);
  dev_request_queue_init(&pv->rq_q[1]);

#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
  pipe_fifo_init(&pv->fifo[0]);
  pipe_fifo_init(&pv->fifo[1]);

  uint8_t *fbuf0 = (uint8_t*)(pv + 1);
  uint8_t *fbuf1 = fbuf0 + fsizes[0];
  pipe_fifo_storage(&pv->fifo[0], fsizes[0], fbuf0);
  pipe_fifo_storage(&pv->fifo[1], fsizes[1], fbuf1);
#endif


  return 0;
}

static DEV_CLEANUP(char_pipe_cleanup)
{
  struct char_pipe_context_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->rq_q[0]) ||
      !dev_request_queue_isempty(&pv->rq_q[1]))
    return -EBUSY;

#ifdef CONFIG_DRIVER_CHAR_PIPE_FIFO
  pipe_fifo_destroy(&pv->fifo[0]);
  pipe_fifo_destroy(&pv->fifo[1]);
#endif

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(char_pipe_drv, 0, "Char pipe", char_pipe,
               DRIVER_CHAR_METHODS(char_pipe));

DRIVER_REGISTER(char_pipe_drv,
                DEV_ENUM_FDTNAME_ENTRY("char_pipe"));


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

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <hexo/iospace.h>

#include <device/class/char.h>
#include <mutek/console.h>
#include <mutek/kroutine.h>

struct console_printk_status_s
{
  char fifo[CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE];
  size_t use_begin, use_end, use_size;
  size_t free_begin, free_end, free_size;
  size_t pending_tx_size;
  struct lock_s lock;
  struct dev_char_rq_s char_rq;
  struct printk_backend_s backend;
  struct kroutine_s service;
};

STRUCT_COMPOSE(console_printk_status_s, backend)
  STRUCT_COMPOSE(console_printk_status_s, char_rq)
  STRUCT_COMPOSE(console_printk_status_s, service)

  static KROUTINE_EXEC(console_printk_service)
{
  struct console_printk_status_s *pv = console_printk_status_s_from_service(kr);
  size_t rptr;

  {
    LOCK_SPIN_IRQ_SCOPED(&pv->lock);

    if (pv->pending_tx_size)
      return;

    pv->pending_tx_size = __MIN(pv->use_size,
                                CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE - pv->use_begin);

    if (!pv->pending_tx_size)
      return;

    rptr = pv->use_begin;
    pv->use_size -= pv->pending_tx_size;
    pv->use_begin += pv->pending_tx_size;
    if (pv->use_begin == CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
      pv->use_begin = 0;

    pv->char_rq.data = (void*)(pv->fifo + rptr);
    pv->char_rq.size = pv->pending_tx_size;
  }
  
  DEVICE_OP(&console_dev, request, &pv->char_rq);
}

static KROUTINE_EXEC(console_printk_done)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct console_printk_status_s *pv = console_printk_status_s_from_char_rq(rq);

  {
    LOCK_SPIN_IRQ_SCOPED(&pv->lock);
    pv->free_size += pv->pending_tx_size;
    pv->free_end += pv->pending_tx_size;
    pv->pending_tx_size = 0;

    if (pv->free_begin >= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
      pv->free_begin -= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE;
  }
  
  kroutine_exec(&pv->service);
}

static PRINTK_HANDLER(console_printk_out)
{
  struct console_printk_status_s *pv =
    console_printk_status_s_from_backend(backend);

  if (!device_check_accessor(&console_dev.base))
    return;

  for (;;) {
    size_t wptr, copied;

    {
      LOCK_SPIN_IRQ_SCOPED(&pv->lock);
      copied = __MIN(__MIN(len, pv->free_size),
                     CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE - pv->free_begin);

      wptr = pv->free_begin;
      pv->free_size -= copied;
      pv->free_begin += copied;
      if (pv->free_begin == CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
        pv->free_begin = 0;
    }

    if (!copied)
      break;

    memcpy(pv->fifo + wptr, str, copied);
    str += copied;
    len -= copied;

    {
      LOCK_SPIN_IRQ_SCOPED(&pv->lock);
      pv->use_size += copied;
      pv->use_end += copied;
      if (pv->free_begin >= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
        pv->free_begin -= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE;
    }
  }

  kroutine_exec(&pv->service);
}

void console_printk_init(void)
{
  static struct console_printk_status_s status;

  kroutine_init_idle(&status.service, console_printk_service);
  
  lock_init(&status.lock);
  status.char_rq.type = DEV_CHAR_WRITE;
  dev_char_rq_init(&status.char_rq, console_printk_done);

  status.free_size = CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE;
  status.free_begin = 0;
  status.free_end = 0;
  status.use_size = 0;
  status.use_begin = 0;
  status.use_end = 0;

  printk_register(&status.backend, console_printk_out);
}

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
#include <hexo/iospace.h>

#include <device/class/char.h>
#include <mutek/console.h>

struct console_printk_status_s {
  char fifo[CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE];
  size_t use_begin, use_end, use_size;
  size_t free_begin, free_end, free_size;
  size_t pending_tx_size;
  struct lock_irq_s lock;
  struct dev_char_rq_s char_rq;
};

static void console_printk_rq_next(struct console_printk_status_s *pv)
{
  size_t rptr;

  assert(!pv->pending_tx_size);

  pv->pending_tx_size = __MIN(pv->use_size,
                 CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE - pv->use_begin);

  if (!pv->pending_tx_size)
    return;

  rptr = pv->use_begin;
  pv->use_size -= pv->pending_tx_size;
  pv->use_begin += pv->pending_tx_size;
  if (pv->use_begin == CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
    pv->use_begin = 0;
  lock_release_irq(&pv->lock);

  pv->char_rq.data = (void*)(pv->fifo + rptr);
  pv->char_rq.size = pv->pending_tx_size;

  DEVICE_OP(&console_dev, request, &pv->char_rq);
}

static KROUTINE_EXEC(console_printk_done)
{
  struct console_printk_status_s *pv = KROUTINE_CONTAINER(kr, *pv, char_rq.base.kr);
  
  lock_spin_irq(&pv->lock);
  pv->free_size += pv->pending_tx_size;
  pv->free_end += pv->pending_tx_size;
  if (pv->free_begin >= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
    pv->free_begin -= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE;

  pv->pending_tx_size = 0;
  console_printk_rq_next(pv);

  lock_release_irq(&pv->lock);
}

static PRINTF_OUTPUT_FUNC(console_printk_out)
{
  struct console_printk_status_s *pv = ctx;

  if (!device_check_accessor(&console_dev))
    return;

  for (;;) {
    size_t wptr, copied;

    lock_spin_irq(&pv->lock);
    copied = __MIN(__MIN(len, pv->free_size),
                   CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE - pv->free_begin);

    wptr = pv->free_begin;
    pv->free_size -= copied;
    pv->free_begin += copied;
    if (pv->free_begin == CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
      pv->free_begin = 0;
    lock_release_irq(&pv->lock);

    if (!copied)
      break;

    memcpy(pv->fifo + wptr, str, copied);
    str += copied;
    len -= copied;

    lock_spin_irq(&pv->lock);
    pv->use_size += copied;
    pv->use_end += copied;
    if (pv->free_begin >= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE)
      pv->free_begin -= CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE;

    if (!pv->pending_tx_size)
      console_printk_rq_next(pv);
    lock_release_irq(&pv->lock);
  }
}

void console_printk_init(void)
{
  static struct console_printk_status_s status;

  lock_init_irq(&status.lock);
  status.char_rq.type = DEV_CHAR_WRITE;
  kroutine_init_immediate(&status.char_rq.base.kr, console_printk_done);

  status.free_size = CONFIG_DRIVER_CONSOLE_PRINTK_BUFFER_SIZE;
  status.free_begin = 0;
  status.free_end = 0;
  status.use_size = 0;
  status.use_begin = 0;
  status.use_end = 0;

  printk_set_output(console_printk_out, &status);
}

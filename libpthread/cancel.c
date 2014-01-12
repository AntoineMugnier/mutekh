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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include "pthread_pv.h"


/** cancelation context linked list head */
CONTEXT_LOCAL struct __pthread_cleanup_s *__pthread_cleanup_list = NULL;

void pthread_testcancel(void)
{
  struct pthread_s	*this = pthread_self();

  LOCK_SPIN_IRQ(&this->lock);

  if ((this->state & _PTHREAD_STATE_CANCELED)
      && !(this->state & _PTHREAD_STATE_NOCANCEL))
    {
      /* call thread cleanup handlers */
      struct __pthread_cleanup_s *c;
      for (c = CONTEXT_LOCAL_GET(__pthread_cleanup_list); c; c = c->prev)
        c->fcn(c->arg);

      return pthread_exit(PTHREAD_CANCELED);
    }

  LOCK_RELEASE_IRQ(&this->lock);
}

error_t pthread_cancel(pthread_t thread)
{
  struct pthread_s	*this = pthread_self();
  error_t err = 0;

  LOCK_SPIN_IRQ(&this->lock);

  if (thread->state & _PTHREAD_STATE_CANCELED)
    err = EBUSY;
  else
    thread->state |= _PTHREAD_STATE_CANCELED;

  LOCK_RELEASE_IRQ(&this->lock);

  return err;
}

error_t
pthread_setcancelstate(int_fast8_t state, int_fast8_t *oldstate)
{
  struct pthread_s	*this = pthread_self();

  LOCK_SPIN_IRQ(&this->lock);

  if (oldstate)
    *oldstate = this->state & _PTHREAD_STATE_NOCANCEL
      ? PTHREAD_CANCEL_DISABLE
      : PTHREAD_CANCEL_ENABLE;

  if (state == PTHREAD_CANCEL_ENABLE)
    this->state &= ~_PTHREAD_STATE_NOCANCEL;
  else
    this->state |= _PTHREAD_STATE_NOCANCEL;

  LOCK_RELEASE_IRQ(&this->lock);

  return 0;
}

error_t
pthread_setcanceltype(int_fast8_t type, int_fast8_t *oldtype)
{
  struct pthread_s	*this = pthread_self();

  LOCK_SPIN_IRQ(&this->lock);

  if (oldtype)
    *oldtype = this->state & _PTHREAD_STATE_CANCELASYNC
      ? PTHREAD_CANCEL_ASYNCHRONOUS
      : PTHREAD_CANCEL_DEFERRED;

  if (type == PTHREAD_CANCEL_DEFERRED)
    this->state &= ~_PTHREAD_STATE_CANCELASYNC;
  else
    this->state |= _PTHREAD_STATE_CANCELASYNC;

  LOCK_RELEASE_IRQ(&this->lock);

  return 0;
}


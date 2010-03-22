/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef _PTHREAD_PV_H_
#define _PTHREAD_PV_H_

#include <pthread.h>

#ifdef CONFIG_PTHREAD_CANCEL

void
__pthread_switch(void);

void
__pthread_cleanup(void);

void
__pthread_cancel_self(void);

#if 0
static inline void
__pthread_testcancel_async(void)
{
  pthread_t self = pthread_self();

  if (!self->canceled && self->cancelasync)
    __pthread_cancel_self();
}
#endif

#endif

#endif


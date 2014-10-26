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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <device/request.h>

GCT_CONTAINER_PROTOTYPES(dev_request_queue, extern inline, dev_request_queue,
                        init, destroy, pushback, pop, isempty, head);

GCT_CONTAINER_PROTOTYPES(dev_request_pqueue, extern inline, dev_request_pqueue,
                        init, destroy, pop, isempty, head);

extern inline KROUTINE_EXEC(dev_request_spin_done);

extern inline KROUTINE_EXEC(dev_request_spin_done);

extern inline void
dev_request_spin_init(struct dev_request_s *rq,
                      struct dev_request_status_s *status);

extern inline void
dev_request_spin_wait(struct dev_request_status_s *status);

#ifdef CONFIG_MUTEK_SCHEDULER

extern inline KROUTINE_EXEC(dev_request_sched_done);

extern inline void
dev_request_sched_init(struct dev_request_s *rq,
                       struct dev_request_status_s *status);

extern inline void
dev_request_sched_wait(struct dev_request_status_s *status);

#endif


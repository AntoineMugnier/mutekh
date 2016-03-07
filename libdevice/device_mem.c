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

    Synchronous read and write functions for mem device.

*/

#include <device/class/mem.h>

#include <alloca.h>
#include <enums.h>

const char dev_mem_type_e[] = ENUM_DESC_DEV_MEM_TYPE_E;
const char dev_mem_flags_e[] = ENUM_DESC_DEV_MEM_FLAGS_E;
const char dev_mem_rq_type_e[] = ENUM_DESC_DEV_MEM_RQ_TYPE_E;

# ifdef CONFIG_MUTEK_CONTEXT_SCHED
extern inline error_t
dev_mem_wait_op(struct device_mem_s *accessor,
                struct dev_mem_rq_s *rq);
#endif

extern inline error_t
dev_mem_spin_op(struct device_mem_s *accessor,
                struct dev_mem_rq_s *rq);

void dev_mem_mapped_op_helper(uintptr_t base, uint_fast8_t page_log2, struct dev_mem_rq_s *rq)
{
  size_t s = 1 << (rq->sc_log2 + page_log2);
  size_t i;
  
  switch (rq->type & (DEV_MEM_OP_PARTIAL_READ | DEV_MEM_OP_PARTIAL_WRITE |
                      DEV_MEM_OP_PAGE_READ | DEV_MEM_OP_PAGE_WRITE))
    {
    case DEV_MEM_OP_PARTIAL_READ:
      memcpy(rq->data, (void*)(uintptr_t)(base + rq->addr), rq->size);
      break;
    case DEV_MEM_OP_PARTIAL_WRITE:
      memcpy((void*)(uintptr_t)(base + rq->addr), rq->data, rq->size);
      break;
    case DEV_MEM_OP_PAGE_READ: {
      for (i = 0; i < (rq->size >> rq->sc_log2); i++)
        memcpy(rq->sc_data[i], (void*)(uintptr_t)(base + rq->addr + s * i), s);
      break;
    }
    case DEV_MEM_OP_PAGE_WRITE:
      for (i = 0; i < (rq->size >> rq->sc_log2); i++)
        memcpy((void*)(uintptr_t)(base + rq->addr + s * i), rq->sc_data[i], s);
      break;
    }
}


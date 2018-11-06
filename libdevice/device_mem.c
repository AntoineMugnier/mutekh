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
#include <hexo/enum.h>
#include <hexo/flash.h>

const char dev_mem_type_e[] = ENUM_DESC_DEV_MEM_TYPE_E;
const char dev_mem_flags_e[] = ENUM_DESC_DEV_MEM_FLAGS_E;

# ifdef CONFIG_MUTEK_CONTEXT_SCHED
extern inline error_t
dev_mem_wait_rq(const struct device_mem_s *accessor,
                struct dev_mem_rq_s *rq);
#endif

error_t dev_mem_mapped_op_helper(uintptr_t base, uintptr_t end,
                                 struct dev_mem_rq_s * __restrict__ rq)
{
  size_t i;

  enum dev_mem_rq_type_e type = rq->type & ~(_DEV_MEM_FLUSH | _DEV_MEM_INVAL);

  if (type & _DEV_MEM_PARTIAL)
    {
      uintptr_t a = base + rq->partial.addr;
      uintptr_t s = rq->partial.size;
      if (end - a < s)
        return -ERANGE;

      switch (type & _DEV_MEM_OP_MASK)
        {
        case _DEV_MEM_READ:
          memcpy(rq->partial.data, (void*)a, s);
          return 0;

        case _DEV_MEM_WRITE:
          memcpy((void*)a, rq->partial.data, s);
          return 0;
        }
    }
  else if (type & _DEV_MEM_PAGE)
    {
      uintptr_t s = 1 << rq->page.page_log2;

      switch (type & _DEV_MEM_OP_MASK)
        {
        case _DEV_MEM_READ:
          for (i = 0; i < rq->page.sc_count; i++)
            {
              uintptr_t a = base + rq->page.sc[i].addr;
              if (end - a < s)
                return -ERANGE;
              memcpy(rq->page.sc[i].data, (void*)a, s);
            }
          return 0;

        case _DEV_MEM_WRITE:
          for (i = 0; i < rq->page.sc_count; i++)
            {
              uintptr_t a = base + rq->page.sc[i].addr;
              if (end - a < s)
                return -ERANGE;
              memcpy((void*)a, rq->page.sc[i].data, s);
            }
          return 0;
        }
    }

  return -ENOTSUP;
}

error_t dev_mem_flash_op(uintptr_t base, uintptr_t end,
                         uint_fast8_t page_log2,
                         struct dev_mem_rq_s * __restrict__ rq)
{
  enum dev_mem_rq_type_e type = rq->type & ~(_DEV_MEM_FLUSH | _DEV_MEM_INVAL);
  size_t ps = 1 << page_log2;

  if (type & _DEV_MEM_PAGE)
    {
      if (rq->page.page_log2 < page_log2)
        return -ERANGE;

      uint_fast16_t pcnt = 1 << (rq->page.page_log2 - page_log2);
      size_t i, j, scc = rq->page.sc_count;

      for (i = 0; i < scc; i++)
        {
          uint8_t *data = rq->page.sc[i].data;
          uintptr_t addr = base + rq->page.sc[i].addr;

          for (j = 0; j < pcnt; j++)
            {
              if (end - addr < ps)
                return -ERANGE;

              error_t err;
              switch (type & _DEV_MEM_OP_MASK)
                {
                case _DEV_MEM_READ:
                  memcpy(data, (void*)addr, ps);
                  err = 0;
                  break;

                case _DEV_MEM_ERASE | _DEV_MEM_WRITE:
                  err = flash_page_erase(addr);
                  if (err)
                    break;
                  // fallthrough

                case _DEV_MEM_WRITE:
                  err = flash_page_write(addr, (void*)data, ps);
                  break;

                case _DEV_MEM_ERASE:
                  err = flash_page_erase(addr);
                  break;

                default:
                  UNREACHABLE();
                }

              if (err)
                return -EIO;
              data += ps;
              addr += ps;
            }
        }
      return 0;
    }
  else if (type & _DEV_MEM_PARTIAL)
    {
      uintptr_t a = base + rq->partial.addr;
      uintptr_t s = rq->partial.size;

      if ((a | s) & 3)
        return -ERANGE;
      if (end - a < s)
        return -ERANGE;

      switch (type & _DEV_MEM_OP_MASK)
        {
        case _DEV_MEM_READ:
          memcpy(rq->partial.data, (void*)a, s);
          return 0;

        case _DEV_MEM_WRITE: {
          if ((a & (ps - 1)) + s > ps)
            return -ERANGE;   // cross page boundary

          if (s > 0 &&
              flash_page_write(a, rq->partial.data, s))
            return -EIO;
          return 0;
        }

        default:
          UNREACHABLE();
        }
    }

  return -ENOTSUP;
}

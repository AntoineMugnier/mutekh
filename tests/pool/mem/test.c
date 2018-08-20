
#include <device/class/iomux.h>
#include <device/resources.h>
#include <device/class/gpio.h>

static size_t msize = 0;     /* use this amount of pages for the test when not 0 */
static size_t moffset = 0;     /* skip this amount of pages */
static size_t ml2psize = 8;      /* force log 2 of page size to be at least this */
static const char *devpath = "mem";

#include <device/class/mem.h>
#include <mutek/startup.h>
#include <mutek/thread.h>
#include <mutek/printk.h>

#define PCOUNT 8

struct page_s
{
  uint64_t addr;
  uint8_t *data;
  bool_t dirty;
};

static struct page_s p[PCOUNT];
static size_t pl2size;          /* log 2 of page size in bytes */
static size_t psize;            /* page size in bytes */
static size_t dsize;            /* device size in pages */
static size_t cpsize;           /* page size for cross page partial access */

#warning test DEV_MEM_MAPPED_READ DEV_MEM_MAPPED_WRITE

static uint64_t pick_page_addr()
{
  uint64_t a;
 again:
  a = (moffset + rand() % (dsize - 1)) * psize;
  for (uint_fast8_t j = 0; j < PCOUNT; j++)
    if (p[j].data && (p[j].addr == a || p[j].addr == a + psize || p[j].addr + psize == a))
      goto again;
  return a;
}

static bool_t memcmp_verbose(const void *a_, const void *b_, size_t s)
{
  const uint8_t *a = a_, *b = b_;
  size_t i;

  for (i = 0; i < s; i++)
    if (a[i] != b[i])
      {
        printk("cmp fail at %u: %02x != %02x\n", i, a[i], b[i]);
        abort();
        return 1;
      }
  return 0;
}

static size_t change_data(uint8_t *d, size_t size, enum dev_mem_flags_e flags)
{
  size_t c = 0;
  for (size_t o = 0; o < size; o++)
    {
      if (flags & DEV_MEM_ERASE_ONE)
        {
          c += bit_popc8(~d[o]);
          d[o] &= rand();
        }
      else if (flags & DEV_MEM_ERASE_ZERO)
        {
          c += bit_popc8(d[o]);
          d[o] |= rand();
        }
      else
        {
          d[o] = rand();
        }
    }
  return c;
}

static CONTEXT_ENTRY(thread)
{
  struct device_mem_s mem_dev;

  ensure(!device_get_accessor_by_path(&mem_dev.base, NULL, devpath, DRIVER_CLASS_MEM));

  struct dev_mem_info_s info;
  struct dev_mem_rq_s rq;

  ensure(!DEVICE_OP(&mem_dev, info, &info, 0));

  pl2size = __MAX(ml2psize, __MAX(info.page_log2, info.erase_log2));

  if (!msize)
    dsize = ((info.size << info.page_log2) >> pl2size) - moffset;
  else
    dsize = msize;

  cpsize = 1 << info.page_log2;
  psize = 1 << pl2size;

  printk("page size %u\nmem size %u\n", psize, dsize);

  assert(dsize != 0);

  for (uint_fast8_t i = 0; i < PCOUNT; i++)
    p[i].data = NULL;

  uint8_t *ptmp = malloc(psize * 2);

  while (1)
    {
      uint_fast8_t i = rand() % PCOUNT;

      if (!p[i].data)
        {
          p[i].data = malloc(psize * 2);

          if (info.flags & DEV_MEM_PAGE_READ)
            goto page_init;
          else if (info.flags & DEV_MEM_PARTIAL_READ)
            goto partial_init;
          else
            abort();
        }

      uint_fast8_t op = rand() % 32;

      memset(ptmp, 0x55, psize * 2);

      switch (op)
        {
        case 0:
          if (p[i].dirty)
            break;
          if (info.flags & DEV_MEM_PARTIAL_READ)
            {
            partial_init:
              p[i].addr = pick_page_addr();
              p[i].dirty = 0;

              rq.type = DEV_MEM_OP_PARTIAL_READ;
              rq.band_mask = 1;
              rq.partial.addr = p[i].addr;
              rq.partial.data = p[i].data;
              rq.partial.size = psize;

              printk("%2u:part init  a:%llx sz:%zx", i,
                     rq.partial.addr, rq.partial.size);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));

              rq.partial.addr = p[i].addr + psize;
              rq.partial.data = p[i].data + psize;
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));

              printk(" d:%P\n", p[i].data, __MIN(16, rq.partial.size));
            }
          break;

        case 1:
          if (p[i].dirty)
            break;
          if (info.flags & DEV_MEM_PAGE_READ)
            {
            page_init:
              p[i].addr = pick_page_addr();
              p[i].dirty = 0;

              struct dev_mem_page_sc_s sc;
              sc.addr = p[i].addr;
              sc.data = p[i].data;

              rq.type = DEV_MEM_OP_PAGE_READ;
              rq.band_mask = 1;
              rq.page.page_log2 = pl2size + 1;
              rq.page.sc = &sc;
              rq.page.sc_count = 1;

              printk("%2u:page init  a:%llx pl2:%u scc:%u", i,
                     sc.addr, rq.page.page_log2, rq.page.sc_count);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
              printk(" d:%P ... %P ...\n",
                     p[i].data, __MIN(16, psize * 2),
                     p[i].data + psize, __MIN(16, psize * 2));
            }
          break;

        case 2 ... 7:
          if (info.flags & DEV_MEM_PAGE_READ)
            {
              struct dev_mem_page_sc_s sc[2];
              sc[0].addr = p[i].addr;
              sc[0].data = ptmp;
              sc[1].addr = p[i].addr + psize;
              sc[1].data = ptmp + psize;

              rq.type = DEV_MEM_OP_PAGE_READ;
              rq.band_mask = 1;
              rq.page.page_log2 = pl2size;
              rq.page.sc = sc;
              rq.page.sc_count = 2;

              printk("%2u:page read  a:%llx pl2:%u scc:%u", i,
                     sc[0].addr, rq.page.page_log2, rq.page.sc_count);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
              printk(" d:%P ... %P ...\n",
                     ptmp, __MIN(16, psize * 2),
                     ptmp + psize, __MIN(16, psize * 2));

              ensure(!memcmp_verbose(ptmp, p[i].data, psize * 2));
              p[i].dirty = 0;
            }
          break;

        case 8 ... 15:
          if (info.flags & DEV_MEM_PAGE_READ)
            {
              struct dev_mem_page_sc_s sc;
              sc.addr = p[i].addr;
              sc.data = ptmp;

              rq.type = DEV_MEM_OP_PAGE_READ;
              rq.band_mask = 1;
              rq.page.page_log2 = pl2size + 1;
              rq.page.sc = &sc;
              rq.page.sc_count = 1;

              printk("%2u:page read  a:%llx pl2:%u scc:%u", i,
                     sc.addr, rq.page.page_log2, rq.page.sc_count);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
              printk(" d:%P ... %P ...\n",
                     ptmp, __MIN(16, psize * 2),
                     ptmp + psize, __MIN(16, psize * 2));

              ensure(!memcmp_verbose(ptmp, p[i].data, psize * 2));
              p[i].dirty = 0;
            }
          else
            {
              rq.band_mask = 1;

              rq.type = DEV_MEM_OP_PARTIAL_READ;
              rq.band_mask = 1;
              rq.partial.addr = p[i].addr;
              rq.partial.data = ptmp;
              rq.partial.size = psize * 2;

              printk("%2u:part check a:%llx sz:%zx", i,
                     rq.partial.addr, rq.partial.size);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
              printk(" d:%P\n", ptmp, __MIN(16, rq.partial.size));

              ensure(!memcmp_verbose(ptmp, p[i].data, psize * 2));
              p[i].dirty = 0;
            }
          break;

        case 16 ... 24:
          if (info.flags & DEV_MEM_PARTIAL_READ)
            {
              uint32_t o = cpsize > 1
                ? rand() % (cpsize / 2)
                : rand() % (__MIN(256, dsize - p[i].addr) / 2);

              o &= ~bit_mask(0, info.partial_log2);

              uint32_t s = (cpsize > 1) && (info.flags & DEV_MEM_CROSS_READ)
                ? (cpsize - o) * 2 : o;

              rq.type = DEV_MEM_OP_PARTIAL_READ;
              rq.band_mask = 1;
              rq.partial.addr = p[i].addr + o;
              rq.partial.data = ptmp;
              rq.partial.size = s;

              printk("%2u:part read  a:%llx sz:%zx", i,
                     rq.partial.addr, rq.partial.size);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
              printk(" d:%P\n",
                     ptmp, __MIN(16, rq.partial.size));
              ensure(!memcmp_verbose(ptmp, p[i].data + o, s));
            }
          break;

        case 25:
        case 26:
        case 27:
          if (p[i].dirty)
            break;
          if (info.flags & DEV_MEM_PARTIAL_WRITE)
            {
              uint32_t o = cpsize > 1
                ? rand() % (cpsize / 2)
                : rand() % (__MIN(256, dsize - p[i].addr) / 2);

              o &= ~bit_mask(0, info.partial_log2);

              uint32_t s = (cpsize > 1) && (info.flags & DEV_MEM_CROSS_WRITE)
                ? (cpsize - o) * 2 : o;

              size_t c = change_data(p[i].data + o, s, info.flags);

              if (s > 32 && c > s * 7) /* to many written bits ? */
                {
                  printk("%2u:force erase: %P\n", i, p[i].data + o, __MIN(16, s));
                  goto erase_page;
                }

              p[i].dirty = 1;

              rq.type = DEV_MEM_OP_PARTIAL_WRITE;
              rq.band_mask = 1;
              rq.partial.addr = p[i].addr + o;
              rq.partial.data = p[i].data + o;
              rq.partial.size = s;

              printk("%2u:part write  a:%llx sz:%zx", i,
                     rq.partial.addr, rq.partial.size);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
              printk(" d:%P\n",
                     p[i].data + o, __MIN(16, rq.partial.size));
            }
          break;

        case 28:
        case 29:
        case 30:
          if (p[i].dirty)
            break;
          if (info.flags & DEV_MEM_PAGE_WRITE)
            {
              size_t c = change_data(p[i].data, psize * 2, info.flags);

              if (c > psize * 2 * 7) /* to many written bits ? */
                {
                  printk("%2u:force erase: %P\n", i, p[i].data, __MIN(16, psize * 2));
                  goto erase_page;
                }

              p[i].dirty = 1;

              struct dev_mem_page_sc_s sc;
              rq.page.page_log2 = pl2size + 1;
              rq.page.sc = &sc;
              rq.page.sc_count = 1;

              sc.addr = p[i].addr;
              sc.data = p[i].data;
              rq.type = DEV_MEM_OP_PAGE_WRITE;

              printk("%2u:page write a:%llx pl2:%u scc:%u d:%P ... %P ...\n", i,
                     sc.addr, rq.page.page_log2, rq.page.sc_count,
                     sc.data, __MIN(16, psize * 2),
                     sc.data + psize, __MIN(16, psize * 2));
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
            }
          break;

        case 31:
          if (p[i].dirty)
            break;
          if (info.flags & (DEV_MEM_ERASE_ONE | DEV_MEM_ERASE_ZERO))
            {
            erase_page:;
              p[i].dirty = 1;

              struct dev_mem_page_sc_s sc;
              rq.page.page_log2 = pl2size + 1;
              rq.page.sc = &sc;
              rq.page.sc_count = 1;

              sc.addr = p[i].addr;
              sc.data = NULL;
              rq.type = DEV_MEM_OP_PAGE_ERASE;

              if (info.flags & DEV_MEM_ERASE_ONE)
                memset(p[i].data, 0xff, psize * 2);
              else
                memset(p[i].data, 0, psize * 2);

              printk("%2u:page erase a:%llx pl2:%u scc:%u\n", i,
                     sc.addr, rq.page.page_log2, rq.page.sc_count);
              ensure(!dev_mem_wait_rq(&mem_dev, &rq));
            }
          break;
        }

    }

}

void app_start()
{
  thread_create(thread, NULL, NULL);
}

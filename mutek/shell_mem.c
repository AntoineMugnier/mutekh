
#include <hexo/iospace.h>
#include <hexo/flash.h>

#include <mutek/shell.h>
#include <stdlib.h>

TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_blocks);

#ifdef CONFIG_MUTEK_MEMALLOC_STATS
TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_stats);
#endif

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_alloc)
{
  uint32_t size = strto_uintl32(argv[0], NULL, 0);
  termui_con_printf(con, "%p\n", malloc(size));
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_hexdump)
{
  uintptr_t addr = strto_uintl32(argv[0], NULL, 0);
  size_t size = strto_uintl32(argv[1], NULL, 0);

  while (size)
    {
      size_t s = __MIN(16,size);
      termui_con_printf(con, "%p: %P\n", addr, addr, s);
      size -= s;
      addr += s;
    }

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_fill)
{
  uintptr_t addr = strto_uintl32(argv[0], NULL, 0);
  size_t size = strto_uintl32(argv[1], NULL, 0);
  uint8_t val = argc > 2 ? strto_uintl32(argv[2], NULL, 0) : 0;
  uint8_t inc = argc > 3 ? strto_uintl32(argv[3], NULL, 0) : 0;

  while (size--)
    {
      cpu_mem_write_8(addr, val);
      addr++;
      val += inc;
    }
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_w32)
{
  cpu_mem_write_32(strto_uintl32(argv[0], NULL, 0),
                   strto_uintl32(argv[1], NULL, 0));
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_r32)
{
  uint32_t x = cpu_mem_read_32(strto_uintl32(argv[0], NULL, 0));

  termui_con_printf(con, "%08x\n", x);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_w8)
{
  cpu_mem_write_8(strto_uintl32(argv[0], NULL, 0),
                   strto_uintl32(argv[1], NULL, 0));
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_r8)
{
  uint32_t x = cpu_mem_read_8(strto_uintl32(argv[0], NULL, 0));

  termui_con_printf(con, "%02x\n", x);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_flash_erase)
{
  return flash_page_erase(strto_uintl32(argv[0], NULL, 0))
    ? -EINVAL : 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_flash_write)
{
  return flash_page_write(strto_uintl32(argv[0], NULL, 0),
                          (void*)argv[1], argl[1])
    ? -EINVAL : 0;
}

static TERMUI_CON_GROUP_DECL(shell_mem_subgroup) =
{
#ifdef CONFIG_MUTEK_MEMALLOC_SMART
  TERMUI_CON_ENTRY(shell_mem_blocks, "blocks",
  )

# ifdef CONFIG_MUTEK_MEMALLOC_STATS
  TERMUI_CON_ENTRY(shell_mem_stats, "stats",
  )
# endif
#endif

  TERMUI_CON_ENTRY(shell_mem_alloc, "alloc",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_mem_hexdump, "hexdump",
    TERMUI_CON_ARGS(2, 2)
  )

  TERMUI_CON_ENTRY(shell_mem_fill, "fill",
    TERMUI_CON_ARGS(2, 4)
  )

  TERMUI_CON_ENTRY(shell_mem_w32, "w32",
    TERMUI_CON_ARGS(2, 2)
  )

  TERMUI_CON_ENTRY(shell_mem_r32, "r32",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_mem_w8, "w8",
    TERMUI_CON_ARGS(2, 2)
  )

  TERMUI_CON_ENTRY(shell_mem_r8, "r8",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_mem_flash_erase, "flash_erase",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_mem_flash_write, "flash_write",
    TERMUI_CON_ARGS(2, 2)
  )

  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(shell_mem_subgroup, "mem")

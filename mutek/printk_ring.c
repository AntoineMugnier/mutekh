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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2018
*/

#include <mutek/printk.h>
#include <mutek/startup.h>
#include <mutek/shell.h>
#include <termui/term.h>

#include <stdlib.h>

static char printk_ring[CONFIG_MUTEK_PRINTK_RING_SIZE];
static uint32_t printk_ring_size;
static uint32_t printk_ring_ptr;

static PRINTK_HANDLER(printk_ring_out)
{
  /* skip head of string if longer than buffer */
  if (len > CONFIG_MUTEK_PRINTK_RING_SIZE)
    {
      str += len - CONFIG_MUTEK_PRINTK_RING_SIZE;
      len = CONFIG_MUTEK_PRINTK_RING_SIZE;
    }

  uint32_t end = (printk_ring_ptr + printk_ring_size)
                 % CONFIG_MUTEK_PRINTK_RING_SIZE;

  if (len + printk_ring_size >= CONFIG_MUTEK_PRINTK_RING_SIZE)
    {
      /* drop buffer head */
      printk_ring_ptr = (printk_ring_ptr + len + printk_ring_size) % CONFIG_MUTEK_PRINTK_RING_SIZE;
      printk_ring_size = CONFIG_MUTEK_PRINTK_RING_SIZE;
    }
  else
    {
      printk_ring_size += len;
    }

  /* write tail of string at ring[0] if overflow */
  uint32_t l = CONFIG_MUTEK_PRINTK_RING_SIZE - end;
  if (len > l)
    {
      memcpy(printk_ring, str + l, len - l);
      len = l;
    }

  /* write head of string */
  memcpy(printk_ring + end, str, len);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_logk_tail)
{
  uint_fast16_t lines = argc ? atoi(argv[0]) : 5;

  if (printk_ring_size == 0)
    return 0;

  uint32_t end = printk_ring_ptr + printk_ring_size;
  uint32_t lptr = end, ptr = end;

  /* look backward for end of line */
  while (lines > 0)
    {
      if (ptr == printk_ring_ptr)
        {
          lptr = ptr;
          break;
        }

      if (printk_ring[(ptr + CONFIG_MUTEK_PRINTK_RING_SIZE - 1)
                      % CONFIG_MUTEK_PRINTK_RING_SIZE] == '\n')
        {
          lptr = ptr;
          lines--;
        }

      ptr--;
    }

  /* display buffer tail */
  ptr = lptr % CONFIG_MUTEK_PRINTK_RING_SIZE;

  if (end % CONFIG_MUTEK_PRINTK_RING_SIZE <= ptr)
    {
      termui_con_putsl(con, printk_ring + lptr % CONFIG_MUTEK_PRINTK_RING_SIZE,
                       CONFIG_MUTEK_PRINTK_RING_SIZE - ptr);
      termui_con_putsl(con, printk_ring, end % CONFIG_MUTEK_PRINTK_RING_SIZE);
    }
  else
    {
      termui_con_putsl(con, printk_ring + ptr, end - lptr);
    }

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_logk_write)
{
  writek(argv[0], argl[0]);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_logk_clear)
{
  printk_ring_size = 0;
  printk_ring_ptr = 0;
  return 0;
}

static TERMUI_CON_GROUP_DECL(shell_logk_subgroup) =
{
  TERMUI_CON_ENTRY(shell_logk_tail, "tail",
    TERMUI_CON_ARGS(0, 1)
  )

  TERMUI_CON_ENTRY(shell_logk_write, "write",
    TERMUI_CON_ARGS(1, 1)
  )

  TERMUI_CON_ENTRY(shell_logk_clear, "clear",
  )

  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(shell_logk_subgroup, "logk");

void printk_ring_init()
{
  printk_ring_size = 0;
  printk_ring_ptr = 0;

  static struct printk_backend_s printk_ring_backend;
  printk_register(&printk_ring_backend, printk_ring_out);
}

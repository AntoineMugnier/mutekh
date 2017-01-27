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
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#include <hexo/types.h>
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <mutek/printk.h>
#include <mutek/startup.h>

static PRINTK_HANDLER(soclib_printk_out)
{
  size_t i;
  for (i = 0; i < len; ++i)
    cpu_mem_write_8(CONFIG_MUTEK_PRINTK_ADDR, str[i]);
}

void soclib_printk_init()
{
  static struct printk_backend_s backend;
  printk_register(&backend, soclib_printk_out);
}


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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/

#include <hexo/interrupt.h>
#include <device/class/char.h>

#include <mutek/printk.h>
#include <mutek/fileops.h>
#include <mutek/startup.h>

#include <mutek/console.h>
#include <device/class/char.h>
#include <device/device.h>
#include <device/driver.h>

#ifdef CONFIG_MUTEK_CONTEXT_SCHED

static FILEOPS_READ(tty_read)
{
#if defined(CONFIG_MUTEK_CONSOLE)
  return dev_char_wait_op(&console_dev, DEV_CHAR_READ_PARTIAL, (uint8_t*)buffer, count);
#else
  return 0;
#endif
}

static FILEOPS_WRITE(tty_write)
{
#if defined(CONFIG_MUTEK_CONSOLE)
  return dev_char_wait_op(&console_dev, DEV_CHAR_WRITE_PARTIAL, (uint8_t*)buffer, count);
#else
  return count;
#endif
}

const struct fileops_s console_file_ops =
{
  .read = &tty_read,
  .write = &tty_write,
};

#endif

struct device_char_s console_dev = DEVICE_ACCESSOR_INIT;

void mutek_console_initsmp(void)
{
  if (!cpu_isbootstrap())
    return;

  if (device_get_accessor_by_path(&console_dev, NULL,
                                  CONFIG_MUTEK_CONSOLE_DEVICE_PATHS,
                                  DRIVER_CLASS_CHAR))
    printk("error: mutek console: No initialized device found matching `"
           CONFIG_MUTEK_CONSOLE_DEVICE_PATHS "' in the device tree.\n");
}

void mutek_console_cleanupsmp(void)
{
  if (!cpu_isbootstrap())
    return;

  if (device_check_accessor(&console_dev))
    device_put_accessor(&console_dev);
}


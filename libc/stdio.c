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

#include <string.h>

#include <hexo/types.h>

#include <hexo/device/char.h>
#include <hexo/device.h>
#include <hexo/driver.h>

extern struct device_s *tty_dev;

//lock_t stdio_lock	= LOCK_INITIALIZER;

void __puts(const char *s, size_t len)
{
  //  lock_spin(&stdio_lock);

  while (len > 0)
    {
      ssize_t	res;

      res = dev_char_write(tty_dev, (uint8_t*)s, len);

      if (res > 0)
	{
	  len -= res;
	  s += res;
	  /* pthread_yield(); */
	}
    }

  //  lock_release(&stdio_lock);
}

inline int_fast8_t putchar(char c)
{
  while (dev_char_write(tty_dev, (uint8_t*)&c, 1) != 1)
    /* pthread_yield() */ ;

  return c;
}

int_fast8_t puts(const char *s)
{
  __puts(s, strlen(s));

  putchar('\n');

  return 0;
}


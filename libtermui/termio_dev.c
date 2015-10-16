/*
    This file is part of libtermui.

    libtermui is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libtermui is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libtermui.  If not, see <http://www.gnu.org/licenses/>.

    Copyright 2006, Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include <unistd.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>

#include "term_pv.h"
#include <termui/term_keys.h>

#include <termui/mutekh.h>

#include <device/class/char.h>

static TERMUI_TERMIO_FCN_WRITE(termui_dev_write)
{
  struct device_char_s *dev = tm->io_pv;

  dev_char_wait_write(dev, (void*)data, size);
}

static TERMUI_TERMIO_FCN_READ(termui_dev_read)
{
  struct device_char_s *dev = tm->io_pv;
  uint8_t c;
  ssize_t res = dev_char_wait_read(dev, &c, 1);
  return res == 1 ? c : -1;
}

static TERMUI_TERMIO_FCN_POLLFD(termui_dev_pollfd)
{
#warning remove this poll function.
  return 42;
}

static TERMUI_TERMIO_FCN_CLEANUP(termui_dev_cleanup)
{
}

termui_err_t
termui_dev_io_init(struct termui_term_s *tm, struct device_char_s *dev, const char *type)
{
  memset(tm->type, 0, TERMUI_TYPESTR_MAXLEN);
  tm->ioerr = 0;
  tm->io_pv = dev;

  /* register tty operations in term object */
  tm->s_write = termui_dev_write;
  tm->s_read = termui_dev_read;
  tm->s_pollfd = termui_dev_pollfd;
  tm->s_cleanup = termui_dev_cleanup;

  tm->s_getsize = (termui_termio_getsize_t*)term_err;

  strncpy(tm->type, type, TERMUI_TYPESTR_MAXLEN);
  termui_term_set(tm);

  return 0;
}


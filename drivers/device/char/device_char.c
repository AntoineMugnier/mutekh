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

    Synchronous read and write functions for block device.

*/

#include <hexo/device.h>
#include <device/char.h>
#include <device/driver.h>

/* FIXME develop and use asynchronous char device API */
error_t dev_char_wait_read(struct device_s *dev, uint8_t *data, size_t size)
{
  ssize_t res;

  do
    {
      res = dev_char_read(dev, data, size);
    }
  while (res == 0);

  return res;
}

/* FIXME develop and use asynchronous char device API */
error_t dev_char_wait_write(struct device_s *dev, const uint8_t *data, size_t size)
{
  ssize_t res;

  do
    {
      res = dev_char_write(dev, data, size);
    }
  while (res == 0);

  return res;
}


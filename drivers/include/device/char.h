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

#ifndef __DEVICE_CHAR_H__
#define __DEVICE_CHAR_H__

#include <hexo/types.h>
#include <hexo/error.h>

struct device_s;
struct driver_s;

/** Char device class read() function tempate. */
#define DEVCHAR_READ(n)	ssize_t  (n) (struct device_s *dev, uint8_t *data, size_t size)

/** Char device class read() methode shortcut */

#define dev_char_read(dev, ...) (dev)->drv->f.chr.f_read(dev, __VA_ARGS__ )
/**
   Char device class read() function type.  Read bytes data from the
   device. Should not block if unable to read more bytes.
    
   @param dev pointer to device descriptor
   @param data pointer to data buffer
   @param size max data read bytes count
   @return data bytes count read from the device or negative error code
*/
typedef DEVCHAR_READ(devchar_read_t);




/** Char device class write() function tempate. */
#define DEVCHAR_WRITE(n)	ssize_t  (n) (struct device_s *dev, const uint8_t *data, size_t size)

/** 
    Char device class write() function type.  Write bytes data to the
    device. Return number of bytes written. Should not block if unable
    to write more bytes.

    @param dev pointer to device descriptor
    @param data pointer to read only data buffer
    @param size data bytes count
    @return data bytes count written to the device or negative error code
*/
typedef DEVCHAR_WRITE(devchar_write_t);

/** Char device class write() methode shortcut */
#define dev_char_write(dev, ...) (dev)->drv->f.chr.f_write(dev, __VA_ARGS__ )




/** Char device class methodes */
struct dev_class_char_s
{
  devchar_read_t		*f_read;
  devchar_write_t		*f_write;
};


#endif


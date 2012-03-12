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

*/

/**
 * @file
 * @module{Device drivers}
 * @short Sound device driver API
 */

#ifndef __DEVICE_SOUND_H__
#define __DEVICE_SOUND_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>

struct device_s;
struct driver_s;

#define DEVSOUND_CALLBACK(n)	void (n)(void *priv)
typedef DEVSOUND_CALLBACK(devsound_callback_t);

/** Sound device class read() function tempate. */
#define DEVSOUND_READ(n)	ssize_t  (n) (struct device_s *dev, uint8_t *data, size_t count, \
					      devsound_callback_t *cback, void *priv)

/**
   Sound device class read() function type.  Read bytes data from the
   device. Should not block if unable to read more bytes.

   @param dev pointer to device descriptor
   @param data pointer to data buffer
   @param count max sample to read
   @param cback function called when reading done
   @return data bytes count read from the device or negative error code
*/
typedef DEVSOUND_READ(devsound_read_t);




/** Sound device class write() function tempate. */
#define DEVSOUND_WRITE(n)	ssize_t  (n) (struct device_s *dev, const uint8_t *data, size_t count, \
					      devsound_callback_t *cback, void *priv)

/** 
    Sound device class write() function type.  Write bytes data to the
    device. Return number of bytes written. Should not block if unable
    to write more bytes.

    @param dev pointer to device descriptor
    @param data pointer to read only data buffer
    @param count data bytes count
    @return data bytes count written to the device or negative error code
*/
typedef DEVSOUND_WRITE(devsound_write_t);



enum dev_sound_mode_e
  {
    dev_sound_8bit_signed,
    dev_sound_8bit_unsigned,
    dev_sound_16bit_signed,
    dev_sound_16bit_unsigned,
  };

/** Sound device class write() function tempate. */
#define DEVSOUND_MODE(n)	error_t  (n) (struct device_s *dev,	\
					      enum dev_sound_mode_e mode, \
					      uint_fast8_t chan_count)

/** 
    Sound device class mode() function type. Set sound samples format.

    @param dev pointer to device descriptor
    @param mode sample format
    @param chan_count number of channels
    @return non zero value on error
*/
typedef DEVSOUND_MODE(devsound_mode_t);



/** Sound device class methodes */
struct driver_sound_s
{
  enum device_class_e cl;
  devsound_read_t		*f_read;
  devsound_write_t		*f_write;
  devsound_mode_t		*f_mode;
};


#endif


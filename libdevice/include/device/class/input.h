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
 * @module{Devices support library}
 * @short Event driven input device driver API
 */

#ifndef __DEVICE_INPUT_H__
#define __DEVICE_INPUT_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>

struct device_s;
struct driver_s;
struct device_input_s;
struct driver_input_s;

#define DEV_INPUT_EVENT_BUTTON_UP	0x01
#define DEV_INPUT_EVENT_BUTTON_DOWN	0x02
#define DEV_INPUT_EVENT_AXIS_MOVED	0x04

#define DEV_INPUT_CTRLID_ALL		((dev_input_ctrlid_t)-1)

/** id of a control (button or axe) for an input device */
typedef uint_fast8_t dev_input_ctrlid_t;
/** control value type. This can be a simple boolean value for button,
    a position or a move offset for an axis control */
typedef uint_fast16_t dev_input_value_t;

struct dev_input_info_s
{
  const char	*name;
  size_t	ctrl_button_count;
  size_t	ctrl_axe_count;
};


/** input device class callback function template */
#define DEV_INPUT_CALLBACK(n)	void (n) (dev_input_ctrlid_t id, dev_input_value_t value, void *priv)
/** input device class callback function type */
typedef DEV_INPUT_CALLBACK(dev_input_callback_t);


/** Input device class info function tempate. */
#define DEV_INPUT_INFO(n)	void  (n) (struct device_input_s *idev,		\
					   struct dev_input_info_s *info)

/**
   Input device class info() function type. This function get
   informations about available controles.

   @param dev pointer to device descriptor
   @param info pointer to information structure to fill in
*/
typedef DEV_INPUT_INFO(dev_input_info_t);


/** Input device class read function tempate. */
#define DEV_INPUT_READ(n)	dev_input_value_t (n) (struct device_input_s *idev,	\
						      dev_input_ctrlid_t id)

/**
   Input device class read() function type. This function read control
   current value.

   @param dev pointer to device descriptor
   @param id id of the controle to read
*/
typedef DEV_INPUT_READ(dev_input_read_t);



/** Input device class write function tempate. */
#define DEV_INPUT_WRITE(n)	error_t (n) (struct device_input_s *idev,	\
					     dev_input_ctrlid_t id,	\
					     dev_input_value_t value)

/**
   Input device class write() function type. This function set control
   current value.

   @param dev pointer to device descriptor
   @param id id of the controle to write
   @param value new status value for the control
*/
typedef DEV_INPUT_WRITE(dev_input_write_t);



/** Input device class event setcallback function tempate. */
#define DEV_INPUT_SETCALLBACK(n)	error_t (n) (struct device_input_s *idev, \
					     uint_fast8_t type,			\
					     dev_input_ctrlid_t id,		\
					     dev_input_callback_t *callback,	\
					     void *priv)

/**
   Input device class setcallback() function type. This function set
   a new event handler for a control. Special DEV_INPUT_CTRLID_ALL
   value can be used to get events for all controls available on this
   device. a NULL function pointer disable callback.

   @param dev pointer to device descriptor
   @param type type mask value for event types to watch
   @param id id of the control to watch
   @param callback new callback function
   @param priv private data passed to callback function
   @return non zero value on error
*/
typedef DEV_INPUT_SETCALLBACK(dev_input_setcallback_t);

DRIVER_CLASS_STRUCT(input, 
                    dev_input_info_t *f_info;
                    dev_input_read_t *f_read;
                    dev_input_write_t *f_write;
                    dev_input_setcallback_t *f_setcallback;
                    );

#endif


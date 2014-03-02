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

    Copyright (c) 2014 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include <device/class/gpio.h>
#include <device/device.h>
#include <device/driver.h>

#include <stdarg.h>

error_t device_gpio_map_set_mode(struct device_gpio_s *gpdev,
                                 const gpio_id_t *map, const gpio_width_t *wmap,
                                 uint_fast8_t count, /* enum dev_gpio_mode_e */ ...)
{
  va_list ap;
  uint_fast8_t i;
  error_t err = 0;

  va_start(ap, count);

  for (i = 0; i < count; i++)
    {
      gpio_id_t id = map[i];
      enum dev_gpio_mode_e mode = va_arg(ap, __compiler_sint_t);

      if (id != GPIO_INVALID_ID)
        if (DEVICE_OP(gpdev, set_mode, id, id + wmap[i] - 1, dev_gpio_mask1, mode))
          {
            err = -EINVAL;
            break;
          }
    }

  va_end(ap);
  return err;
}

error_t device_res_gpio_map(struct device_s *dev, const char *pin_list,
                            gpio_id_t *map, gpio_width_t *wmap)
{
  while (*pin_list)
    {
      struct dev_resource_s *r = device_res_get_from_name(dev, DEV_RES_GPIO, 0, pin_list);

      while (*pin_list && *pin_list != ':' && *pin_list != '?')
        pin_list++;

      if (*pin_list == '?')
        pin_list++;
      else if (!r)
        return -ENOENT;

      *map++ = r ? r->u.gpio.id : GPIO_INVALID_ID;
      if (wmap)
        *wmap++ = r ? r->u.gpio.width : 0;

      if (*pin_list == ':')
        {
          uint_fast8_t w = strtoul(pin_list + 1, (char**)&pin_list, 0);
          if (r && r->u.gpio.width != w)
            return -ERANGE;
        }

      while (*pin_list == ' ')
        pin_list++;
    }

  return 0;
}


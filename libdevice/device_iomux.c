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

#include <device/class/iomux.h>
#include <device/device.h>
#include <device/driver.h>

#include <stdarg.h>

static enum dev_pin_driving_e device_iomux_mode(char l)
{
  struct switch_s { char c; char n; };
  static const struct switch_s sw[10] = {
    { '^', DEV_PIN_OPENSOURCE },
    { '_', DEV_PIN_OPENDRAIN },
    { '`', DEV_PIN_OPENSOURCE_PULLDOWN },
    { '+', DEV_PIN_INPUT_PULLUP },
    { ',', DEV_PIN_OPENDRAIN_PULLUP },
    { '-', DEV_PIN_INPUT_PULLDOWN },
    { 0, 0 },
    { '<', DEV_PIN_INPUT },
    { '=', DEV_PIN_INPUT_PULL },
    { '>', DEV_PIN_PUSHPULL },
  };

  /* decode direction symbol using a perfect hash */
  uint32_t x = ((319838000U * (uint32_t)l) >> 28);
  if (x < 10 && sw[x].c == l)
    return sw[x].n;

  return DEV_PIN_DISABLED;
}

error_t device_iomux_setup(struct device_s *dev, const char *io_list,
                           iomux_demux_t *demux, iomux_io_id_t *io_id,
                           iomux_config_t *config)
{
  struct device_iomux_s iomux;

  error_t err, ent = device_get_param_dev_accessor(dev, "iomux",
                                        &iomux.base, DRIVER_CLASS_IOMUX);
  if (ent && ent != -ENOENT)
    return ent;

  while (*io_list)
    {
      enum dev_pin_driving_e dir = device_iomux_mode(*io_list);
      if (dir)
        io_list++;

      /* lookup io name in resources */
      struct dev_resource_s *r = device_res_get_from_name(dev, DEV_RES_IOMUX, 0, io_list);

      /* skip io name */
      while (*io_list > ' ')
        {
          io_list++;
          if (io_list[-1] == '?')
            goto done;
        }
      if (!r)
        return -ENOENT;
    done:

      if (r)
        {
          enum dev_pin_driving_e dir2 = device_iomux_mode(*r->u.iomux.label);
          if (dir2)
            dir = dir2;

          /* configure io on iomux */
          if (!ent && (err = DEVICE_OP(&iomux, setup, r->u.iomux.io_id, dir,
                                       r->u.iomux.mux, r->u.iomux.config)))
            return err;
        }

      /* fill arrays */
      if (demux)
        *demux++ = r ? r->u.iomux.demux : IOMUX_INVALID_DEMUX;
      if (io_id)
        *io_id++ = !ent && r ? r->u.iomux.io_id : IOMUX_INVALID_ID;
      if (config)
        *config++ = ent && r ? r->u.iomux.config : 0;

      while (*io_list == ' ')
        io_list++;
    }

  if (!ent)
    device_put_accessor(&iomux.base);

  return 0;
}


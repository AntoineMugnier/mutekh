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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2016
*/

#include <hexo/enum.h>

const char *enums_get_name(const char desc[], intptr_t val)
{
  ENUM_FOREACH(uintptr_t, desc, {
      if (value == val)
        return name;
    });

  return NULL;
}

bool_t enums_get_value2(const char desc[], const char *name_,
                        size_t name_len, intptr_t *val)
{
  ENUM_FOREACH(uintptr_t, desc, {
      if (!strncmp(name, name_, name_len) && !name[name_len])
        {
          *val = value;
          return 1;
        }
    });

  return 0;
}


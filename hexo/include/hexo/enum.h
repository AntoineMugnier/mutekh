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

/**
   @file
   @module{Hardware abstraction layer}
   @short C enum reflection

   This header provides functions which can be used to convert between
   an enum value and an enum entry name at run time.

   C enumerations are be parsed by the @sourcelink scripts/enum.pl
   script at compile time in order to produce enum descriptors usable
   at run time. A descriptor is more compact than an array of strings.

   The script is invoked automatically by the MutekH build system. It
   contains syntax and usage details.

   Any source file which contains enums which needs to be processed in
   this way must be specified in the @tt enum_headers variable of a
   @tt{Makefile}. All enums with a @ref #ENUM_DESCRIPTOR declaration
   will then produce a string descriptor macro available when this
   header is included.
*/

#include "enums.h"

#include <hexo/types.h>
#include <string.h>

#ifndef __HEXO_ENUMS_H__
#define __HEXO_ENUMS_H__

/** @This returns the name associated to an enum value according to
    the passed descriptor string. */
const char *enums_get_name(const char desc[], intptr_t value);

/** @This gets the value of an enum entry from the name. @This returns
    false if no entry is found. */
bool_t enums_get_value2(const char desc[], const char *name,
                        size_t name_len, intptr_t *value);

/** @This is a wrapper of the @ref enums_get_value2 function for NUL
    terminated strings. */
ALWAYS_INLINE bool_t enums_get_value(const char desc[], const char *name,
                                     intptr_t *value)
{
  return enums_get_value2(desc, name, strlen(name), value);
}

# undef ENUM_DESCRIPTOR
/** This mark an enum for processing by the @sourceref scripts/enum.pl
    script. It also produce an @tt extern declaration of the enum
    descriptor. */
# define ENUM_DESCRIPTOR(name, ...) extern const char name[];

#endif


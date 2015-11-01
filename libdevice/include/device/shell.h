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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#ifndef __DEVICE_SHELL_H__
#define __DEVICE_SHELL_H__

#include <mutek/shell.h>
#include <device/device.h>
#include <device/driver.h>

struct dev_console_opt_device_s
{
  struct termui_con_opts_s opt;
  device_filter_t          *filter;
  uint16_t                 offset; /* offset of field struct device_s * */
};

FIRST_FIELD_ASSERT(dev_console_opt_device_s, opt);

/* define a macro to use as console option descriptor parameter */
#define TERMUI_CON_OPT_DEV_DEVICE_ENTRY(sname_, lname_, id_, type_, field_, filter_, ...) \
  TERMUI_CON_OPT_CUSTOM_ENTRY(dev_console_opt_device_s, sname_, lname_, id_, \
    TERMUI_CON_OPT_PARSE(dev_console_opt_device_parse, 1)          \
    TERMUI_CON_OPT_COMPLETE(dev_console_opt_device_comp, NULL)     \
    .offset = offsetof(type_, field_),                             \
    .filter = filter_,                                             \
    __VA_ARGS__                                                    \
  )

struct dev_console_opt_driver_s
{
  struct termui_con_opts_s opt;
  uint16_t                 offset; /* offset of field struct driver_s * */
};

FIRST_FIELD_ASSERT(dev_console_opt_driver_s, opt);

/* define a macro to use as console option descriptor parameter */
#define TERMUI_CON_OPT_DEV_DRIVER_ENTRY(sname_, lname_, id_, type_, field_, ...) \
  TERMUI_CON_OPT_CUSTOM_ENTRY(dev_console_opt_device_s, sname_, lname_, id_, \
    TERMUI_CON_OPT_PARSE(dev_console_opt_driver_parse, 1)          \
    .offset = offsetof(type_, field_),                             \
    __VA_ARGS__                                                    \
  )

struct dev_console_opt_accessor_s
{
  struct termui_con_opts_s opt;
  enum driver_class_e      BITFIELD(cl,8);
  uint16_t                 offset; /* offset of device accessor field */
};

FIRST_FIELD_ASSERT(dev_console_opt_accessor_s, opt);

#define TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY(sname_, lname_, id_, type_, field_, class_, ...) \
  TERMUI_CON_OPT_CUSTOM_ENTRY(dev_console_opt_accessor_s, sname_, lname_, id_, \
    TERMUI_CON_OPT_PARSE(dev_console_opt_accessor_parse, 1)             \
    TERMUI_CON_OPT_COMPLETE(dev_console_opt_accessor_comp, NULL)        \
    .offset = offsetof(type_, field_),                                  \
    .cl = class_,                                                       \
    __VA_ARGS__                                                         \
  )

struct dev_console_opt_freq_s
{
  struct termui_con_opts_s opt;
  uint16_t                 offset;
};

FIRST_FIELD_ASSERT(dev_console_opt_freq_s, opt);

#define TERMUI_CON_OPT_FREQ_ENTRY(sname_, lname_, id_, type_, field_, ...) \
  TERMUI_CON_OPT_CUSTOM_ENTRY(dev_console_opt_freq_s, sname_, lname_, id_, \
    TERMUI_CON_OPT_PARSE(dev_console_opt_freq_parse, 1)                    \
    .offset = offsetof(type_, field_),                                     \
    __VA_ARGS__                                                            \
  )

#define TERMUI_CON_OPT_FREQ_RATIO_ENTRY(sname_, lname_, id_, type_, field_, ...) \
  TERMUI_CON_OPT_CUSTOM_ENTRY(dev_console_opt_freq_s, sname_, lname_, id_,       \
    TERMUI_CON_OPT_PARSE(dev_console_opt_freq_ratio_parse, 1)                    \
    .offset = offsetof(type_, field_),                                           \
    __VA_ARGS__                                                                  \
  )

/* generic parsing function for our custom option */
TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_device_parse);

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_accessor_parse);

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_driver_parse);

TERMUI_CON_ARGS_COLLECT_PROTOTYPE(dev_console_opt_device_comp);

TERMUI_CON_ARGS_COLLECT_PROTOTYPE(dev_console_opt_accessor_comp);

TERMUI_CON_ARGS_COLLECT_PROTOTYPE(dev_console_device_comp);

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_freq_parse);

TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_console_opt_freq_ratio_parse);

#endif


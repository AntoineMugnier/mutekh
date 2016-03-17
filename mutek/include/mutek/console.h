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
 * @module{Kernel services}
 * @short Console device defs
 * @internal
 */

#ifndef MUTEK_CONSOLE_H_
#define MUTEK_CONSOLE_H_

struct fileops_s;
struct device_char_s;

/** @internal File operations needed to access the mutek console using
    the libc stdio functions. */
config_depend_and2(CONFIG_MUTEK_CONSOLE, CONFIG_MUTEK_CONTEXT_SCHED)
extern const struct fileops_s console_file_ops;

/** This is the main mutek console device. This @xref{device accessor}
    is initialized on startup according to the value of the @ref
    #CONFIG_MUTEK_CONSOLE_DEVICE_PATHS token. */
config_depend(CONFIG_MUTEK_CONSOLE)
extern struct device_char_s console_dev;

#endif


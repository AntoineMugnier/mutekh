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

    Synchronous read and write functions for character device.
*/

#include <device/device.h>
#include <device/class/char.h>
#include <device/driver.h>

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
# include <mutek/scheduler.h>

extern inline ssize_t dev_char_wait_op(
    const struct device_char_s *accessor,
    enum dev_char_rq_type_e type, uint8_t *data, size_t size);

extern inline error_t dev_char_wait_rq(
    const struct device_char_s *accessor,
    struct dev_char_rq_s *rq);

#endif


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

#include <device/class/valio.h>

extern inline error_t dev_valio_spin_request(
    const struct device_valio_s *accessor,
    struct dev_valio_rq_s *req);

extern inline error_t dev_valio_spin_read(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data);

extern inline error_t dev_valio_spin_write(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    const void *data);

extern inline error_t dev_valio_spin_update(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data);

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
# include <mutek/scheduler.h>

extern inline error_t dev_valio_wait_request(
    const struct device_valio_s *accessor,
    struct dev_valio_rq_s *req);

extern inline error_t dev_valio_wait_read(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data);

extern inline error_t dev_valio_wait_write(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    const void *data);

extern inline error_t dev_valio_wait_update(
    const struct device_valio_s *accessor,
    uint16_t attribute,
    void *data);

#endif

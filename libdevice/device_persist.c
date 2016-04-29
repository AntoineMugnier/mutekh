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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <device/class/persist.h>
#include <hexo/enum.h>

//const char dev_persist_type_e[] = ENUM_DESC_DEV_PERSIST_TYPE_E;
//const char dev_persist_op_e[] = ENUM_DESC_DEV_PERSIST_OP_E;

extern inline error_t
dev_persist_spin_op(struct device_persist_s *accessor,
                    struct dev_persist_rq_s *rq);

extern inline error_t
dev_persist_wait_op(struct device_persist_s *accessor,
                   struct dev_persist_rq_s *rq);

inline error_t
dev_persist_wait_read(struct device_persist_s *accessor,
                      const struct dev_persist_descriptor_s *desc,
                      uint16_t uid_offset,
                      const void **data);

inline error_t
dev_persist_wait_write(struct device_persist_s *accessor,
                       const struct dev_persist_descriptor_s *desc,
                       uint16_t uid_offset,
                       const void *data);

inline error_t
dev_persist_wait_inc(struct device_persist_s *accessor,
                     const struct dev_persist_descriptor_s *desc,
                     uint16_t uid_offset);

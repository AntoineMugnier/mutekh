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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
*/

#include <device/class/rfpacket.h>
#include <hexo/enum.h>

const char dev_rfpacket_modulation_e[] = ENUM_DESC_DEV_RFPACKET_MODULATION_E;
const char dev_rfpacket_encoding_e[] = ENUM_DESC_DEV_RFPACKET_ENCODING_E;
const char dev_rfpacket_format_e[] = ENUM_DESC_DEV_RFPACKET_FORMAT_E;
const char dev_rfpacket_lora_encoding_e[] = ENUM_DESC_DEV_RFPACKET_LORA_ENCODING_E;

# ifdef CONFIG_MUTEK_CONTEXT_SCHED

extern inline error_t dev_rfpacket_wait_request(
       const struct device_rfpacket_s *accessor,
       struct dev_rfpacket_rq_s *rq);

#endif

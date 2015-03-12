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

#include <device/class/rfpacket.h>


extern inline error_t dev_rfpacket_spin_send_packet(
       const struct device_rfpacket_s *accessor,
       const uint8_t *buf,
       const size_t size,
       int16_t pwr,
       uint32_t lifetime);

extern inline error_t dev_rfpacket_spin_config(
    const struct device_rfpacket_s *accessor,
    const struct dev_rfpacket_config_s *cfg,
    enum dev_rfpacket_cfg_msk_e mask);

# ifdef CONFIG_MUTEK_SCHEDULER
extern inline error_t dev_rfpacket_wait_send_packet(
       const struct device_rfpacket_s *accessor,
       const uint8_t *buf,
       const size_t size,
       int16_t pwr,
       uint32_t lifetime);

extern inline error_t dev_rfpacket_wait_config(
    const struct device_rfpacket_s *accessor,
    const struct dev_rfpacket_config_s *cfg,
    enum dev_rfpacket_cfg_msk_e mask);

#endif

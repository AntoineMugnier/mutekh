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
*/

#ifndef S2LP_CONFIG_H_
#define S2LP_CONFIG_H_

// Init config table
extern const uint8_t s2lp_config[];

// Public functions
error_t s2lp_build_config(struct s2lp_ctx_s *pv);
uint8_t s2lp_build_pwr(struct s2lp_ctx_s *pv, int16_t pwr);
void s2lp_init_rf_cfg_array(uint8_t *pArray, uint16_t array_size);
void s2lp_init_pk_cfg_array(uint8_t *pArray, uint16_t array_size);

#endif
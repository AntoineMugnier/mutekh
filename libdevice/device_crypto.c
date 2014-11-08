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

#include <device/class/crypto.h>
#include <enums.h>

const char dev_crypto_mode_e[] = ENUM_DESC_DEV_CRYPTO_MODE_E;
const char dev_crypto_op_e[] = ENUM_DESC_DEV_CRYPTO_OP_E;

extern inline error_t
dev_crypto_wait_op(struct device_crypto_s *accessor,
                   struct dev_crypto_rq_s *rq);

extern inline bool_t
dev_crypto_ctx_bind(struct dev_crypto_context_s *ctx,
                    struct dev_crypto_context_s *ctx_array[],
                    dev_crypto_context_id_t *next,
                    dev_crypto_context_id_t count);

extern inline uint8_t
dev_crypto_memcmp(const void *a, const void *b, size_t len);

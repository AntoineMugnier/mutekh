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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <device/class/nfc.h>

const char dev_nfc_req_type_e[] = ENUM_DESC_DEV_NFC_REQ_TYPE_E;
const char dev_nfc_side_e[] = ENUM_DESC_DEV_NFC_SIDE_E;
const char dev_nfc_protocol_e[] = ENUM_DESC_DEV_NFC_PROTOCOL_E;

extern inline error_t dev_nfc_spin_request(
    const struct device_nfc_s *accessor,
    struct dev_nfc_rq_s *req);

extern inline error_t dev_nfc_wait_request(
    const struct device_nfc_s *accessor,
    struct dev_nfc_rq_s *req);

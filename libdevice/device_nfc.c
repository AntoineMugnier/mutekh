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

    Copyright (c) 2016, Nicolas Pouillon, <nipo@ssji.net>
*/

#include <device/class/nfc.h>

const char dev_nfc_req_type_e[] = ENUM_DESC_DEV_NFC_REQ_TYPE_E;
const char dev_nfc_side_e[] = ENUM_DESC_DEV_NFC_SIDE_E;
const char dev_nfc_protocol_e[] = ENUM_DESC_DEV_NFC_PROTOCOL_E;

extern inline error_t dev_nfc_spin_request(
    const struct device_nfc_s *accessor,
    struct dev_nfc_rq_s *rq);

extern inline error_t dev_nfc_wait_request(
    const struct device_nfc_s *accessor,
    struct dev_nfc_rq_s *rq);

error_t dev_nfc_wait_transceive_std(
    const struct device_nfc_s *accessor,
    const uint8_t *tx_data, size_t tx_size,
    uint8_t *rx_data, size_t *rx_size,
    enum dev_nfc_div_log2_e div)
{
  struct dev_nfc_rq_s rq1, rq2;
  struct dev_request_status_s st1, st2;

  dev_request_sched_init(&rq1.base, &st1);
  rq1.type = DEV_NFC_TRANSMIT;
  rq1.data.data = (uint8_t*)tx_data;
  rq1.data.size = tx_size;
  rq1.data.framing = DEV_NFC_FRAMING_STD;
  rq1.data.last_byte_bits = 0;
  rq1.data.div_log2 = div;

  dev_request_sched_init(&rq2.base, &st2);
  rq2.type = DEV_NFC_RECEIVE;
  rq2.data.data = rx_data;
  rq2.data.size = *rx_size;
  rq2.data.framing = DEV_NFC_FRAMING_STD;
  rq2.data.last_byte_bits = 0;
  rq2.data.div_log2 = div;

  DEVICE_OP(accessor, request, &rq1, &rq2);

  dev_request_sched_wait(&st1);
  dev_request_sched_wait(&st2);

  *rx_size = rq2.error ? 0 : rq2.data.size;

  return rq1.error ? rq1.error : rq2.error;
}

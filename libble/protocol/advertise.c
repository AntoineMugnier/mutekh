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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <hexo/error.h>
#include <mutek/printk.h>
#include <mutek/buffer_pool.h>
#include <ble/protocol/address.h>
#include <ble/protocol/gap.h>
#include <ble/protocol/company.h>
#include <ble/protocol/llcp.h>
#include <ble/protocol/advertise.h>

const char *ble_adv_type_name[BLE_ADV_TYPE_LAST + 1] = {
    [BLE_ADV_IND] = "Adv. Ind",
    [BLE_ADV_DIRECT_IND] = "Dir. Ind",
    [BLE_ADV_NONCONN_IND] = "NonC Ind",
    [BLE_SCAN_REQ] = "Scan req",
    [BLE_SCAN_RSP] = "Scan rsp",
    [BLE_CONNECT_REQ] = "Conn req",
    [BLE_ADV_SCAN_IND] = "Scan Ind",
};

void ble_adv_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva)
{
  p->begin = 0;
  p->end = 8;

  p->data[BLE_ADV_TYPE_OFFSET] = BLE_ADV_IND;
  p->data[BLE_ADV_LEN_OFFSET] = 6;

  ble_advertise_packet_txaddr_set(p, adva);
}

void ble_adv_direct_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva,
    const struct ble_addr_s *inita)
{
  p->begin = 0;
  p->end = 14;

  p->data[BLE_ADV_TYPE_OFFSET] = BLE_ADV_DIRECT_IND;
  p->data[BLE_ADV_LEN_OFFSET] = 12;

  ble_advertise_packet_txaddr_set(p, adva);
  ble_advertise_packet_rxaddr_set(p, inita);
}

void ble_adv_nonconn_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva)
{
  p->begin = 0;
  p->end = 8;

  p->data[BLE_ADV_TYPE_OFFSET] = BLE_ADV_NONCONN_IND;
  p->data[BLE_ADV_LEN_OFFSET] = 6;

  ble_advertise_packet_txaddr_set(p, adva);
}

void ble_adv_scan_ind_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva)
{
  p->begin = 0;
  p->end = 8;

  p->data[BLE_ADV_TYPE_OFFSET] = BLE_ADV_SCAN_IND;
  p->data[BLE_ADV_LEN_OFFSET] = 6;

  ble_advertise_packet_txaddr_set(p, adva);
}

void ble_adv_scan_req_set(
    struct buffer_s *p,
    const struct ble_addr_s *scana,
    const struct ble_addr_s *adva)
{
  p->begin = 0;
  p->end = 14;

  p->data[BLE_ADV_TYPE_OFFSET] = BLE_SCAN_REQ;
  p->data[BLE_ADV_LEN_OFFSET] = 12;

  ble_advertise_packet_txaddr_set(p, scana);
  ble_advertise_packet_rxaddr_set(p, adva);
}

void ble_adv_scan_rsp_set(
    struct buffer_s *p,
    const struct ble_addr_s *adva)
{
  p->begin = 0;
  p->end = 8;

  p->data[BLE_ADV_TYPE_OFFSET] = BLE_SCAN_RSP;
  p->data[BLE_ADV_LEN_OFFSET] = 6;

  ble_advertise_packet_txaddr_set(p, adva);
}

void ble_adv_connect_set(
    struct buffer_s *p,
    const struct ble_adv_connect_s *params)
{
  p->begin = 0;
  p->end = 36;

  p->data[BLE_ADV_TYPE_OFFSET] = BLE_CONNECT_REQ;
  p->data[BLE_ADV_LEN_OFFSET] = 34;

  ble_advertise_packet_txaddr_set(p, &params->master);
  ble_advertise_packet_rxaddr_set(p, &params->slave);

  endian_le32_na_store(&p->data[14], params->access_address);
  endian_le32_na_store(&p->data[18], params->crc_init);
  p->data[21] = params->win_size;
  endian_le16_na_store(&p->data[22], params->win_offset);
  endian_le16_na_store(&p->data[24], params->timing.interval);
  endian_le16_na_store(&p->data[26], params->timing.latency);
  endian_le16_na_store(&p->data[28], params->timing.timeout);
  endian_le32_na_store(&p->data[30], params->channel_map);
  p->data[34] = params->channel_map >> 32;
  p->data[35] = (params->sca << 5) | params->hop;
}

error_t ble_adv_connect_parse(
  const struct buffer_s *p,
  struct ble_adv_connect_s *params)
{
  const uint8_t *data = p->data + p->begin;

  if (p->end - p->begin < 36)
    return -ENOMEM;

  if (ble_advertise_packet_type_get(p) != BLE_CONNECT_REQ)
    return -EINVAL;

  if (data[1] != 34)
    return -ENOMEM;

  ble_advertise_packet_txaddr_get(p, &params->master);
  ble_advertise_packet_rxaddr_get(p, &params->slave);

  params->access_address = endian_le32_na_load(&data[14]);
  params->crc_init = endian_le32_na_load(&data[18]) & 0xffffff;
  params->win_size = data[21];
  params->win_offset = endian_le16_na_load(&data[22]);
  params->timing.interval = endian_le16_na_load(&data[24]);
  params->timing.latency = endian_le16_na_load(&data[26]);
  params->timing.timeout = endian_le16_na_load(&data[28]);
  params->channel_map = endian_le32_na_load(&data[30]);
  params->channel_map |= (uint64_t)data[34] << 32;
  params->sca = data[35] >> 5;
  params->hop = data[35] & 0x1f;

  return 0;
}

error_t ble_adv_data_append(
    struct buffer_s *p,
    const uint8_t type,
    const void *data, const uint8_t size)
{
  if (2 + 6 + 31 - (p->end - p->begin) < size + 2)
    return -ENOMEM;

  p->data[p->end++] = size + 1;
  p->data[p->end++] = type;
  memcpy(p->data + p->end, data, size);
  p->end += size;
  p->data[p->begin + BLE_ADV_LEN_OFFSET] += size + 2;

  return 0;
}

void ble_advertise_packet_dump(const struct buffer_s *p)
{
  const uint8_t *data = p->data + p->begin;
  uint8_t size = p->end - p->begin;
  struct ble_addr_s addr;
  enum ble_adv_type type = ble_advertise_packet_type_get(p);

  if (type > BLE_ADV_TYPE_LAST)
    goto invalid;

  if (size < 8)
    goto invalid;

  printk("%s", ble_adv_type_name[type]);

  //  printk(" [%P]", data, size);

  switch (type) {
  case BLE_ADV_NONCONN_IND:
  case BLE_SCAN_RSP:
  case BLE_ADV_IND:
  case BLE_ADV_SCAN_IND:
    ble_advertise_packet_txaddr_get(p, &addr);
    printk(" from " BLE_ADDR_FMT, BLE_ADDR_ARG(&addr));
    data += 8;
    size -= 8;

    while (size) {
      if (data[0] + 1 > size || data[0] == 0)
        goto invalid;

      switch (data[1]) {
      case BLE_GAP_FLAGS:
          printk(", flags 0x%x", data[2]);
          break;

      case BLE_GAP_UUID16_SERVICE_LIST_INCOMPLETE:
          printk(", srv");
          for (uint8_t i = 0; i < data[0] - 1; i += 2)
              printk(" %04x", endian_le16_na_load(data + i + 2));
          printk(" ...");
          break;

      case BLE_GAP_UUID16_SERVICE_LIST_COMPLETE:
          printk(", srv");
          for (uint8_t i = 2; i < data[0] + 1; i += 2)
              printk(" %04x", endian_le16_na_load(data + i));
          break;

      case BLE_GAP_SHORTENED_LOCAL_NAME:
          printk(", \"%S...\"", data + 2, data[0] - 1);
          break;

      case BLE_GAP_SLAVE_CONNECTION_INTERVAL_RANGE:
          printk(", connInt %d-%d",
                 endian_le16_na_load(data + 2),
                 endian_le16_na_load(data + 4));
          break;

      case BLE_GAP_COMPLETE_LOCAL_NAME:
          printk(", \"%S\"", data + 2, data[0] - 1);
          break;

      case BLE_GAP_APPEARANCE:
          printk(", appear 0x%x", endian_le16_na_load(data + 2));
          break;

      case BLE_GAP_MANUFACTURER_SPECIFIC_DATA:
          printk(", %s-specific (%d) [%P]",
                 ble_company_name(endian_le16_na_load(data + 2)),
                 endian_le16_na_load(data + 2),
                 data + 4, data[0] - 3);
          break;

      default:
          printk(" %d: [%P]", data[1], data + 2, data[0] - 1);
          break;
      }

      size -= data[0] + 1;
      data += data[0] + 1;
    }
    break;

  case BLE_ADV_DIRECT_IND:
    ble_advertise_packet_txaddr_get(p, &addr);
    printk(" from " BLE_ADDR_FMT, BLE_ADDR_ARG(&addr));

    ble_advertise_packet_rxaddr_get(p, &addr);
    printk(" to " BLE_ADDR_FMT, BLE_ADDR_ARG(&addr));

    break;

  case BLE_SCAN_REQ:
    ble_advertise_packet_rxaddr_get(p, &addr);
    printk(" to   " BLE_ADDR_FMT, BLE_ADDR_ARG(&addr));

    ble_advertise_packet_txaddr_get(p, &addr);
    printk(" from " BLE_ADDR_FMT, BLE_ADDR_ARG(&addr));

    break;

  case BLE_CONNECT_REQ:
    ble_advertise_packet_rxaddr_get(p, &addr);
    printk(" to   " BLE_ADDR_FMT, BLE_ADDR_ARG(&addr));

    ble_advertise_packet_txaddr_get(p, &addr);
    printk(" from " BLE_ADDR_FMT, BLE_ADDR_ARG(&addr));
    printk("\n            int %dus, lat %d, ws %dus, +%d, chan map 0x%02x%08x, %d-%d ppm, timeout %dms, inc %d",
           endian_le16_na_load(data + 24) * 1250,
           endian_le16_na_load(data + 26), data[21] * 1250,
           endian_le16_na_load(data + 22),
           data[34], endian_le32_na_load(data + 30),
           ble_sca_bc[data[35] >> 5], ble_sca_wc[data[35] >> 5],
           endian_le16_na_load(data + 28) * 10,
           data[35] & 0x1f);
    break;
  }
  printk("\n");
  return;

invalid:
  printk("\nInvalid data, %d bytes left, %P\n", size, data, size);
  return;
}

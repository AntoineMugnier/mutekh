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

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <ble/protocol/data.h>
#include <ble/protocol/error.h>
#include <ble/protocol/company.h>

#include <hexo/decls.h>

const uint16_t ble_sca_wc[8] = {
  500, 250, 150, 100, 75, 50, 30, 20,
};

const uint16_t ble_sca_bc[8] = {
  251, 151, 101, 76, 51, 31, 21, 0,
};

void ble_data_conn_params_update_parse(
    const uint8_t *pdu,
    struct ble_conn_params_update *cpu)
{
    cpu->timing.win_size = pdu[1];
    cpu->win_offset = endian_le16_na_load(pdu + 2);
    cpu->timing.interval = endian_le16_na_load(pdu + 4);
    cpu->timing.latency = endian_le16_na_load(pdu + 6);
    cpu->timing.timeout = endian_le16_na_load(pdu + 8);
    cpu->instant = endian_le16_na_load(pdu + 10);
}

const char *const bt_hci_version_string[] =
{
  [BLE_LL_VERSION_1_0b] = "1.0b",
  [BLE_LL_VERSION_1_1] = "1.1",
  [BLE_LL_VERSION_1_2] = "1.2",
  [BLE_LL_VERSION_2_0_EDR] = "2.0+EDR",
  [BLE_LL_VERSION_2_1_EDR] = "2.1+EDR",
  [BLE_LL_VERSION_3_0_HS] = "3.0+HS",
  [BLE_LL_VERSION_4_0] = "4.0",
  [BLE_LL_VERSION_4_1] = "4.1",
  [BLE_LL_VERSION_4_2] = "4.2",
};

const char *const ble_feature_names[] =
{
  [BLE_LL_FEATURE_LE_ENCRYPTION] = "LE Encryption",
  [BLE_LL_FEATURE_CONNECTION_PARAMETERS_REQUEST_PROCEDURE] = "Connection Parameters Request Procedure",
  [BLE_LL_FEATURE_EXTENDED_REJECT_INDICATION] = "Extended Reject Indication",
  [BLE_LL_FEATURE_SLAVE_INITIATED_FEATURES_EXCHANGE] = "Slave-initiated Features Exchange",
  [BLE_LL_FEATURE_LE_PING] = "LE Ping",
  [BLE_LL_FEATURE_LE_DATA_PACKET_LENGTH_EXTENSION] = "LE Data Packet Length Extension",
  [BLE_LL_FEATURE_LL_PRIVACY] = "LL Privacy",
  [BLE_LL_FEATURE_EXTENDED_SCANNER_FILTER_POLICIES] = "Extended Scanner Filter Policies",
};

void ble_control_packet_dump(const struct buffer_s *p)
{
    const uint8_t *data = p->data + p->begin;
    uint8_t size = p->end - p->begin;

    if (size < 2) {
        printk("Bad size");
        return;
    }

    switch (ble_data_control_get(p)) {
    case BLE_LL_CONNECTION_UPDATE_REQ:
      printk("Conn update req at %d +%d, ws %d, int %d, lat %d, timeout %d\n",
             endian_le16_na_load(data + 12),
             endian_le16_na_load(data + 4),
             data[3],
             endian_le16_na_load(data + 6),
             endian_le16_na_load(data + 8),
             endian_le16_na_load(data + 10));
      break;

    case BLE_LL_CHANNEL_MAP_REQ:
      printk("Chan update req %02x%02x%02x%02x%02x at %d\n",
             data[7], data[6], data[5], data[4], data[3],
             endian_le16_na_load(data + 8));
      break;

    case BLE_LL_TERMINATE_IND:
      printk("Conn terminated %d\n", ble_error_string[data[3]]);
      break;

    case BLE_LL_ENC_REQ:
      printk("Enc req\n"
             "         Rand [%P]\n"
             "         EDIV [%P]\n"
             "         SKDm [%P]\n"
             "         IVm [%P]\n",
             data + 3, 8,
             data + 11, 2,
             data + 13, 8,
             data + 21, 4);
      break;

    case BLE_LL_ENC_RSP:
      printk("Enc rsp\n"
             "         SKDs [%P]\n"
             "         IVs [%P]\n",
             data + 3, 8,
             data + 11, 4);
      break;

    case BLE_LL_START_ENC_REQ:
      printk("Start enc req\n");
      break;

    case BLE_LL_START_ENC_RSP:
      printk("Start enc rsp\n");
      break;

    case BLE_LL_PAUSE_ENC_REQ:
      printk("Pause enc req\n");
      break;

    case BLE_LL_PAUSE_ENC_RSP:
      printk("Pause enc rsp\n");
      break;

    case BLE_LL_UNKNOWN_RSP:
      printk("Unknown rsp\n");
      break;

    case BLE_LL_FEATURE_REQ:
    case BLE_LL_FEATURE_RSP:
      printk("Feature set:\n");
      for (uint8_t i = 0; i < ARRAY_SIZE(ble_feature_names); ++i) {
        if (data[3 + (i >> 3)] & (1 << (i & 7)))
          printk("        - %s\n", ble_feature_names[i]);
      }
      break;

    case BLE_LL_VERSION_IND: {
      const char *company = ble_company_name(endian_le16_na_load(data + 4));
      const char *version = "other";

      if (data[3] < ARRAY_SIZE(bt_hci_version_string))
        version = bt_hci_version_string[data[3]];

      printk("Version v.%s (%d), company %d (%s), impl %04x\n",
             version, data[3],
             endian_le16_na_load(data + 4), company,
             endian_le16_na_load(data + 6));
      break;
    }

    case BLE_LL_REJECT_IND:
      printk("Reject ind: %s\n", ble_error_string[data[3]]);
      break;

    case BLE_LL_SLAVE_FEATURE_REQ:
      printk("Slave feature req [%P]\n", data + 3, size - 3);
      break;

    case BLE_LL_CONNECTION_PARAM_REQ:
      printk("Conn param req: int %d-%d, lat %d, timeout %dms, pper %d, ref cc %d\n"
             "        offsets: %d %d %d %d %d\n",
             endian_le16_na_load(data + 3), endian_le16_na_load(data + 5),
             endian_le16_na_load(data + 7), endian_le16_na_load(data + 9) * 10,
             data[11], endian_le16_na_load(data + 12),
             endian_le16_na_load(data + 14), endian_le16_na_load(data + 16),
             endian_le16_na_load(data + 18), endian_le16_na_load(data + 20),
             endian_le16_na_load(data + 22), endian_le16_na_load(data + 24));
      break;

    case BLE_LL_CONNECTION_PARAM_RSP:
      printk("Conn param rsp: int %d-%d, lat %d, timeout %dms, pper %d, ref cc %d\n"
             "        offsets: %d %d %d %d %d\n",
             endian_le16_na_load(data + 3), endian_le16_na_load(data + 5),
             endian_le16_na_load(data + 7), endian_le16_na_load(data + 9) * 10,
             data[11], endian_le16_na_load(data + 12),
             endian_le16_na_load(data + 14), endian_le16_na_load(data + 16),
             endian_le16_na_load(data + 18), endian_le16_na_load(data + 20),
             endian_le16_na_load(data + 22), endian_le16_na_load(data + 24));
      break;

    case BLE_LL_REJECT_IND_EXT:
      printk("Reject ind ext opcode %d error %d\n", data[3], data[4]);
      break;

    case BLE_LL_PING_REQ:
      printk("Ping req\n");
      break;

    case BLE_LL_PING_RSP:
      printk("Ping rsp\n");
      break;

    default:
      printk("Control other [%P]\n", data + 2, data[1]);
      break;
    }
}

void ble_data_packet_dump(const struct buffer_s *p, bool_t m2s)
{
    const uint8_t *data = p->data + p->begin;
    uint8_t size = p->end - p->begin;

    if (size < 2) {
        printk("Bad size\n");
        return;
    }

    switch (ble_data_llid_get(p)) {
    case BLE_LL_CONTROL:
      ble_control_packet_dump(p);
        break;

    case BLE_LL_DATA_CONT:
    case BLE_LL_DATA_START:
    default:
        if (size - 2) {
            printk("data:\n");
            hexdumpk(0, data + 2, size - 2);
        }
        break;
    }

    return;
}

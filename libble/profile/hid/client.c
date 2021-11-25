#define LOGK_MODULE_ID "hogc"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <ble/net/gatt.h>
#include <ble/net/att.h>
#include <ble/uuid.h>
#include <ble/protocol/att.h>
#include <ble/protocol/l2cap.h>
#include <ble/protocol/gatt.h>

#include <ble/gattdb/db.h>
#include <ble/gattdb/client.h>
#include <ble/gatt/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/descriptor.h>
#include <ble/gatt/dis.h>
#include <ble/gatt/hid.h>
#include <ble/security_db.h>

#include <ble/net/generic.h>

#include <ble/profile/hid/client.h>

enum hid_client_state_e
{
  HC_IDLE,
  HC_DISCOVER_HID,
  HC_DISCOVER_HID_CHARS,
  HC_DISCOVER_HID_REPORT,
  HC_DISCOVER_HID_CCCD,
  HC_DISCOVER_HID_COMPLETE,
  HC_READ_VENDOR,
  HC_READ_PRODUCT,
  HC_READ_SERIAL,
  HC_READ_PNP_ID,
  HC_READ_DESCRIPTOR,
  HC_SUBSCRIBE,
  HC_DISCOVERY_DONE,
  HC_FAIL,
};

struct hid_client_s
{
  struct net_layer_s layer;
  uint16_t service_handle;
  uint16_t point;
  uint16_t service_group_end;
  uint16_t descriptor_handle;
  struct hid_report_s chars[HID_CLIENT_CHAR_MAX];
  size_t char_count;

  enum hid_client_state_e state;

  struct ble_att_transaction_s att_txn;

  struct hid_client_descriptor_s meta;

  uint8_t descriptor[256];
  size_t descriptor_size;

  uint8_t scratch[32];
};

STRUCT_COMPOSE(hid_client_s, layer);
STRUCT_COMPOSE(hid_client_s, att_txn);

static void hid_client_txn_destroyed(void *task)
{
  struct ble_att_transaction_s *txn = ble_att_transaction_s_from_task(task);
  struct hid_client_s *hid = hid_client_s_from_att_txn(txn);

  hid->att_txn.task.destroy_func = NULL;
}

static
void hid_client_att_txn_init(struct hid_client_s *hid)
{
  assert(!hid->att_txn.task.destroy_func);

  hid->att_txn.task.destroy_func = hid_client_txn_destroyed;
}

static
void hid_client_find_service(struct hid_client_s *hid, const struct ble_uuid_s *service_type)
{
  hid_client_att_txn_init(hid);

  logk_debug("Find service "BLE_UUID_FMT"\n", BLE_UUID_ARG(service_type));

  hid->att_txn.command = BLE_ATT_FIND_BY_TYPE_VALUE_RQT;
  hid->att_txn.find_by_type_value.start = 1;
  hid->att_txn.find_by_type_value.end = -1;
  hid->att_txn.find_by_type_value.type = *BLE_UUID_BT_BASED_P(BLE_GATT_ATT_SERVICE_PRIMARY);
  if (ble_uuid_is_uuid16(service_type)) {
    hid->att_txn.find_by_type_value.value = (void*)&service_type->value16[6];
    hid->att_txn.find_by_type_value.value_size = 2;
  } else {
    hid->att_txn.find_by_type_value.value = (void*)service_type;
    hid->att_txn.find_by_type_value.value_size = 16;
  }
  hid->att_txn.find_by_type_value.information = (void*)hid->scratch;
  hid->att_txn.find_by_type_value.information_count = 0;
  hid->att_txn.find_by_type_value.information_max_count = sizeof(hid->scratch) / sizeof(struct ble_att_handle_information_s);

  net_task_query_push(&hid->att_txn.task,
                      hid->layer.parent, &hid->layer,
                      BLE_ATT_REQUEST);
}

static
void hid_client_find_chars(struct hid_client_s *hid)
{
  hid_client_att_txn_init(hid);

  logk_debug("Find chars %d-%d\n", hid->point, hid->service_group_end);

  hid->att_txn.command = BLE_ATT_READ_BY_TYPE_RQT;
  hid->att_txn.read_by_type.start = hid->point;
  hid->att_txn.read_by_type.end = hid->service_group_end;
  hid->att_txn.read_by_type.type = *BLE_UUID_BT_BASED_P(BLE_GATT_ATT_CHARACTERISTIC);
  hid->att_txn.read_by_type.handle_value = (void*)hid->scratch;
  hid->att_txn.read_by_type.handle_value_size = 0;
  hid->att_txn.read_by_type.handle_value_stride = 0;
  hid->att_txn.read_by_type.handle_value_size_max = sizeof(hid->scratch);

  net_task_query_push(&hid->att_txn.task,
                      hid->layer.parent, &hid->layer,
                      BLE_ATT_REQUEST);
}

static
void hid_client_read_by_type(struct hid_client_s *hid,
                             uint16_t start, uint16_t end,
                             const struct ble_uuid_s *type)
{
  hid_client_att_txn_init(hid);

  logk_debug("Read by type "BLE_UUID_FMT" %d-%d\n",
          BLE_UUID_ARG(type),
          hid->point, hid->service_group_end);

  hid->att_txn.command = BLE_ATT_READ_BY_TYPE_RQT;
  hid->att_txn.read_by_type.start = start;
  hid->att_txn.read_by_type.end = end;
  hid->att_txn.read_by_type.type = *type;
  hid->att_txn.read_by_type.handle_value = (void*)hid->scratch;
  hid->att_txn.read_by_type.handle_value_size = 0;
  hid->att_txn.read_by_type.handle_value_stride = 0;
  hid->att_txn.read_by_type.handle_value_size_max = sizeof(hid->scratch);

  net_task_query_push(&hid->att_txn.task,
                      hid->layer.parent, &hid->layer,
                      BLE_ATT_REQUEST);
}

static
void hid_client_cccd_subscribe(struct hid_client_s *hid,
                          uint16_t cccd, bool_t subs)
{
  hid_client_att_txn_init(hid);

  logk_debug("Subscribe to cccd %d: %d\n", cccd, subs);

  hid->att_txn.command = BLE_ATT_WRITE_RQT;
  hid->att_txn.write.handle = cccd;
  hid->att_txn.write.value_size = 2;
  hid->att_txn.write.value = (void*)hid->scratch;
  hid->att_txn.write.authenticated = 0;
  hid->att_txn.write.encrypted = 0;

  endian_le16_na_store(hid->scratch, subs ? BLE_GATT_CCCD_NOTIFICATION : 0);

  net_task_query_push(&hid->att_txn.task,
                      hid->layer.parent, &hid->layer,
                      BLE_ATT_REQUEST);
}

static
void hid_client_read_long(struct hid_client_s *hid,
                          void *target, size_t max_size,
                          uint16_t handle, uint16_t offset)
{
  hid_client_att_txn_init(hid);

  logk_debug("Read long %d from %d\n", handle, offset);

  hid->att_txn.command = BLE_ATT_READ_BLOB_RQT;
  hid->att_txn.read.handle = handle;
  hid->att_txn.read.offset = offset;
  hid->att_txn.read.value = target;
  hid->att_txn.read.value_size_max = max_size;

  net_task_query_push(&hid->att_txn.task,
                      hid->layer.parent, &hid->layer,
                      BLE_ATT_REQUEST);
}

static
void hid_client_read(struct hid_client_s *hid,
                     void *target, size_t max_size,
                     uint16_t handle)
{
  hid_client_att_txn_init(hid);

  logk_debug("Read att %d\n", handle);

  hid->att_txn.command = BLE_ATT_READ_RQT;
  hid->att_txn.read.handle = handle;
  hid->att_txn.read.value = target;
  hid->att_txn.read.offset = 0;
  hid->att_txn.read.value_size = 0;
  hid->att_txn.read.value_size_max = max_size;

  net_task_query_push(&hid->att_txn.task,
                      hid->layer.parent, &hid->layer,
                      BLE_ATT_REQUEST);
}

static const struct ble_uuid_s hid_report_ref_type
= BLE_UUID_BT_BASED(BLE_GATT_DESC_REPORT_REFERENCE);
static const struct ble_uuid_s hid_report_type
= BLE_UUID_BT_BASED(BLE_GATT_CHAR_REPORT);
static const struct ble_uuid_s hid_report_map_type
= BLE_UUID_BT_BASED(BLE_GATT_CHAR_REPORT_MAP);

static
void hid_client_request(struct hid_client_s *hid)
{
  static const struct ble_uuid_s gatt_hid_service_type
    = BLE_UUID_BT_BASED(BLE_GATT_SERVICE_HUMAN_INTERFACE_DEVICE);
  static const struct ble_uuid_s gatt_cccd_type
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_DESC_CCCD);
  static const struct ble_uuid_s dis_manufacturer_name_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_MANUFACTURER_NAME_STRING);
  static const struct ble_uuid_s dis_serial_number_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_SERIAL_NUMBER_STRING);
  static const struct ble_uuid_s dis_pnp_id_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_PNP_ID);
  static const struct ble_uuid_s gap_device_name_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_GAP_DEVICE_NAME);

 again:

  switch (hid->state) {
  case HC_IDLE:
    if (!hid->layer.parent) {
      hid->state = HC_FAIL;
      goto fail;
    } else {
      hid->state = HC_DISCOVER_HID;
      goto discover_hid;
    }

  case HC_DISCOVER_HID:
  discover_hid:
    hid_client_find_service(hid, &gatt_hid_service_type);
    return;

  case HC_DISCOVER_HID_CHARS:
    hid_client_find_chars(hid);
    return;

  case HC_DISCOVER_HID_COMPLETE:
    logk_debug("HID service discovery done\n");
    hid->state++;
    goto again;

  case HC_DISCOVER_HID_CCCD:
    hid_client_read_by_type(hid, hid->point, hid->service_group_end, &gatt_cccd_type);
    return;

  case HC_DISCOVER_HID_REPORT:
    hid_client_read_by_type(hid, hid->point, hid->service_group_end, &hid_report_ref_type);
    return;

  case HC_READ_PRODUCT:
    hid_client_read_by_type(hid, 1, -1, &gap_device_name_type);
    return;

  case HC_READ_SERIAL:
    hid_client_read_by_type(hid, 1, -1, &dis_serial_number_type);
    return;

  case HC_READ_VENDOR:
    hid_client_read_by_type(hid, 1, -1, &dis_manufacturer_name_type);
    return;

  case HC_READ_PNP_ID:
    hid_client_read_by_type(hid, 1, -1, &dis_pnp_id_type);
    return;

  case HC_READ_DESCRIPTOR:
    hid_client_read_long(hid, hid->descriptor, sizeof(hid->descriptor),
                         hid->descriptor_handle + 1, hid->descriptor_size);
    return;

  case HC_SUBSCRIBE:
    while (hid->point < hid->char_count) {
      if (!hid->chars[hid->point].cccd_handle) {
        logk_debug("No CCCD for handle %d\n", hid->chars[hid->point].handle);
        hid->point++;
        continue;
      }

      logk_debug("Subscription to CCCD %d\n", hid->chars[hid->point].cccd_handle);

      hid_client_cccd_subscribe(hid, hid->chars[hid->point].cccd_handle, 1);
      return;
    }

    logk_debug("Subscription done\n");
    hid->state++;
    goto again;

  case HC_FAIL:
  fail:
    logk_error("Remote client failed in state %d\n", hid->state);
    return;

  case HC_DISCOVERY_DONE: {
    hid->meta.descriptor_size = hid->descriptor_size;

    logk("Remote client done\n");
    logk(" \"%s\" \"%s\" Serial '%s' %04x:%04x v%d.%d\n",
           hid->meta.vendor, hid->meta.product,
           hid->meta.serial,
           hid->meta.vendor_id, hid->meta.product_id,
           hid->meta.hid_version >> 8, hid->meta.hid_version & 0xff);
    logk_debug(" HID Report descriptor: %P\n",
           hid->meta.descriptor, hid->meta.descriptor_size);
    const struct hid_client_delegate_vtable_s *vtable
      = const_hid_client_delegate_vtable_s_from_base(hid->layer.delegate_vtable);

    vtable->descriptor_discovered(hid->layer.delegate, &hid->layer, &hid->meta);
    return;
  }
  }
}

static void hid_read_done(struct hid_client_s *hid,
                          uint8_t error,
                          const void *data, size_t size)
{
  const struct hid_client_delegate_vtable_s *vtable
    = const_hid_client_delegate_vtable_s_from_base(hid->layer.delegate_vtable);

  if (error)
    vtable->report_read_error(hid->layer.delegate, &hid->layer);
  else
    vtable->report_read_done(hid->layer.delegate, &hid->layer, data, size);
}

static
void hid_client_reply_handle(struct hid_client_s *hid)
{
  struct ble_att_transaction_s *txn = &hid->att_txn;

  switch (hid->state) {
  case HC_DISCOVER_HID_COMPLETE:
  case HC_IDLE:
  case HC_FAIL:
  case HC_DISCOVERY_DONE:
    switch (txn->command) {
    case BLE_ATT_READ_RQT:
      hid_read_done(hid, txn->error, txn->read.value, txn->read.value_size);
      net_task_destroy(&txn->task);
      return;

    default:
      assert(0);
    }

  case HC_SUBSCRIBE:
    assert(txn->command == BLE_ATT_WRITE_RQT);
    hid->point++;
    goto again;

  case HC_DISCOVER_HID:
    assert(txn->command == BLE_ATT_FIND_BY_TYPE_VALUE_RQT);
    if (txn->error || txn->find_by_type_value.information_count != 1)
      goto fail;

    logk_debug("Service "BLE_UUID_FMT" is %d-%d\n",
            BLE_UUID_ARG(&txn->find_by_type_value.type),
            txn->find_by_type_value.information[0].found,
            txn->find_by_type_value.information[0].end_group);
    hid->service_handle = txn->find_by_type_value.information[0].found;
    hid->service_group_end = txn->find_by_type_value.information[0].end_group;
    hid->char_count = 0;
    goto next;

  case HC_DISCOVER_HID_CHARS: {
    assert(txn->command == BLE_ATT_READ_BY_TYPE_RQT);
    if (txn->error == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)
      goto next;

    if (txn->error)
      goto fail;

    uint16_t value_size = txn->read_by_type.handle_value_stride - 2;

    for (uint16_t offset = 0;
         offset < txn->read_by_type.handle_value_size;
         offset += txn->read_by_type.handle_value_stride) {
      const struct ble_att_handle_value_s *hv =
        (void*)(hid->scratch + offset);

      uint16_t handle = endian_16_na_load(&hv->handle);
      const uint8_t *value = hv->value;
      struct ble_uuid_s char_type;

      if (value_size == 5) {
        ble_uuid_bluetooth_based(&char_type, endian_le16_na_load(value + 3));
      } else {
        memcpy(&char_type, value + 3, 16);
      }

      logk_debug("  - Char type "BLE_UUID_FMT" at %d, value at %d, flags %02x\n",
              BLE_UUID_ARG(&char_type),
              handle, endian_le16_na_load(value + 1),
              value[0]);

      if (!ble_uuid_cmp(&char_type, &hid_report_type) && hid->char_count < HID_CLIENT_CHAR_MAX) {
        struct hid_report_s *ch = &hid->chars[hid->char_count++];
        ch->handle = handle;
        ch->cccd_handle = 0;
        ch->report_id = 0;
        ch->report_type = 0;

        if (hid->point >= hid->service_group_end)
          goto next;
      } else if (!ble_uuid_cmp(&char_type, &hid_report_map_type)) {
        hid->descriptor_handle = handle;
      }
      hid->point = handle + 2;
    }

    goto again;
  }

  case HC_DISCOVER_HID_REPORT: {
    assert(txn->command == BLE_ATT_READ_BY_TYPE_RQT);
    if (txn->error == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)
      goto next;

    if (txn->error)
      goto fail;

    __unused__
      uint16_t value_size = txn->read_by_type.handle_value_stride - 2;

    for (uint16_t offset = 0;
         offset < txn->read_by_type.handle_value_size;
         offset += txn->read_by_type.handle_value_stride) {
      const struct ble_att_handle_value_s *hv =
        (void*)(hid->scratch + offset);

      uint16_t handle = endian_16_na_load(&hv->handle);
      const uint8_t *value = hv->value;

      logk_debug("  - Report ref at %d: %P\n", handle, value, value_size);

      for (size_t char_no = 0; char_no < hid->char_count; ++char_no) {
        struct hid_report_s *ch = &hid->chars[char_no];

        if (handle < ch->handle)
          continue;

        if (char_no + 1 < hid->char_count
            && hid->chars[char_no + 1].handle < handle)
          continue;

        ch->report_id = value[0];
        ch->report_type = value[1];
        break;
      }

      hid->point = handle + 1;
      if (hid->point >= hid->service_group_end)
        goto next;
    }

    goto again;
  }

  case HC_DISCOVER_HID_CCCD: {
    assert(txn->command == BLE_ATT_READ_BY_TYPE_RQT);
    if (txn->error == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)
      goto next;

    if (txn->error)
      goto fail;

    __unused__
      uint16_t value_size = txn->read_by_type.handle_value_stride - 2;

    for (uint16_t offset = 0;
         offset < txn->read_by_type.handle_value_size;
         offset += txn->read_by_type.handle_value_stride) {
      const struct ble_att_handle_value_s *hv =
        (void*)(hid->scratch + offset);

      uint16_t handle = endian_16_na_load(&hv->handle);
      const uint8_t *value = hv->value;

      (void)value;
      (void)handle;
      logk_debug("  - CCCD at %d: %P\n", handle, value, value_size);

      for (size_t char_no = 0; char_no < hid->char_count; ++char_no) {
        struct hid_report_s *ch = &hid->chars[char_no];

        if (handle < ch->handle)
          continue;

        if (char_no + 1 < hid->char_count
            && hid->chars[char_no + 1].handle < handle)
          continue;

        ch->cccd_handle = handle;
        break;
      }

      hid->point = handle + 1;
      if (hid->point >= hid->service_group_end)
        goto next;
    }

    goto again;
  }

  case HC_READ_PRODUCT:
  case HC_READ_VENDOR:
  case HC_READ_SERIAL:
  case HC_READ_PNP_ID: {
    assert(txn->command == BLE_ATT_READ_BY_TYPE_RQT);
    if (txn->error)
      goto next;

    if (txn->read_by_type.handle_value_stride == 0
        || txn->read_by_type.handle_value_stride !=
        txn->read_by_type.handle_value_size)
      goto fail;

    uint16_t value_size = txn->read_by_type.handle_value_stride - 2;
    uint16_t handle = endian_16_na_load(hid->scratch);
    const uint8_t *value = hid->scratch + 2;

    (void)value_size;
    (void)handle;
    logk_debug("At %d value %S\n", handle, value, value_size);
        
    switch (hid->state) {
    case HC_READ_PRODUCT:
      strncpy(hid->meta.product, (const void*)value, __MIN(value_size, sizeof(hid->meta.product)));
      hid->meta.product[__MIN(value_size, sizeof(hid->meta.product) - 1)] = 0;
      break;

    case HC_READ_VENDOR:
      strncpy(hid->meta.vendor, (const void*)value, __MIN(value_size, sizeof(hid->meta.vendor)));
      hid->meta.vendor[__MIN(value_size, sizeof(hid->meta.vendor) - 1)] = 0;
      break;

    case HC_READ_SERIAL:
      strncpy(hid->meta.serial, (const void*)value, __MIN(value_size, sizeof(hid->meta.serial)));
      hid->meta.serial[__MIN(value_size, sizeof(hid->meta.serial) - 1)] = 0;
      break;

    case HC_READ_PNP_ID: {
      if (value_size != 7)
        goto fail;
      const struct ble_dis_pnp_id_s *id = (const void *)value;
      hid->meta.vendor_id = endian_le16_na_load(&id->vendor_id);
      hid->meta.product_id = endian_le16_na_load(&id->product_id);
      hid->meta.hid_version = endian_le16_na_load(&id->version);
      break;
    }

    case HC_READ_DESCRIPTOR:
    case HC_DISCOVER_HID:
    case HC_DISCOVER_HID_CHARS:
    case HC_DISCOVER_HID_REPORT:
    case HC_DISCOVER_HID_CCCD:
    case HC_DISCOVER_HID_COMPLETE:
    case HC_DISCOVERY_DONE:
    case HC_SUBSCRIBE:
    case HC_FAIL:
    case HC_IDLE:
      assert(0);
    }

    goto next;
  }

  case HC_READ_DESCRIPTOR:
    assert(txn->command == BLE_ATT_READ_BLOB_RQT);
    if (txn->error == BLE_ATT_ERR_INVALID_OFFSET
        || txn->read.offset == txn->read.value_size) {
      hid->point = 0;
      hid->state++;
      hid->att_txn.error = 0;
      goto again;
    }

    if (txn->error)
      goto fail;

    hid->descriptor_size = txn->read.value_size;
    goto again;
  }

 fail:
  logk_error("Remote client failure error %d\n", txn->error);
  logk_error("find_by_type_value count %d\n", txn->find_by_type_value.information_count);

  net_task_destroy(&txn->task);

  hid->state = HC_FAIL;

  return;

 next:
  hid->state++;
  hid->point = hid->service_handle;
  hid->att_txn.error = 0;
 again:
  net_task_destroy(&txn->task);
  hid_client_request(hid);
}

static
void hid_client_notif_handle(struct hid_client_s *hid,
                             uint16_t handle,
                             const void *data,
                             size_t size)
{
  const struct hid_client_delegate_vtable_s *vtable
    = const_hid_client_delegate_vtable_s_from_base(hid->layer.delegate_vtable);
  uint8_t report_id = 0;

  for (size_t i = 0; i < hid->char_count; ++i) {
    if (hid->chars[i].handle + 1 == handle
        && hid->chars[i].report_type == BLE_HID_REPORT_INPUT) {
      report_id = hid->chars[i].report_id;
      break;
    }
  }

  if (!report_id) {
    logk_warning("Att notification for unresolved handle %d: %P\n",
                 handle, data, size);
    return;
  }

  vtable->report_changed(hid->layer.delegate, &hid->layer,
                         report_id, data, size);
}

static
void hid_client_task_handle(struct net_layer_s *layer,
                            struct net_task_s *task)
{
  struct hid_client_s *hid = hid_client_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_QUERY:
    break;

  case NET_TASK_INBOUND:
    hid_client_notif_handle(hid, task->packet.src_addr.att,
                            task->packet.buffer->data + task->packet.buffer->begin,
                            task->packet.buffer->end - task->packet.buffer->begin);
    break;

  case NET_TASK_RESPONSE:
    if (task->query.opcode == BLE_ATT_REQUEST) {
      hid_client_reply_handle(hid);
      return;
    }
    break;

  case NET_TASK_TIMEOUT:
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

static void hid_client_context_changed(struct net_layer_s *layer)
{
  struct hid_client_s *hid = hid_client_s_from_layer(layer);

  if (hid->state == HC_IDLE)
    hid_client_request(hid);
}

static void hid_client_dangling(struct net_layer_s *layer)
{
  struct hid_client_s *hid = hid_client_s_from_layer(layer);
  (void)hid;
}

static
void hid_client_destroyed(struct net_layer_s *layer)
{
  struct hid_client_s *hid = hid_client_s_from_layer(layer);

  mem_free(hid);
}

static
void _client_descriptor_discover(struct net_layer_s *layer)
{
}

static
void _client_descriptor_serialize(struct net_layer_s *layer,
                                  void *target, size_t size)
{
}

static
void _client_report_set(struct net_layer_s *layer,
                        uint8_t report_id,
                        const void *data, size_t size)
{
  struct hid_client_s *hid = hid_client_s_from_layer(layer);
  uint16_t handle = 0;

  if (!layer->parent)
    return;

  for (size_t i = 0; i < hid->char_count; ++i) {
    if (hid->chars[i].report_id == report_id
        && hid->chars[i].report_type != BLE_HID_REPORT_INPUT) {
      handle = hid->chars[i].handle + 1;
      break;
    }
  }

  if (handle == 0)
    return;

  struct net_addr_s dst = {
    .att = handle,
  };

  struct buffer_s *packet = net_layer_packet_alloc(layer, layer->context.prefix_size, size);
  if (!packet)
    return;

  memcpy(packet->data + packet->begin, data, size);

  struct net_task_s *task = net_scheduler_task_alloc(layer->scheduler);

  if (task)
    net_task_outbound_push(task, layer->parent, layer, 0, NULL, &dst, packet);

  buffer_refdec(packet);
}

static
void _client_report_read(struct net_layer_s *layer,
                        uint8_t report_id)
{
  struct hid_client_s *hid = hid_client_s_from_layer(layer);
  uint16_t handle = 0;

  if (!layer->parent)
    return;

  for (size_t i = 0; i < hid->char_count; ++i) {
    if (hid->chars[i].report_id == report_id
        && hid->chars[i].report_type != BLE_HID_REPORT_OUTPUT) {
      handle = hid->chars[i].handle + 1;
      break;
    }
  }

  if (handle)
    hid_client_read(hid, hid->scratch, 20, handle);
  else
    hid_read_done(hid, 1, NULL, 0);
}

static const struct hid_client_handler_s hid_handler = {
  .base.destroyed = hid_client_destroyed,
  .base.task_handle = hid_client_task_handle,
  .base.context_changed = hid_client_context_changed,
  .base.dangling = hid_client_dangling,
  .descriptor_discover = _client_descriptor_discover,
  .descriptor_serialize = _client_descriptor_serialize,
  .report_set = _client_report_set,
  .report_read = _client_report_read,
};

error_t hid_client_create(struct net_scheduler_s *scheduler,
                          const void *params_,
                          void *delegate,
                          const struct net_layer_delegate_vtable_s *delegate_vtable,
                          struct net_layer_s **layer)
{
  struct hid_client_s *hid = mem_alloc(sizeof(*hid), mem_scope_sys);

  if (!hid)
    return -ENOMEM;

  memset(hid, 0, sizeof(*hid));

  hid->meta.descriptor = hid->descriptor;
  strcpy(hid->meta.vendor, "MutekH");
  strcpy(hid->meta.product, "HID bridge");
  strcpy(hid->meta.serial, "Unknown");
  hid->meta.vendor_id = 0x10eb;
  hid->meta.product_id = 0x0099;
  hid->meta.hid_version = 0x0101;

  error_t err = net_layer_init(&hid->layer, &hid_handler.base, scheduler,
                               delegate, delegate_vtable);
  if (err) {
    mem_free(hid);
    return err;
  }

  *layer = &hid->layer;
  hid->state = HC_IDLE;

  return 0;
}

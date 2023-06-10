#define LOGK_MODULE_ID "oads"

#include <mutek/printk.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <hexo/power.h>

#include <net/layer.h>
#include <net/task.h>
#include <net/scheduler.h>

#include <stdio.h>

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

#include <ble/net/generic.h>
#include <ble/profile/oad/client.h>
#include <ble/profile/oad/ids.h>

#define OAD_CLIENT_CHAR_MAX 4

enum oad_client_state_e
{
  OAD_IDLE,
  OAD_DISCOVER_OAD,
  OAD_DISCOVER_OAD_CHARS,
  OAD_DISCOVER_OAD_CCCD,
  OAD_DISCOVER_OAD_COMPLETE,
  OAD_READ_FW_VERSION,
  OAD_READ_SW_VERSION,
  OAD_READ_MODEL,
  OAD_READ_INFO,
  OAD_SUBSCRIBE_STATUS,
  OAD_DISCOVERY_DONE,
  OAD_FAIL,
};

struct oad_client_char_s
{
  struct ble_uuid_s type;
  uint16_t handle;
  uint16_t cccd;
  uint16_t flags;
};

struct oad_client_s
{
    struct net_layer_s layer;
    uint16_t service_handle;
    uint16_t point;
    uint16_t service_group_end;
    struct oad_client_char_s chars[OAD_CLIENT_CHAR_MAX];
    size_t char_count;

    uint32_t availability;
    
    uint16_t payload_idx;
    uint16_t status_idx;
    uint16_t info_idx;
    uint16_t command_idx;
  
    enum oad_client_state_e state;

    struct ble_att_transaction_s att_txn;
    uint8_t scratch[64];

    char model[32];
    char fw_version[32];
    char sw_version[32];
};

STRUCT_COMPOSE(oad_client_s, layer);
STRUCT_COMPOSE(oad_client_s, att_txn);

static void oad_client_txn_destroyed(void *task)
{
  struct ble_att_transaction_s *txn = ble_att_transaction_s_from_task(task);
  struct oad_client_s *oad = oad_client_s_from_att_txn(txn);

  memset(&oad->att_txn, 0x55, sizeof(oad->att_txn));

  oad->att_txn.task.destroy_func = NULL;
}

static
void oad_client_att_txn_init(struct oad_client_s *oad)
{
  assert(!oad->att_txn.task.destroy_func);

  memset(&oad->att_txn, 0, sizeof(oad->att_txn));

  oad->att_txn.task.destroy_func = oad_client_txn_destroyed;
}

static
void oad_client_find_service(struct oad_client_s *oad, const struct ble_uuid_s *service_type)
{
  oad_client_att_txn_init(oad);

  logk_trace("Find service "BLE_UUID_FMT"", BLE_UUID_ARG(service_type));

  oad->att_txn.command = BLE_ATT_FIND_BY_TYPE_VALUE_RQT;
  oad->att_txn.find_by_type_value.start = 1;
  oad->att_txn.find_by_type_value.end = -1;
  oad->att_txn.find_by_type_value.type = *BLE_UUID_BT_BASED_P(BLE_GATT_ATT_SERVICE_PRIMARY);
  if (ble_uuid_is_uuid16(service_type)) {
    oad->att_txn.find_by_type_value.value = (void*)&service_type->value16[6];
    oad->att_txn.find_by_type_value.value_size = 2;
  } else {
    oad->att_txn.find_by_type_value.value = (void*)service_type;
    oad->att_txn.find_by_type_value.value_size = 16;
  }
  oad->att_txn.find_by_type_value.information = (void*)oad->scratch;
  oad->att_txn.find_by_type_value.information_count = 0;
  oad->att_txn.find_by_type_value.information_max_count = sizeof(oad->scratch) / sizeof(struct ble_att_handle_information_s);

  net_task_query_push(&oad->att_txn.task,
                      oad->layer.parent, &oad->layer,
                      BLE_ATT_REQUEST);
}

static
void oad_client_find_information(struct oad_client_s *oad,
                                 uint16_t start, uint16_t end)
{
  oad_client_att_txn_init(oad);

  logk_trace("Find information %d -> %d", start, end);

  oad->att_txn.command = BLE_ATT_FIND_INFORMATION_RQT;
  oad->att_txn.find_information.start = start;
  oad->att_txn.find_information.end = end;
  oad->att_txn.find_information.information = (void*)oad->scratch;
  oad->att_txn.find_information.information_count = 0;
  oad->att_txn.find_information.information_max_count = sizeof(oad->scratch) / sizeof(struct ble_att_handle_information_s);

  net_task_query_push(&oad->att_txn.task,
                      oad->layer.parent, &oad->layer,
                      BLE_ATT_REQUEST);
}

static
void oad_client_find_chars(struct oad_client_s *oad)
{
  oad_client_att_txn_init(oad);

  logk_trace("Find chars %d-%d", oad->point, oad->service_group_end);

  oad->att_txn.command = BLE_ATT_READ_BY_TYPE_RQT;
  oad->att_txn.read_by_type.start = oad->point;
  oad->att_txn.read_by_type.end = oad->service_group_end;
  oad->att_txn.read_by_type.type = *BLE_UUID_BT_BASED_P(BLE_GATT_ATT_CHARACTERISTIC);
  oad->att_txn.read_by_type.handle_value = (void*)oad->scratch;
  oad->att_txn.read_by_type.handle_value_size = 0;
  oad->att_txn.read_by_type.handle_value_stride = 0;
  oad->att_txn.read_by_type.handle_value_size_max = sizeof(oad->scratch);

  net_task_query_push(&oad->att_txn.task,
                      oad->layer.parent, &oad->layer,
                      BLE_ATT_REQUEST);
}

static
void oad_client_read_by_type(struct oad_client_s *oad,
                             uint16_t start, uint16_t end,
                             const struct ble_uuid_s *type)
{
  oad_client_att_txn_init(oad);

  logk_trace("Read by type "BLE_UUID_FMT" %d-%d",
          BLE_UUID_ARG(type),
          oad->point, oad->service_group_end);

  oad->att_txn.command = BLE_ATT_READ_BY_TYPE_RQT;
  oad->att_txn.read_by_type.start = start;
  oad->att_txn.read_by_type.end = end;
  oad->att_txn.read_by_type.type = *type;
  oad->att_txn.read_by_type.handle_value = (void*)oad->scratch;
  oad->att_txn.read_by_type.handle_value_size = 0;
  oad->att_txn.read_by_type.handle_value_stride = 0;
  oad->att_txn.read_by_type.handle_value_size_max = sizeof(oad->scratch);

  net_task_query_push(&oad->att_txn.task,
                      oad->layer.parent, &oad->layer,
                      BLE_ATT_REQUEST);
}

static
void oad_client_read_request(struct oad_client_s *oad,
                               uint16_t handle)
{
  oad_client_att_txn_init(oad);

  logk_trace("Read by handle %d", handle);

  oad->att_txn.command = BLE_ATT_READ_RQT;
  oad->att_txn.read.handle = handle;
  oad->att_txn.read.value = (void*)oad->scratch;
  oad->att_txn.read.value_size_max = sizeof(oad->scratch);
  oad->att_txn.read.value_size = 0;
  oad->att_txn.read.offset = 0;

  net_task_query_push(&oad->att_txn.task,
                      oad->layer.parent, &oad->layer,
                      BLE_ATT_REQUEST);
}

static
void oad_client_subscribe(struct oad_client_s *oad,
                              uint16_t cccd, bool_t subs)
{
    oad_client_att_txn_init(oad);

    logk_trace("Subscribe to cccd %d: %d", cccd, subs);

    oad->att_txn.command = BLE_ATT_WRITE_RQT;
    oad->att_txn.write.handle = cccd;
    oad->att_txn.write.value_size = 2;
    oad->att_txn.write.value = (void*)oad->scratch;
    oad->att_txn.write.authenticated = 0;
    oad->att_txn.write.encrypted = 0;

    endian_le16_na_store(oad->scratch, subs ? BLE_GATT_CCCD_NOTIFICATION : 0);

    net_task_query_push(&oad->att_txn.task,
                        oad->layer.parent, &oad->layer,
                        BLE_ATT_REQUEST);
}

static
void oad_client_request(struct oad_client_s *oad)
{
  static const struct ble_uuid_s dis_firmware_revision_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_FIRMWARE_REVISION_STRING);
  static const struct ble_uuid_s dis_software_revision_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_SOFTWARE_REVISION_STRING);
  static const struct ble_uuid_s dis_model_number_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_MODEL_NUMBER_STRING);

  static const struct ble_uuid_s oad_service_type = OAD_SERVICE_TYPE;
  static const struct ble_uuid_s oad_char_status = OAD_CHAR_STATUS_TYPE;
  static const struct ble_uuid_s oad_char_payload = OAD_CHAR_PAYLOAD_TYPE;
  static const struct ble_uuid_s oad_char_info = OAD_CHAR_INFO_TYPE;
  static const struct ble_uuid_s oad_char_command = OAD_CHAR_COMMAND_TYPE;

 again:

  switch (oad->state) {
  case OAD_IDLE:
    if (!oad->layer.parent) {
      logk_trace("No parent, failing");
      oad->state = OAD_FAIL;
      goto fail;
    } else {
      oad->state = OAD_DISCOVER_OAD;
      logk_trace("Starting discovery");
      goto discover_oad;
    }

  case OAD_DISCOVER_OAD:
  discover_oad:
    oad_client_find_service(oad, &oad_service_type);
    return;

  case OAD_DISCOVER_OAD_CHARS:
    oad_client_find_chars(oad);
    return;

  case OAD_DISCOVER_OAD_COMPLETE:
    logk_trace("OAD service discovery done");
    for (size_t i = 0; i < oad->char_count; ++i) {
      struct oad_client_char_s *ch = &oad->chars[i];

      logk_trace("- %d "BLE_UUID_FMT" value at %d, cccd at %d",
                 ch->handle, BLE_UUID_ARG(&ch->type),
                 ch->handle + 1, ch->cccd);

      if (!ble_uuid_cmp(&oad_char_status, &ch->type)) {
        logk_trace("  is status");
        oad->status_idx = i;
      } else if (!ble_uuid_cmp(&oad_char_payload, &ch->type)) {
        logk_trace("  is payload");
        oad->payload_idx = i;
      } else if (!ble_uuid_cmp(&oad_char_info, &ch->type)) {
        logk_trace("  is info");
        oad->info_idx = i;
      } else if (!ble_uuid_cmp(&oad_char_command, &ch->type)) {
        logk_trace("  is command");
        oad->command_idx = i;
      }
    }

    if (oad->status_idx == 0xffff
        || oad->payload_idx == 0xffff
        || oad->info_idx == 0xffff
        || oad->command_idx == 0xffff)
      goto fail;

    oad->state++;
    goto again;

  case OAD_READ_INFO:
    oad_client_read_request(oad, &oad->chars[oad->info_idx].handle + 1);
    return;

  case OAD_DISCOVER_OAD_CCCD:
    oad_client_find_information(oad, oad->point, oad->service_group_end);
    return;

  case OAD_READ_FW_VERSION:
    oad_client_read_by_type(oad, 1, -1, &dis_firmware_revision_type);
    return;

  case OAD_READ_SW_VERSION:
    oad_client_read_by_type(oad, 1, -1, &dis_software_revision_type);
    return;

  case OAD_READ_MODEL:
    oad_client_read_by_type(oad, 1, -1, &dis_model_number_type);
    return;

  case OAD_SUBSCRIBE_STATUS:
    oad_client_subscribe(oad, oad->chars[oad->status_idx].cccd, 1);
    return;

  case OAD_FAIL:
  fail:
    logk_error("Oad client failed in state %d", oad->state);
    return;

  case OAD_DISCOVERY_DONE: {
    if (oad->info.chunk_size_log2 != OAD_CHUNK_SIZE_L2)
      goto fail;
    
    logk_trace("Oad client done");
    const struct oad_client_delegate_vtable_s *vtable
      = const_oad_client_delegate_vtable_s_from_base(oad->layer.delegate_vtable);

    vtable->discovery_done(oad->layer.delegate, &oad->layer,
                           oad->model,
                           oad->fw_version, oad->sw_version,
                           oad->info.storage_size);
    return;
  }
  }
}

static
void oad_client_reply_handle(struct oad_client_s *oad)
{
  static const struct ble_uuid_s gatt_cccd_type
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_DESC_CCCD);

  struct ble_att_transaction_s *txn = &oad->att_txn;

  switch (oad->state) {
  case OAD_DISCOVER_OAD:
    assert(txn->command == BLE_ATT_FIND_BY_TYPE_VALUE_RQT);
    if (txn->error || txn->find_by_type_value.information_count != 1)
      goto fail;

    logk_trace("Service "BLE_UUID_FMT" is %d-%d",
            BLE_UUID_ARG(&txn->find_by_type_value.type),
            txn->find_by_type_value.information[0].found,
            txn->find_by_type_value.information[0].end_group);
    oad->service_handle = txn->find_by_type_value.information[0].found;
    oad->service_group_end = txn->find_by_type_value.information[0].end_group;
    oad->char_count = 0;
    goto next;

  case OAD_DISCOVER_OAD_CHARS: {
    assert(txn->command == BLE_ATT_READ_BY_TYPE_RQT);
    if (txn->error == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)
      goto next;

    if (txn->error)
      goto fail;

    if (txn->read_by_type.handle_value_size == 0)
      goto next;

    uint16_t value_size = txn->read_by_type.handle_value_stride - 2;

    for (uint16_t offset = 0;
         offset < txn->read_by_type.handle_value_size;
         offset += txn->read_by_type.handle_value_stride) {
      const struct ble_att_handle_value_s *hv =
        (void*)(oad->scratch + offset);

      uint16_t handle = endian_16_na_load(&hv->handle);
      const uint8_t *value = hv->value;
      struct ble_uuid_s char_type;

      if (value_size == 5) {
        ble_uuid_bluetooth_based(&char_type, endian_le16_na_load(value + 3));
      } else {
        memcpy(&char_type, value + 3, 16);
      }

      logk_trace("  - Char type "BLE_UUID_FMT" at %d, value at %d, flags %02x",
              BLE_UUID_ARG(&char_type),
              handle, endian_le16_na_load(value + 1),
              value[0]);

      struct oad_client_char_s *ch = &oad->chars[oad->char_count++];
      ch->type = char_type;
      ch->handle = handle;
      ch->flags = value[0];
      ch->cccd = 0;

      oad->point = handle + 2;
      if (oad->point >= oad->service_group_end)
        goto next;
    }

    goto again;
  }

  case OAD_FAIL:
  case OAD_IDLE:
  case OAD_DISCOVERY_DONE:
  case OAD_DISCOVER_OAD_COMPLETE:
    break;

  case OAD_SUBSCRIBE_PROBE_TEMP:
  case OAD_SUBSCRIBE_PROBE_ATTITUDE:
  case OAD_SUBSCRIBE_COIL_ENVELOPE:
  case OAD_SUBSCRIBE_COIL_TEMPERATURE:
    if (txn->error)
      goto fail;
    goto next;

  case OAD_DISCOVER_OAD_CCCD: {
    assert(txn->command == BLE_ATT_FIND_INFORMATION_RQT);
    if (txn->error == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)
      goto next;

    if (txn->error)
      goto fail;

    if (txn->find_information.information_count == 0)
        goto next;
    
    for (uint16_t i = 0; i < txn->find_information.information_count; ++i) {
      const struct ble_att_information_s *inf = txn->find_information.information + i;

      oad->point = inf->handle + 1;

      if (ble_uuid_cmp(&gatt_cccd_type, &inf->type))
        continue;

      logk_trace("  - CCCD at %d", inf->handle);

      for (size_t char_no = 0; char_no < oad->char_count; ++char_no) {
        struct oad_client_char_s *ch = &oad->chars[char_no];

        if (inf->handle < ch->handle)
          continue;

        if (char_no + 1 < oad->char_count
            && oad->chars[char_no + 1].handle < inf->handle)
          continue;

        ch->cccd = inf->handle;
        break;
      }
    }

    if (oad->point >= oad->service_group_end)
      goto next;

    goto again;
  }

  case OAD_READ_INFO: {
    assert(txn->command == BLE_ATT_READ_RQT);
    if (txn->error)
      goto fail;

    if (txn->read.value_size != sizeof(oad->info))
      goto fail;

    memcpy(&oad->info, txn->read.value, txn->read.size);
    goto next;
  }
    
  case OAD_READ_MODEL:
  case OAD_READ_SW_VERSION:
  case OAD_READ_FW_VERSION: {
    assert(txn->command == BLE_ATT_READ_BY_TYPE_RQT);
    if (txn->error)
      goto fail;

    if (txn->read_by_type.handle_value_stride == 0
        || txn->read_by_type.handle_value_stride !=
        txn->read_by_type.handle_value_size)
      goto fail;

    uint16_t value_size = txn->read_by_type.handle_value_stride - 2;
    uint16_t handle = endian_16_na_load(oad->scratch);
    const char *value = (const char*)oad->scratch + 2;

    logk_trace("At %d value %S", handle, value, value_size);
        
    switch (oad->state) {
    case OAD_READ_MODEL:
      strncpy(oad->model, value, __MIN(value_size, sizeof(oad->model)));
      oad->model[__MIN(value_size, sizeof(oad->model) - 1)] = 0;
      break;

    case OAD_READ_FW_VERSION:
      strncpy(oad->fw_version, value, __MIN(value_size, sizeof(oad->fw_version)));
      oad->fw_version[__MIN(value_size, sizeof(oad->fw_version) - 1)] = 0;
      break;

    case OAD_READ_SW_VERSION:
      strncpy(oad->sw_version, value, __MIN(value_size, sizeof(oad->sw_version)));
      oad->sw_version[__MIN(value_size, sizeof(oad->sw_version) - 1)] = 0;
      break;

    case OAD_READ_SERIAL:
      strncpy(oad->serial, value, __MIN(value_size, sizeof(oad->serial)));
      oad->serial[__MIN(value_size, sizeof(oad->serial) - 1)] = 0;
      break;

    default:
      break;
    }

    goto next;
  }
  }

 fail:
  logk_error("Oad client failure state %d error %d", oad->state, txn->error);
  logk_error("find_by_type_value count %d", txn->find_by_type_value.information_count);

  net_task_destroy(&txn->task);

  oad->state = OAD_FAIL;

  return;

 next:
  oad->state++;
  oad->point = oad->service_handle;
  oad->att_txn.error = 0;
 again:
  net_task_destroy(&txn->task);
  oad_client_request(oad);
  return;
 wait:
  net_task_destroy(&txn->task);
}

static void oad_notif_handle(struct oad_client_s *oad,
                               uint16_t att,
                               const uint8_t *data, size_t size)
{
  const struct oad_client_delegate_vtable_s *vtable
      = const_oad_client_delegate_vtable_s_from_base(oad->layer.delegate_vtable);
  
  logk_trace("Notif, handle %d, value %P", att, data, size);

  if (size == 4
      && oad->probe_temp_idx != 0xffff
      && oad->chars[oad->probe_temp_idx].handle + 1 == att) {
      uint32_t temp = endian_le32_na_load(data);

      logk_debug("Probe temperature %d mK", temp);

      vtable->probe_temp_changed(oad->layer.delegate, &oad->layer,
                                 temp);
  }
}

static
void oad_client_task_handle(struct net_layer_s *layer,
                              struct net_task_s *task)
{
  struct oad_client_s *oad = oad_client_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_QUERY:
    break;

  case NET_TASK_INBOUND: {
    oad_notif_handle(oad, task->packet.src_addr.att,
                       task->packet.buffer->data + task->packet.buffer->begin,
                       task->packet.buffer->end - task->packet.buffer->begin);
    break;
  }

  case NET_TASK_RESPONSE:
    if (task->query.opcode == BLE_ATT_REQUEST) {
      assert(task == &oad->att_txn.task);
      oad_client_reply_handle(oad);
      return;
    }
    break;

  case NET_TASK_TIMEOUT:
    logk_trace("Wait over !");
    oad->state++;
    oad_client_request(oad);
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

static void oad_client_context_changed(struct net_layer_s *layer)
{
  struct oad_client_s *oad = oad_client_s_from_layer(layer);

  logk_trace("%s %d", __func__, oad->state);
  
  if (oad->state == OAD_IDLE)
    oad_client_request(oad);
}

static void oad_client_dangling(struct net_layer_s *layer)
{
  struct oad_client_s *oad = oad_client_s_from_layer(layer);

  (void)oad;
}

static
void oad_client_destroyed(struct net_layer_s *layer)
{
  struct oad_client_s *oad = oad_client_s_from_layer(layer);

  mem_free(oad);
}

static
error_t oad_client_payload_set(struct net_layer_s *layer,
                               uint32_t offset, const void *data)
{
  struct oad_client_s *oad = oad_client_s_from_layer(layer);

  if (!layer->parent)
    return -EIO;

  if (oad->payload_idx == 0xffff)
    return -ENOENT;

  if (oad->att_txn.task.destroy_func)
    return -ENOMEM;

  uint16_t handle = oad->chars[oad->status_indicator_idx].handle + 1;
  if (handle == 0)
    return -ENOENT;

  struct net_addr_s dst = {
    .att = handle,
  };
  struct buffer_s *packet = net_layer_packet_alloc(layer, layer->context.prefix_size, sizeof(struct oad_payload_s));
  if (!packet)
    return;

  endian_le16_na_store(&packet->data[packet->begin], offset);
  memcpy(&packet->data[packet->begin + 4], data, OAD_CHUNK_SIZE);

  struct net_task_s *task = &oad->att_txn.task;

  oad_client_att_txn_init(oad);
  net_task_outbound_push(task, layer->parent, layer, 0, NULL, &dst, packet);
  buffer_refdec(packet);
}

static
error_t oad_client_command_set(struct net_layer_s *layer,
                               uint8_t command, uint32_t arg)
{
  struct oad_client_s *oad = oad_client_s_from_layer(layer);

  if (!layer->parent)
    return -EIO;

  if (oad->command_idx == 0xffff)
    return -ENOENT;

  if (oad->att_txn.task.destroy_func)
    return -ENOMEM;

  uint16_t handle = oad->chars[oad->status_indicator_idx].handle + 1;
  if (handle == 0)
    return -ENOENT;

  struct net_addr_s dst = {
    .att = handle,
  };
  struct buffer_s *packet = net_layer_packet_alloc(layer, layer->context.prefix_size, sizeof(struct oad_command_s));
  if (!packet)
    return;

  packet->data[packet->begin] = command;
  endian_le32_na_load(&packet->data[packet->begin + 1], arg);

  struct net_task_s *task = &oad->att_txn.task;

  oad_client_att_txn_init(oad);
  net_task_outbound_push(task, layer->parent, layer, 0, NULL, &dst, packet);
  buffer_refdec(packet);
}

static const struct oad_client_handler_s oad_handler = {
  .base.destroyed = oad_client_destroyed,
  .base.task_handle = oad_client_task_handle,
  .base.context_changed = oad_client_context_changed,
  .base.dangling = oad_client_dangling,
  .payload_set = oad_client_payload_set,
  .command_set = oad_client_command_set,
};

error_t oad_client_create(struct net_scheduler_s *scheduler,
                          const void *params_,
                          void *delegate,
                          const struct oad_client_delegate_vtable_s *delegate_vtable,
                          struct net_layer_s **layer)
{
  struct oad_client_s *oad = mem_alloc(sizeof(*oad), mem_scope_sys);

  if (!oad)
    return -ENOMEM;

  memset(oad, 0, sizeof(*oad));

  error_t err = net_layer_init(&oad->layer, &oad_handler.base, scheduler,
                               delegate, &delegate_vtable->base);
  if (err) {
    mem_free(oad);
    return err;
  }

  *layer = &oad->layer;
  oad->state = OAD_IDLE;
    
  oad->status_idx = 0xffff;
  oad->payload_idx = 0xffff;
  oad->info_idx = 0xffff;
  oad->command_idx = 0xffff;

  return 0;
}

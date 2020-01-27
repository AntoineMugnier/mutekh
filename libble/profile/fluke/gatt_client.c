#define LOGK_MODULE_ID "fluk"

#include <mutek/printk.h>
#include <mutek/buffer_pool.h>

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
#include <ble/security_db.h>

#include <ble/net/generic.h>

#include "gatt_client.h"
#include <ble/profile/fluke/measurement.h>

#define FLUKE_GATT_CLIENT_CHAR_MAX 4

enum fluke_gatt_client_state_e
{
  FC_IDLE,
  FC_DISCOVER_FLUKE,
  FC_DISCOVER_FLUKE_CHARS,
  FC_DISCOVER_FLUKE_CCCD,
  FC_DISCOVER_FLUKE_COMPLETE,
  FC_READ_SERIAL,
  FC_READ_FW_VERSION,
  FC_READ_NAME,
  FC_FLUKE_SUBSCRIBE_BINARY,
  FC_DISCOVERY_DONE,
  FC_FAIL,
};

struct fluke_gatt_client_char_s
{
  struct ble_uuid_s type;
  uint16_t handle;
  uint16_t cccd;
  uint16_t flags;
};

struct fluke_gatt_client_s
{
  struct net_layer_s layer;
  uint16_t service_handle;
  uint16_t point;
  uint16_t service_group_end;
  struct fluke_gatt_client_char_s chars[FLUKE_GATT_CLIENT_CHAR_MAX];
  size_t char_count;

  uint16_t bin_handle;
  uint16_t bin_cccd_handle;
  
  enum fluke_gatt_client_state_e state;
  struct fluke_measurement_s last_measurement;

  struct ble_att_transaction_s att_txn;
  uint8_t scratch[64];

  char name[32];
  char serial[32];
  char fw_version[32];
};

STRUCT_COMPOSE(fluke_gatt_client_s, layer);
STRUCT_COMPOSE(fluke_gatt_client_s, att_txn);

static void fluke_gatt_client_txn_destroyed(void *task)
{
  struct ble_att_transaction_s *txn = ble_att_transaction_s_from_task(task);
  struct fluke_gatt_client_s *fluke = fluke_gatt_client_s_from_att_txn(txn);

  fluke->att_txn.task.destroy_func = NULL;
}

static
void fluke_gatt_client_att_txn_init(struct fluke_gatt_client_s *fluke)
{
  assert(!fluke->att_txn.task.destroy_func);

  fluke->att_txn.task.destroy_func = fluke_gatt_client_txn_destroyed;
}

static
void fluke_gatt_client_find_service(struct fluke_gatt_client_s *fluke, const struct ble_uuid_s *service_type)
{
  fluke_gatt_client_att_txn_init(fluke);

  logk_trace("Find service "BLE_UUID_FMT"", BLE_UUID_ARG(service_type));

  fluke->att_txn.command = BLE_ATT_FIND_BY_TYPE_VALUE_RQT;
  fluke->att_txn.find_by_type_value.start = 1;
  fluke->att_txn.find_by_type_value.end = -1;
  fluke->att_txn.find_by_type_value.type = *BLE_UUID_BT_BASED_P(BLE_GATT_ATT_SERVICE_PRIMARY);
  if (ble_uuid_is_uuid16(service_type)) {
    fluke->att_txn.find_by_type_value.value = (void*)&service_type->value16[6];
    fluke->att_txn.find_by_type_value.value_size = 2;
  } else {
    fluke->att_txn.find_by_type_value.value = (void*)service_type;
    fluke->att_txn.find_by_type_value.value_size = 16;
  }
  fluke->att_txn.find_by_type_value.information = (void*)fluke->scratch;
  fluke->att_txn.find_by_type_value.information_count = 0;
  fluke->att_txn.find_by_type_value.information_max_count = sizeof(fluke->scratch) / sizeof(struct ble_att_handle_information_s);

  net_task_query_push(&fluke->att_txn.task,
                      fluke->layer.parent, &fluke->layer,
                      BLE_ATT_REQUEST);
}

static
void fluke_gatt_client_find_information(struct fluke_gatt_client_s *fluke,
                                     uint16_t start, uint16_t end)
{
  fluke_gatt_client_att_txn_init(fluke);

  logk_trace("Find information %d -> %d", start, end);

  fluke->att_txn.command = BLE_ATT_FIND_INFORMATION_RQT;
  fluke->att_txn.find_information.start = start;
  fluke->att_txn.find_information.end = end;
  fluke->att_txn.find_information.information = (void*)fluke->scratch;
  fluke->att_txn.find_information.information_count = 0;
  fluke->att_txn.find_information.information_max_count = sizeof(fluke->scratch) / sizeof(struct ble_att_handle_information_s);

  net_task_query_push(&fluke->att_txn.task,
                      fluke->layer.parent, &fluke->layer,
                      BLE_ATT_REQUEST);
}

static
void fluke_gatt_client_find_chars(struct fluke_gatt_client_s *fluke)
{
  fluke_gatt_client_att_txn_init(fluke);

  logk_trace("Find chars %d-%d", fluke->point, fluke->service_group_end);

  fluke->att_txn.command = BLE_ATT_READ_BY_TYPE_RQT;
  fluke->att_txn.read_by_type.start = fluke->point;
  fluke->att_txn.read_by_type.end = fluke->service_group_end;
  fluke->att_txn.read_by_type.type = *BLE_UUID_BT_BASED_P(BLE_GATT_ATT_CHARACTERISTIC);
  fluke->att_txn.read_by_type.handle_value = (void*)fluke->scratch;
  fluke->att_txn.read_by_type.handle_value_size = 0;
  fluke->att_txn.read_by_type.handle_value_stride = 0;
  fluke->att_txn.read_by_type.handle_value_size_max = sizeof(fluke->scratch);

  net_task_query_push(&fluke->att_txn.task,
                      fluke->layer.parent, &fluke->layer,
                      BLE_ATT_REQUEST);
}

static
void fluke_gatt_client_read_by_type(struct fluke_gatt_client_s *fluke,
                             uint16_t start, uint16_t end,
                             const struct ble_uuid_s *type)
{
  fluke_gatt_client_att_txn_init(fluke);

  logk_trace("Read by type "BLE_UUID_FMT" %d-%d",
          BLE_UUID_ARG(type),
          fluke->point, fluke->service_group_end);

  fluke->att_txn.command = BLE_ATT_READ_BY_TYPE_RQT;
  fluke->att_txn.read_by_type.start = start;
  fluke->att_txn.read_by_type.end = end;
  fluke->att_txn.read_by_type.type = *type;
  fluke->att_txn.read_by_type.handle_value = (void*)fluke->scratch;
  fluke->att_txn.read_by_type.handle_value_size = 0;
  fluke->att_txn.read_by_type.handle_value_stride = 0;
  fluke->att_txn.read_by_type.handle_value_size_max = sizeof(fluke->scratch);

  net_task_query_push(&fluke->att_txn.task,
                      fluke->layer.parent, &fluke->layer,
                      BLE_ATT_REQUEST);
}

static
void fluke_gatt_client_subscribe(struct fluke_gatt_client_s *fluke,
                              uint16_t cccd, bool_t subs)
{
    fluke_gatt_client_att_txn_init(fluke);

    logk_trace("Subscribe to cccd %d: %d", cccd, subs);

    fluke->att_txn.command = BLE_ATT_WRITE_RQT;
    fluke->att_txn.write.handle = cccd;
    fluke->att_txn.write.value_size = 2;
    fluke->att_txn.write.value = (void*)fluke->scratch;
    fluke->att_txn.write.authenticated = 0;
    fluke->att_txn.write.encrypted = 0;

    endian_le16_na_store(fluke->scratch, subs ? BLE_GATT_CCCD_NOTIFICATION : 0);

    net_task_query_push(&fluke->att_txn.task,
                        fluke->layer.parent, &fluke->layer,
                        BLE_ATT_REQUEST);
}

static
void fluke_gatt_client_request(struct fluke_gatt_client_s *fluke)
{
  static const struct ble_uuid_s gatt_fluke_service_type
    = BLE_UUID(0xb6981800, 0x7562, 0x11e2, 0xb50d, 0x00163e46f8feULL);
  static const struct ble_uuid_s dis_serial_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_SERIAL_NUMBER_STRING);
  static const struct ble_uuid_s dis_firmware_revision_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_FIRMWARE_REVISION_STRING);
  static const struct ble_uuid_s gap_device_name_type
    = BLE_UUID_BT_BASED(BLE_GATT_CHAR_GAP_DEVICE_NAME);
  static const struct ble_uuid_s gatt_fluke_reading_char_type
    = BLE_UUID(0xb698290f, 0x7562, 0x11e2, 0xb50d, 0x00163e46f8feULL);

 again:

  switch (fluke->state) {
  case FC_IDLE:
    if (!fluke->layer.parent) {
      fluke->state = FC_FAIL;
      goto fail;
    } else {
      fluke->state = FC_DISCOVER_FLUKE;
      goto discover_fluke;
    }

  case FC_DISCOVER_FLUKE:
  discover_fluke:
    fluke_gatt_client_find_service(fluke, &gatt_fluke_service_type);
    return;

  case FC_DISCOVER_FLUKE_CHARS:
    fluke_gatt_client_find_chars(fluke);
    return;

  case FC_DISCOVER_FLUKE_COMPLETE:
    logk_trace("FLUKE service discovery done");
    for (size_t i = 0; i < fluke->char_count; ++i) {
      struct fluke_gatt_client_char_s *ch = &fluke->chars[i];

      logk_trace("- %d "BLE_UUID_FMT" value at %d, cccd at %d",
              ch->handle, BLE_UUID_ARG(&ch->type),
              ch->handle + 1, ch->cccd);

      bool_t is_bin = !ble_uuid_cmp(&gatt_fluke_reading_char_type, &ch->type);
      if (is_bin) {
        logk_trace("  is binary rx");
        fluke->bin_handle = ch->handle + 1;
        fluke->bin_cccd_handle = ch->cccd;
      }
    }
    fluke->state++;
    goto again;

  case FC_READ_NAME:
    fluke_gatt_client_read_by_type(fluke, 1, -1, &gap_device_name_type);
    return;

  case FC_READ_SERIAL:
    fluke_gatt_client_read_by_type(fluke, 1, -1, &dis_serial_type);
    return;

  case FC_DISCOVER_FLUKE_CCCD:
    fluke_gatt_client_find_information(fluke, fluke->point, fluke->service_group_end);
    return;

  case FC_READ_FW_VERSION:
    fluke_gatt_client_read_by_type(fluke, 1, -1, &dis_firmware_revision_type);
    return;

  case FC_FLUKE_SUBSCRIBE_BINARY:
    fluke_gatt_client_subscribe(fluke, fluke->bin_cccd_handle, 1);
    return;

  case FC_FAIL:
  fail:
    logk_error("Fluke client failed in state %d", fluke->state);
    return;

  case FC_DISCOVERY_DONE: {
    logk_trace("Fluke client done");
    const struct fluke_gatt_client_delegate_vtable_s *vtable
      = const_fluke_gatt_client_delegate_vtable_s_from_base(fluke->layer.delegate_vtable);

    vtable->discovery_done(fluke->layer.delegate, &fluke->layer,
                           fluke->name,
                           fluke->serial,
                           fluke->fw_version);
    return;
  }
  }
}

static
void fluke_gatt_client_reply_handle(struct fluke_gatt_client_s *fluke)
{
  static const struct ble_uuid_s gatt_cccd_type
    = BLE_UUID_BT_BASED(BLE_GATT_ATT_DESC_CCCD);

  struct ble_att_transaction_s *txn = &fluke->att_txn;

  switch (fluke->state) {
  case FC_DISCOVER_FLUKE:
    assert(txn->command == BLE_ATT_FIND_BY_TYPE_VALUE_RQT);
    if (txn->error || txn->find_by_type_value.information_count != 1)
      goto fail;

    logk_trace("Service "BLE_UUID_FMT" is %d-%d",
            BLE_UUID_ARG(&txn->find_by_type_value.type),
            txn->find_by_type_value.information[0].found,
            txn->find_by_type_value.information[0].end_group);
    fluke->service_handle = txn->find_by_type_value.information[0].found;
    fluke->service_group_end = txn->find_by_type_value.information[0].end_group;
    fluke->char_count = 0;
    goto next;

  case FC_DISCOVER_FLUKE_CHARS: {
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
        (void*)(fluke->scratch + offset);

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

      struct fluke_gatt_client_char_s *ch = &fluke->chars[fluke->char_count++];
      ch->type = char_type;
      ch->handle = handle;
      ch->flags = value[0];
      ch->cccd = 0;

      fluke->point = handle + 2;
      if (fluke->point >= fluke->service_group_end)
        goto next;
    }

    goto again;
  }

  case FC_FAIL:
  case FC_IDLE:
  case FC_DISCOVERY_DONE:
  case FC_DISCOVER_FLUKE_COMPLETE:
    break;

  case FC_FLUKE_SUBSCRIBE_BINARY:
    if (txn->error)
      goto fail;
    goto next;

  case FC_DISCOVER_FLUKE_CCCD: {
    assert(txn->command == BLE_ATT_FIND_INFORMATION_RQT);
    if (txn->error == BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND)
      goto next;

    if (txn->error)
      goto fail;

    for (uint16_t i = 0; i < txn->find_information.information_count; ++i) {
      const struct ble_att_information_s *inf = txn->find_information.information + i;

      fluke->point = inf->handle + 1;

      if (ble_uuid_cmp(&gatt_cccd_type, &inf->type))
        continue;

      logk_trace("  - CCCD at %d", inf->handle);

      for (size_t char_no = 0; char_no < fluke->char_count; ++char_no) {
        struct fluke_gatt_client_char_s *ch = &fluke->chars[char_no];

        if (inf->handle < ch->handle)
          continue;

        if (char_no + 1 < fluke->char_count
            && fluke->chars[char_no + 1].handle < inf->handle)
          continue;

        ch->cccd = inf->handle;
        break;
      }
    }

    if (fluke->point >= fluke->service_group_end)
      goto next;

    goto again;
  }
    
  case FC_READ_NAME:
  case FC_READ_FW_VERSION:
  case FC_READ_SERIAL: {
    assert(txn->command == BLE_ATT_READ_BY_TYPE_RQT);
    if (txn->error)
      goto fail;

    if (txn->read_by_type.handle_value_stride == 0
        || txn->read_by_type.handle_value_stride !=
        txn->read_by_type.handle_value_size)
      goto fail;

    uint16_t value_size = txn->read_by_type.handle_value_stride - 2;
    uint16_t handle = endian_16_na_load(fluke->scratch);
    const char *value = (const char*)fluke->scratch + 2;

    logk_trace("At %d value %S", handle, value, value_size);
        
    switch (fluke->state) {
    case FC_READ_NAME:
      strncpy(fluke->name, value, __MIN(value_size, sizeof(fluke->name)));
      fluke->name[__MIN(value_size, sizeof(fluke->name) - 1)] = 0;
      break;

    case FC_READ_FW_VERSION:
      strncpy(fluke->fw_version, value, __MIN(value_size, sizeof(fluke->fw_version)));
      fluke->fw_version[__MIN(value_size, sizeof(fluke->fw_version) - 1)] = 0;
      break;

    case FC_READ_SERIAL:
      strncpy(fluke->serial, value, __MIN(value_size, sizeof(fluke->serial)));
      fluke->serial[__MIN(value_size, sizeof(fluke->serial) - 1)] = 0;
      break;

    default:
      break;
    }

    goto next;
  }
  }

 fail:
  logk_error("Fluke client failure state %d error %d", fluke->state, txn->error);
  logk_error("find_by_type_value count %d", txn->find_by_type_value.information_count);

  net_task_destroy(&txn->task);

  fluke->state = FC_FAIL;

  return;

 next:
  fluke->state++;
  fluke->point = fluke->service_handle;
  fluke->att_txn.error = 0;
 again:
  net_task_destroy(&txn->task);
  fluke_gatt_client_request(fluke);
  return;
 wait:
  net_task_destroy(&txn->task);
}

static void fluke_notif_handle(struct fluke_gatt_client_s *fluke,
                               uint16_t att,
                               const uint8_t *data, size_t size)
{
  const struct fluke_gatt_client_delegate_vtable_s *vtable
      = const_fluke_gatt_client_delegate_vtable_s_from_base(fluke->layer.delegate_vtable);
  if (size < 8)
    return;

  struct fluke_measurement_s measurement;
  fluke_measurement_from_blob(&measurement, data);

  if (!memcmp(&measurement, &fluke->last_measurement, sizeof(measurement)))
    return;
  
  vtable->data_changed(fluke->layer.delegate,
                       &fluke->layer,
                       &measurement);

  memcpy(&fluke->last_measurement, &measurement, sizeof(measurement));
}

static
void fluke_gatt_client_task_handle(struct net_layer_s *layer,
                              struct net_task_s *task)
{
  struct fluke_gatt_client_s *fluke = fluke_gatt_client_s_from_layer(layer);

  switch (task->type) {
  case NET_TASK_QUERY:
    break;

  case NET_TASK_INBOUND: {
    fluke_notif_handle(fluke, task->packet.src_addr.att,
                       task->packet.buffer->data + task->packet.buffer->begin,
                       task->packet.buffer->end - task->packet.buffer->begin);
    break;
  }

  case NET_TASK_RESPONSE:
    if (task->query.opcode == BLE_ATT_REQUEST) {
      assert(task == &fluke->att_txn.task);
      fluke_gatt_client_reply_handle(fluke);
      return;
    }
    break;

  case NET_TASK_TIMEOUT:
    logk_trace("Wait over !");
    fluke->state++;
    fluke_gatt_client_request(fluke);
    break;

  default:
    break;
  }

  net_task_destroy(task);
}

static void fluke_gatt_client_context_changed(struct net_layer_s *layer)
{
  struct fluke_gatt_client_s *fluke = fluke_gatt_client_s_from_layer(layer);

  if (fluke->state == FC_IDLE)
    fluke_gatt_client_request(fluke);
}

static void fluke_gatt_client_dandling(struct net_layer_s *layer)
{
  struct fluke_gatt_client_s *fluke = fluke_gatt_client_s_from_layer(layer);

  (void)fluke;
}

static
void fluke_gatt_client_destroyed(struct net_layer_s *layer)
{
  struct fluke_gatt_client_s *fluke = fluke_gatt_client_s_from_layer(layer);

  mem_free(fluke);
}

static const struct fluke_gatt_client_handler_s fluke_handler = {
  .base.destroyed = fluke_gatt_client_destroyed,
  .base.task_handle = fluke_gatt_client_task_handle,
  .base.context_changed = fluke_gatt_client_context_changed,
  .base.dandling = fluke_gatt_client_dandling,
};

error_t fluke_gatt_client_create(struct net_scheduler_s *scheduler,
                          const void *params_,
                          void *delegate,
                          const struct fluke_gatt_client_delegate_vtable_s *delegate_vtable,
                          struct net_layer_s **layer)
{
  struct fluke_gatt_client_s *fluke = mem_alloc(sizeof(*fluke), mem_scope_sys);

  if (!fluke)
    return -ENOMEM;

  memset(fluke, 0, sizeof(*fluke));

  error_t err = net_layer_init(&fluke->layer, &fluke_handler.base, scheduler,
                               delegate, &delegate_vtable->base);
  if (err) {
    mem_free(fluke);
    return err;
  }

  *layer = &fluke->layer;
  fluke->state = FC_IDLE;

  return 0;
}

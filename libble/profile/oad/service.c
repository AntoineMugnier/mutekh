#define LOGK_MODULE_ID "oads"

#include <mutek/printk.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <hexo/power.h>

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/profile/oad/service.h>
#include <ble/profile/oad/ids.h>

STRUCT_COMPOSE(oad_service_s, reg);
STRUCT_COMPOSE(oad_service_s, timer_rq);

static struct oad_info_s oad_info;
static struct oad_update_command_s command;

enum oad_service_char_id_e
{
  OAD_CHAR_ID_STATUS,
  OAD_CHAR_ID_PAYLOAD,
  OAD_CHAR_ID_INFO,
  OAD_CHAR_ID_COMMAND,
};

static
void oad_status_update_later(struct oad_service_s *oad)
{
    if (dev_timer_request_is_scheduled(&oad->timer_rq))
        return;

    oad->timer_rq.delay = oad->delay;
    oad->timer_rq.deadline = 0;
    DEVICE_OP(&oad->timer, request, &oad->timer_rq);
}

static
void oad_status_update(struct oad_service_s *oad)
{
  logk_trace("OAD status update transfer %d flow %d next 0x%08x",
             oad->status.transfer,
             oad->status.flow,
             oad->status.next_index);

  ble_gattdb_char_changed(&oad->reg, CHAR_ID_STATUS, 1,
                          &oad->status, sizeof(oad->status));
}

static
KROUTINE_EXEC(oad_status_update_kr)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct oad_service_s *oad = oad_service_s_from_timer_rq(rq);

  oad_status_update(oad);
}

static
uint8_t oad_status_read(struct ble_gattdb_client_s *client,
                        struct ble_gattdb_registry_s *reg,
                        uint8_t charid, uint16_t offset,
                        void *data, size_t *size)
{
  struct oad_service_s *oad = oad_service_s_from_reg(reg);

  memcpy(data, &oad->status, sizeof(oad->status));
  *size = sizeof(oad->status);

  return 0;
}

static
uint8_t oad_info_read(struct ble_gattdb_client_s *client,
                        struct ble_gattdb_registry_s *reg,
                        uint8_t charid, uint16_t offset,
                        void *data, size_t *size)
{
  struct oad_service_s *oad = oad_service_s_from_reg(reg);

  memcpy(data, &oad->info, sizeof(oad->info));
  *size = sizeof(oad->info);

  return 0;
}

static
uint8_t oad_payload_write(struct ble_gattdb_client_s *client,
                          struct ble_gattdb_registry_s *reg,
                          uint8_t charid,
                          const void *data, size_t size)
{
  struct oad_service_s *oad = oad_service_s_from_reg(reg);
  const struct oad_payload_s *payload = data;

  logk_trace("OAD Payload write: %P", data, size);

  if (size != sizeof(*payload))
    return BLE_ATT_ERR_INVALID_PDU;
  
  switch (oad->status.transfer) {
  case OAD_TRANSFER_IDLE:
  case OAD_TRANSFER_DONE:
  case OAD_TRANSFER_FAILED:
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

  case OAD_TRANSFER_RUNNING:
    break;
  }

  enum oad_flow_status_e flow_next = oad->status.flow;
  enum oad_transfer_status_e transfer_next = oad->status.transfer;
  uint32_t index = endian_le32_na_load(&payload->index);
  uint32_t offset = index << OAD_CHUNK_SIZE_L2;
  uint32_t expected = oad->status.next_index << OAD_CHUNK_SIZE_L2;;

  if (offset + OAD_CHUNK_SIZE > oad->payload_size) {
    logk_trace("Chunk 0x%06x overflow", offset);
    flow_next = OAD_FLOW_EXPLICIT;
    transfer_next = OAD_TRANSFER_FAILED;
  } else if (offset < expected) {
    logk_trace("Chunk 0x%06x from past, ignored", offset);
  } else if (offset == expected) {
    logk_trace("Chunk 0x%06x in sequence", offset);
    flow_next = OAD_FLOW_OPTIMISTIC;
    transfer_next = OAD_TRANSFER_RUNNING;

    oad->handler->chunk_write(oad, offset, payload->data, OAD_CHUNK_SIZE);

    oad->status.next_index++;

    if (offset >= oad->payload_size) {
      logk_trace(" All chunks done");
      transfer_next = OAD_TRANSFER_DONE;
    }
  } else {
    logk_trace("Chunk 0x%06x OOS, expected 0x%06x",
               offset, oad->status.next_offset);
    flow_next = OAD_FLOW_EXPLICIT;
  }

  if (oad->status.flow == flow_next && oad->status.transfer == transfer_next) {
    oad_status_update_later(oad);
  } else {
    oad->status.flow = flow_next;
    oad->status.transfer = transfer_next;
    oad_status_update(oad);
  }
        
  return 0;
}

static
uint8_t oad_command_write(struct ble_gattdb_client_s *client,
                          struct ble_gattdb_registry_s *reg,
                          uint8_t charid,
                          const void *data, size_t size)
{
  struct oad_service_s *oad = oad_service_s_from_reg(reg);

  logk_trace("OAD Command write: %P", data, size);

  switch (command.command) {
  case OAD_UPDATE_START: {
    if (size < 5) {
      return BLE_ATT_ERR_INVALID_PDU;
    }

    uint32_t payload_size = endian_le32_na_load(data + 1);

    logk_trace("Update start, total chunk count: %d", payload_size);

    if (payload_size <= oad->storage_size) {
      logk_trace("Update starting");

      oad->status.transfer = OAD_TRANSFER_RUNNING;
      oad->status.flow = OAD_FLOW_EXPLICIT;
      oad->status.next_index = 0;
      oad->payload_size = payload_size;

      oad->handler->start(oad, payload_size);
    } else {
      logk_trace("Update does not fit in flash size");
      oad->status.transfer = OAD_TRANSFER_FAILED;
      oad->status.flow = OAD_FLOW_EXPLICIT;
      oad->status.next_index = -1;
      oad->payload_size = 0;
    }

    oad_status_update(oad);
    break;
  }

  case OAD_UPDATE_COMMIT:
    if (oad->status.status == OAD_TRANSFER_DONE) {
      logk_trace("Update commit");

      oad->handler->commit(oad);

      oad->status.transfer = OAD_TRANSFER_IDLE;
      oad->status.flow = OAD_FLOW_EXPLICIT;
      oad->status.next_index = -1;
      oad->payload_size = 0;
    }
    break;

  case OAD_UPDATE_ABORT:
    logk_trace("Update abort");
    oad->status.transfer = OAD_TRANSFER_IDLE;
    oad->status.flow = OAD_FLOW_EXPLICIT;
    oad->status.next_index = -1;
    oad->payload_size = 0;
    oad_status_update(oad);
    break;
  }

  return 0;
}

BLE_GATTDB_SERVICE_DECL(
  oad_service,
  BLE_GATTDB_SERVICE_PRIMARY,
  BLE_UUID_TO_P(OAD_SERVICE_TYPE),
  NULL,

  [OAD_CHAR_ID_STATUS] =
    BLE_GATTDB_CHAR(BLE_UUID_TO_P(OAD_CHAR_STATUS_TYPE),
                    BLE_GATTDB_PERM_OTHER_READ | BLE_GATTDB_NOTIFIABLE,
                    BLE_GATTDB_CHAR_DATA_DYNAMIC(oad_status_read, NULL, NULL),
                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Status"))),

  [OAD_CHAR_ID_PAYLOAD] =
    BLE_GATTDB_CHAR(BLE_UUID_TO_P(OAD_CHAR_PAYLOAD_TYPE),
                    BLE_GATTDB_PERM_OTHER_WRITE,
                    BLE_GATTDB_CHAR_DATA_DYNAMIC(NULL, oad_payload_write, NULL),
                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Payload"))),

  [OAD_CHAR_ID_COMMAND] =
    BLE_GATTDB_CHAR(BLE_UUID_TO_P(OAD_CHAR_COMMAND_TYPE),
                    BLE_GATTDB_PERM_OTHER_WRITE,
                    BLE_GATTDB_CHAR_DATA_DYNAMIC(NULL, oad_command_write, NULL),
                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Command"))),

  [OAD_CHAR_ID_INFO] =
    BLE_GATTDB_CHAR(BLE_UUID_TO_P(OAD_CHAR_INFO_TYPE),
                    BLE_GATTDB_PERM_OTHER_READ,
                    BLE_GATTDB_CHAR_DATA_DYNAMIC(oad_info_read, NULL, NULL),
                    BLE_GATTDB_DESCRIPTORS(BLE_GATTDB_DESCRIPTOR_USER_DESCRIPTION("Info"))),
  );

error_t oad_init(struct oad_service_s *oad,
                 const struct oad_service_handler_s *handler,
                 uint8_t update_per_sec,
                 size_t storage_size,
                 struct device_timer_s *timer,
                 struct ble_gattdb_s *db)
{
    error_t err;

    err = device_copy_accessor(&oad->timer.base, &timer->base);
    if (err) {
        logk_trace("OAD timer error %d", err);
        goto out_timer;
    }

    struct dev_mem_info_s mem_info;

    dev_timer_init_sec(&oad->timer, &oad->delay, NULL, 1, update_per_sec);
    dev_timer_rq_init(&oad->timer_rq, oad_status_update_kr);
                                     
    oad->info.storage_size = storage_size;
    oad->info.chunk_size_l2 = OAD_CHUNK_SIZE_L2;
    oad->info.min_update_per_sec = update_per_sec;

    oad->status.transfer = OAD_TRANSFER_DISABLED;
    oad->status.flow = OAD_FLOW_EXPLICIT;
    oad->status.next_index = -1;

    oad->payload_size = 0;

    logk("OAD target: %dB storage", storage_size);

    err = ble_gattdb_service_register(&oad->reg, db, &oad_service);
    if (err)
      goto out_timer;
    return 0;

out_timer:
    device_put_accessor(&oad->timer.base);
out:
    return err;
}

void oad_service_cleanup(struct oad_service_s *oad)
{
    ble_gattdb_service_unregister(&oad->reg);

    device_put_accessor(&oad->timer.base);
}

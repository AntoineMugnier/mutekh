#ifndef OAD_IDS_H
#define OAD_IDS_H

#include <ble/uuid.h>

#define OAD_SERVICE_TYPE      BLE_UUID(0xc259e06a, 0x0627, 0x4a08, 0xaf46, 0x1ca93ac59850ull)
#define OAD_CHAR_STATUS_TYPE  BLE_UUID(0x57228bfc, 0xa9b0, 0x458f, 0x859e, 0x66e9b8b151f4ull)
#define OAD_CHAR_PAYLOAD_TYPE BLE_UUID(0x31bc4fc9, 0xdca0, 0x4987, 0xa89a, 0x8a10b110b669ull)
#define OAD_CHAR_INFO_TYPE    BLE_UUID(0x6021332f, 0xcd5d, 0x41c2, 0x8970, 0x0a23629312daull)
#define OAD_CHAR_COMMAND_TYPE BLE_UUID(0x98d0ceec, 0x4841, 0x4965, 0xaaef, 0xef348d15f22bull)

#define OAD_CHUNK_SIZE_L2 4
#define OAD_CHUNK_SIZE (1 << OAD_CHUNK_SIZE_L2)

enum oad_transfer_status_e {
    OAD_TRANSFER_IDLE,
    OAD_TRANSFER_RUNNING,
    OAD_TRANSFER_DONE,
    OAD_TRANSFER_FAILED,
};

enum oad_flow_status_e {
    OAD_FLOW_EXPLICIT,
    OAD_FLOW_OPTIMISTIC,
};

struct payload_status_s {
    uint32_t next_index;
    uint8_t transfer;
    uint8_t flow;
} __attribute__((packed));

struct oad_payload_s {
    uint32_t index;
    uint8_t data[OAD_CHUNK_SIZE];
} __attribute__((packed));

struct oad_info_s {
    uint32_t storage_size;
    uint8_t chunk_size_log2;
    uint8_t min_update_per_sec;
} __attribute__((packed));

enum oad_command_e {
    OAD_COMMAND_ABORT,
    OAD_COMMAND_START,
    OAD_COMMAND_COMMIT,
};

struct oad_update_command_s {
    uint8_t command;
    uint32_t arg;
} __attribute__((packed));

#endif

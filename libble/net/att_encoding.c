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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <ble/protocol/att.h>
#include <ble/net/att.h>
#include <ble/net/layer.h>
#include <net/task.h>
#include <net/layer.h>

#include <string.h>
#include "att_encoding.h"

#include <mutek/printk.h>

#define dprintk(...) do{}while(0)
//#define dprintk printk

error_t att_error_serialize(struct buffer_s *p,
                            uint8_t command,
                            uint16_t handle,
                            uint8_t error)
{
    p->data[p->begin] = BLE_ATT_ERROR_RSP;
    p->data[p->begin + 1] = command;
    endian_le16_na_store(&p->data[p->begin + 2], handle);
    p->data[p->begin + 4] = error;
    p->end = p->begin + 5;

    return 0;
}

error_t att_response_serialize(struct buffer_s *p,
                               const struct ble_att_transaction_s *txn,
                               uint16_t mtu)
{
  assert(ble_att_opcode_is_client_to_server(txn->command));
  assert(ble_att_opcode_is_response_expected(txn->command));

  if (txn->error)
    return att_error_serialize(p, txn->command, txn->error_handle, txn->error);

  p->data[p->begin] = ble_att_opcode_operation(txn->command) + 1;

  switch (ble_att_opcode_operation(txn->command)) {
  case BLE_ATT_FIND_INFORMATION_RQT: {
    const struct ble_att_find_information_task_s *rsp
      = const_ble_att_find_information_task_s_from_base(txn);

    if (rsp->information_count == 0)
      return att_error_serialize(p, txn->command, rsp->start, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);

    bool_t uuid16 = ble_uuid_is_uuid16(&rsp->information[0].type);
    uint16_t elem_size = uuid16 ? 4 : 18;

    assert(p->data[p->begin] == BLE_ATT_FIND_INFORMATION_RSP);
    p->data[p->begin + 1] = uuid16 ? BLE_ATT_TYPE_UUID16 : BLE_ATT_TYPE_UUID128;
    p->end = p->begin + 2;

    for (uint8_t i = 0;
         i < rsp->information_count
           && p->end - p->begin + elem_size <= mtu
           && ble_uuid_is_uuid16(&rsp->information[i].type) == uuid16;
         ++i) {
      endian_le16_na_store(&p->data[p->end], rsp->information[i].handle);
      if (uuid16)
        endian_le16_na_store(&p->data[p->end + 2],
                             ble_uuid_uuid16_get(&rsp->information[i].type));
      else
        memrevcpy(&p->data[p->end + 2], &rsp->information[i].type, 16);
      p->end += elem_size;
    }

    return 0;
  }

  case BLE_ATT_FIND_BY_TYPE_VALUE_RQT: {
    const struct ble_att_find_by_type_value_task_s *rsp
      = const_ble_att_find_by_type_value_task_s_from_base(txn);

    if (rsp->information_count == 0)
      return att_error_serialize(p, txn->command, rsp->start, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);

    assert(p->data[p->begin] == BLE_ATT_FIND_BY_TYPE_VALUE_RSP);
    p->end = p->begin + 1;

    for (uint8_t i = 0;
         i < rsp->information_count
           && p->end - p->begin + 4 <= mtu;
         ++i) {
      endian_le16_na_store(&p->data[p->end], rsp->information[i].found);
      endian_le16_na_store(&p->data[p->end + 2], rsp->information[i].end_group);

      p->end += 4;
    }

    return 0;
  }

  case BLE_ATT_READ_BY_TYPE_RQT: {
    const struct ble_att_read_by_type_task_s *rsp
      = const_ble_att_read_by_type_task_s_from_base(txn);

    assert(p->data[p->begin] == BLE_ATT_READ_BY_TYPE_RSP);
    p->data[p->begin + 1] = rsp->handle_value_stride;
    p->end = p->begin + 2;

    for (uint16_t point = 0;
         p->end - p->begin + rsp->handle_value_stride <= mtu
           && point < rsp->handle_value_size;
         point += rsp->handle_value_stride) {
      const struct ble_att_handle_value_s *hv = (void *)((uint8_t *)rsp->handle_value + point);

      endian_le16_na_store(&p->data[p->end], endian_16_na_load(&hv->handle));
      memcpy(&p->data[p->end + 2], hv->value, rsp->handle_value_stride - 2);

      p->end += rsp->handle_value_stride;
      dprintk(" now at %d\n", p->end);
    }

    return 0;
  }

  case BLE_ATT_READ_BLOB_RQT:
  case BLE_ATT_READ_RQT: {
    const struct ble_att_read_task_s *rsp;
    rsp = const_ble_att_read_task_s_from_base(txn);

    assert(p->data[p->begin] == (txn->command == BLE_ATT_READ_BLOB_RQT
                                 ? BLE_ATT_READ_BLOB_RSP
                                 : BLE_ATT_READ_RSP));
    p->end = p->begin + 1;

    uint16_t size = __MIN(mtu - 1, rsp->value_size - rsp->offset);
    memcpy(p->data + p->end, (const uint8_t*)rsp->value + rsp->offset, size);
    p->end += size;

    return 0;
  }

  case BLE_ATT_READ_MULTIPLE_RQT: {
    const struct ble_att_read_multiple_task_s *rsp;
    rsp = const_ble_att_read_multiple_task_s_from_base(txn);

    assert(p->data[p->begin] == BLE_ATT_READ_MULTIPLE_RSP);
    p->end = p->begin + 1;

    uint16_t size = __MIN(mtu - 1, rsp->buffer_size);
    memcpy(p->data + p->end, rsp->buffer, size);
    p->end += size;

    return 0;
  }

  case BLE_ATT_READ_BY_GROUP_TYPE_RQT: {
    const struct ble_att_read_by_group_type_task_s *rsp;
    rsp = const_ble_att_read_by_group_type_task_s_from_base(txn);

    if (rsp->attribute_data_size == 0)
      return att_error_serialize(p, txn->command, rsp->start, BLE_ATT_ERR_ATTRIBUTE_NOT_FOUND);

    assert(p->data[p->begin] == BLE_ATT_READ_BY_GROUP_TYPE_RSP);
    p->data[p->begin + 1] = rsp->attribute_data_stride;
    p->end = p->begin + 2;

    for (uint16_t point = 0;
         p->end - p->begin + rsp->attribute_data_stride <= mtu
           && point < rsp->attribute_data_size;
         point += rsp->attribute_data_stride) {
      const struct ble_att_data_s *ad = (void *)((uint8_t *)rsp->attribute_data + point);

      endian_le16_na_store(&p->data[p->end], endian_16_na_load(&ad->handle));
      endian_le16_na_store(&p->data[p->end + 2], endian_16_na_load(&ad->end_group));
      memcpy(&p->data[p->end + 4], ad->value, rsp->attribute_data_stride - 4);

      p->end += rsp->attribute_data_stride;
    }

    return 0;
  }

  case BLE_ATT_WRITE_RQT:
    assert(p->data[p->begin] == BLE_ATT_WRITE_RSP);
    p->end = p->begin + 1;
    return 0;

  case BLE_ATT_HANDLE_VALUE_INDIC:
    assert(p->data[p->begin] == BLE_ATT_HANDLE_VALUE_CONFIRM);
    p->end = p->begin + 1;
    return 0;
  
  default:
    return -EINVAL;
  }

  return -EINVAL;
}

error_t att_request_serialize(struct buffer_s *p,
                              const struct ble_att_transaction_s *txn,
                              uint16_t mtu)
{
  assert(ble_att_opcode_is_client_to_server(txn->command));

  p->data[p->begin] = txn->command;

  switch (ble_att_opcode_operation(txn->command)) {
  case BLE_ATT_FIND_INFORMATION_RQT: {
    const struct ble_att_find_information_task_s *req;
    req = const_ble_att_find_information_task_s_from_base(txn);

    assert(p->data[p->begin] == BLE_ATT_FIND_INFORMATION_RQT);
    endian_le16_na_store(p->data + p->begin + 1, req->start);
    endian_le16_na_store(p->data + p->begin + 3, req->end);
    p->end = p->begin + 5;

    return 0;
  }

  case BLE_ATT_FIND_BY_TYPE_VALUE_RQT: {
    const struct ble_att_find_by_type_value_task_s *req;
    req = const_ble_att_find_by_type_value_task_s_from_base(txn);

    if (!ble_uuid_is_uuid16(&req->type))
      return -EINVAL;

    if (req->value_size > mtu - 7)
      return -ENOSPC;

    assert(p->data[p->begin] == BLE_ATT_FIND_BY_TYPE_VALUE_RQT);
    endian_le16_na_store(p->data + p->begin + 1, req->start);
    endian_le16_na_store(p->data + p->begin + 3, req->end);
    endian_le16_na_store(p->data + p->begin + 5, ble_uuid_uuid16_get(&req->type));
    memcpy(p->data + p->begin + 7, req->value, req->value_size);
    p->end = p->begin + 7 + req->value_size;

    return 0;
  }

  case BLE_ATT_READ_BY_TYPE_RQT: {
    const struct ble_att_read_by_type_task_s *req;
    req = const_ble_att_read_by_type_task_s_from_base(txn);

    assert(p->data[p->begin] == BLE_ATT_READ_BY_TYPE_RQT);
    endian_le16_na_store(p->data + p->begin + 1, req->start);
    endian_le16_na_store(p->data + p->begin + 3, req->end);
    if (ble_uuid_is_uuid16(&req->type)) {
      endian_le16_na_store(p->data + p->begin + 5, ble_uuid_uuid16_get(&req->type));
      p->end = p->begin + 7;
    } else {
      memrevcpy(p->data + p->begin + 5, &req->type, 16);
      p->end = p->begin + 21;
    }

    return 0;
  }

  case BLE_ATT_READ_RQT: {
    const struct ble_att_read_task_s *req
      = const_ble_att_read_task_s_from_base(txn);

    assert(p->data[p->begin] == BLE_ATT_READ_RQT);

    endian_le16_na_store(p->data + p->begin + 1, req->handle);
    p->end = p->begin + 3;

    return 0;
  }

  case BLE_ATT_READ_BLOB_RQT: {
    const struct ble_att_read_task_s *req
      = const_ble_att_read_task_s_from_base(txn);

    assert(p->data[p->begin] == BLE_ATT_READ_BLOB_RQT);

    endian_le16_na_store(p->data + p->begin + 1, req->handle);
    endian_le16_na_store(p->data + p->begin + 3, req->offset);
    p->end = p->begin + 5;

    return 0;
  }

  case BLE_ATT_READ_MULTIPLE_RQT: {
    const struct ble_att_read_multiple_task_s *req
      = const_ble_att_read_multiple_task_s_from_base(txn);

    size_t count = __MIN(req->handle_count, (mtu - 1) / 2);

    assert(p->data[p->begin] == BLE_ATT_READ_MULTIPLE_RQT);

    if (count < 2)
      return -EINVAL;

    for (size_t i = 0; i < count; ++i)
      endian_le16_na_store(p->data + p->begin + 1 + 2 * i, req->handle[i]);
    p->end = p->begin + 1 + 2 * count;

    return 0;
  }

  case BLE_ATT_READ_BY_GROUP_TYPE_RQT: {
    const struct ble_att_read_by_group_type_task_s *req
      = const_ble_att_read_by_group_type_task_s_from_base(txn);

    assert(p->data[p->begin] == BLE_ATT_READ_BY_GROUP_TYPE_RQT);

    endian_le16_na_store(p->data + p->begin + 1, req->start);
    endian_le16_na_store(p->data + p->begin + 3, req->end);
    if (ble_uuid_is_uuid16(&req->type)) {
      endian_le16_na_store(p->data + p->begin + 5, ble_uuid_uuid16_get(&req->type));
      p->end = p->begin + 7;
    } else {
      memrevcpy(p->data + p->begin + 5, &req->type, 16);
      p->end = p->begin + 21;
    }

    return 0;
  }

  case BLE_ATT_WRITE_RQT:
  case BLE_ATT_WRITE_CMD:
  case BLE_ATT_HANDLE_VALUE_NOTIF:
  case BLE_ATT_HANDLE_VALUE_INDIC: {
    const struct ble_att_write_task_s *req;
    req = const_ble_att_write_task_s_from_base(txn);

    if (ble_att_opcode_is_signed(txn->command))
      mtu -= 12;

    if (req->value_size > mtu - 3)
      return -ENOSPC;

    endian_le16_na_store(p->data + p->begin + 1, req->handle);
    memcpy(p->data + p->begin + 3, req->value, req->value_size);
    p->end = p->begin + 3 + req->value_size;
    return 0;
  }
  }

  return -ENOTSUP;
}

enum ble_att_error_e att_request_parse(struct ble_att_s *att,
                                       const struct buffer_s *p,
                                       struct ble_att_transaction_s **rtxn)
{
  const uint8_t *data = p->data + p->begin;
  const size_t size = p->end - p->begin;
  struct ble_att_transaction_s *txn;

  switch ((enum ble_att_opcode_e)data[0]) {
  case BLE_ATT_FIND_INFORMATION_RQT: {
    if (size != 5)
      return BLE_ATT_ERR_INVALID_PDU;

    size_t max_count = (att->mtu - 2) / 4;
    struct ble_att_find_information_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + max_count * sizeof(*req->information) + 8);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_find_information_task_s_from_base(txn);

    req->start = endian_le16_na_load(data + 1);
    req->end = endian_le16_na_load(data + 3);
    req->information = (void*)(req + 1);
    req->information_max_count = max_count;
    req->information_count = 0;

    break;
  }

  case BLE_ATT_FIND_BY_TYPE_VALUE_RQT: {
    if (size < 7)
      return BLE_ATT_ERR_INVALID_PDU;

    size_t max_count = (att->mtu - 1) / 4;
    size_t value_size = size - 7;
    struct ble_att_find_by_type_value_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + value_size
                               + max_count * sizeof(*req->information) + 4 + 8);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_find_by_type_value_task_s_from_base(txn);

    req->start = endian_le16_na_load(data + 1);
    req->end = endian_le16_na_load(data + 3);
    ble_uuid_bluetooth_based(&req->type, endian_le16_na_load(data + 5));
    req->value = (void*)(req + 1);
    memcpy(req->value, data + 7, value_size);
    req->value_size = value_size;
    req->information_count = 0;
    req->information = ALIGN_ADDRESS_UP(req->value + value_size, 8);
    req->information_max_count = max_count;

    break;
  }

  case BLE_ATT_READ_BY_TYPE_RQT: {
    if (size != 7 && size != 21)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_by_type_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + att->mtu - 2 + 4);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_read_by_type_task_s_from_base(txn);

    req->start = endian_le16_na_load(data + 1);
    req->end = endian_le16_na_load(data + 3);
    if (size == 7)
      ble_uuid_bluetooth_based(&req->type, endian_le16_na_load(data + 5));
    else
      memrevcpy(&req->type, (void *)(data + 5), 16);
    req->handle_value = (void*)(req + 1);
    req->handle_value_stride = 0;
    req->handle_value_size = 0;
    req->handle_value_size_max = att->mtu - 2;

    break;
  }

  case BLE_ATT_READ_RQT: {
    if (size != 3)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + att->mtu - 1 + 8);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_read_task_s_from_base(txn);

    req->handle = endian_le16_na_load(data + 1);
    req->value = (void*)(req + 1);
    req->value_size = 0;
    req->offset = 0;
    req->value_size_max = att->mtu - 1;

    break;
  }

  case BLE_ATT_READ_BLOB_RQT: {
    if (size != 5)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + att->mtu - 1 + 8);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_read_task_s_from_base(txn);

    req->handle = endian_le16_na_load(data + 1);
    req->offset = endian_le16_na_load(data + 3);
    req->value_size = endian_le16_na_load(data + 3);
    req->value = (uint8_t*)(req + 1) - req->value_size;
    req->value_size_max = req->value_size + att->mtu - 1;

    break;
  }

  case BLE_ATT_READ_MULTIPLE_RQT: {
    if (size < 5 || ((size - 1) % 2))
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_multiple_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + att->mtu - 1 + size - 1 + 4 + 8);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_read_multiple_task_s_from_base(txn);

    req->handle = ALIGN_ADDRESS_UP((void*)(req + 1), 2);
    req->handle_count = 0;
    while (req->handle_count * 2 + 1 < size) {
      req->handle[req->handle_count]
        = endian_le16_na_load(data + req->handle_count * 2 + 1);
      req->handle_count++;
    }

    req->buffer = (void*)(req->handle + req->handle_count);
    req->buffer_size = 0;
    req->buffer_size_max = att->mtu - 1;

    break;
  }

  case BLE_ATT_READ_BY_GROUP_TYPE_RQT: {
    if (size != 7 && size != 21)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_by_group_type_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + att->mtu - 2 + 8);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_read_by_group_type_task_s_from_base(txn);

    req->start = endian_le16_na_load(data + 1);
    req->end = endian_le16_na_load(data + 3);
    if (size == 7)
      ble_uuid_bluetooth_based(&req->type, endian_le16_na_load(data + 5));
    else
      memrevcpy(&req->type, (void *)(data + 5), 16);
    req->attribute_data = (void*)(req + 1);
    req->attribute_data_size = 0;
    req->attribute_data_stride = 0;
    req->attribute_data_size_max = att->mtu - 2;

    break;
  }

  case BLE_ATT_HANDLE_VALUE_NOTIF:
  case BLE_ATT_HANDLE_VALUE_INDIC:
  case BLE_ATT_WRITE_RQT:
  case BLE_ATT_WRITE_CMD: {
    if (size < 3)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_write_task_s *req;
    txn = att_request_allocate(att, sizeof(*req) + size - 3 + 8);
    if (!txn)
      return BLE_ATT_ERR_INSUF_RESOURCES;
    req = ble_att_write_task_s_from_base(txn);

    req->handle = endian_le16_na_load(data + 1);
    req->value = (void*)(req + 1);
    req->value_size = size - 3;
    memcpy(req->value, data + 3, size - 3);

    break;
  }

  default:
    return BLE_ATT_ERR_INVALID_PDU;
  }

  txn->command = data[0];
  txn->error = 0;
  txn->error_handle = 0;

  *rtxn = txn;

  return 0;
}

enum ble_att_error_e att_response_parse(struct ble_att_s *att,
                                        const struct buffer_s *p,
                                        struct ble_att_transaction_s **rtxn)
{
  const uint8_t *data = p->data + p->begin;
  const size_t size = p->end - p->begin;
  struct ble_att_transaction_s *txn;

  txn = att->transaction_pending;
  if (txn && ble_att_opcode_operation(txn->command) + 1 != data[0])
    txn = NULL;

  if (data[0] != BLE_ATT_ERROR_RSP && !txn)
    return BLE_ATT_ERR_INVALID_PDU;

  switch ((enum ble_att_opcode_e)data[0]) {
  case BLE_ATT_ERROR_RSP: {
    uint16_t handle = endian_le16_na_load(data + 2);
    uint8_t error = data[4];

    txn = att->transaction_pending;
    if (!txn || txn->command != data[1])
      return BLE_ATT_ERR_INVALID_PDU;

    txn->error_handle = handle;
    txn->error = error;
    break;
  }

  case BLE_ATT_FIND_INFORMATION_RSP: {
    if (size < 4 || data[1] < 1 || data[1] > 2)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_find_information_task_s *query
      = ble_att_find_information_task_s_from_base(txn);

    uintptr_t info = (uintptr_t)data + 2;
    uintptr_t end = (uintptr_t)data + size;
    uintptr_t info_size = data[1] == 1 ? 4 : 18;

    for (;
         info < end && query->information_count && query->information_max_count;
         info += info_size) {
      struct ble_att_information_s *ai = &query->information[query->information_count];

      ai->handle = endian_le16_na_load((const void*)info);
      if (data[1] == 1)
        ble_uuid_bluetooth_based(&ai->type, endian_le16_na_load((const void*)(info + 2)));
      else
        memrevcpy(&ai->type, (void *)(info + 2), 16);
      query->information_count++;
    }

    if (info < end)
      query->start = endian_le16_na_load((const void*)info);
    else
      query->start = query->information[query->information_count - 1].handle + 1;

    break;
  }

  case BLE_ATT_FIND_BY_TYPE_VALUE_RSP: {
    if ((size - 1) % 4)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_find_by_type_value_task_s *query
      = ble_att_find_by_type_value_task_s_from_base(txn);

    uintptr_t info = (uintptr_t)data + 1;
    uintptr_t end = (uintptr_t)data + size;

    for (;
         info < end && query->information_count && query->information_max_count;
         info += 4) {
      struct ble_att_handle_information_s *ai = &query->information[query->information_count];

      ai->found = endian_le16_na_load((const void*)info);
      ai->end_group = endian_le16_na_load((const void*)(info + 2));
      query->information_count++;
    }

    if (info < end)
      query->start = endian_le16_na_load((const void*)info);
    else
      query->start = query->information[query->information_count - 1].end_group + 1;

    break;
  }


  case BLE_ATT_READ_BY_TYPE_RSP: {
    if (size < 4)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_by_type_task_s *query
      = ble_att_read_by_type_task_s_from_base(txn);

    if (query->handle_value_size && query->handle_value_stride != data[1])
      break;

    query->handle_value_stride = data[1];

    for (const uint8_t *point = data + 2;
         point < data + size && query->handle_value_size + data[1] <= query->handle_value_size_max;
         point += data[1]) {
      struct ble_att_handle_value_s *hv
        = (void*)(query->handle_value + query->handle_value_size);
      uint16_t h = endian_le16_na_load(point);

      query->start = h + 1;
      endian_16_na_store(&hv->handle, h);
      memcpy(hv->value, point + 2, data[1] - 2);
      query->handle_value_size += data[1];
    }

    break;
  }

  case BLE_ATT_READ_RSP:
  case BLE_ATT_READ_BLOB_RSP: {
    struct ble_att_read_task_s *query
      = ble_att_read_task_s_from_base(txn);

    size_t to_copy = __MIN(size - 1, query->value_size_max - query->value_size);
    memcpy(query->value + query->value_size, data + 1, to_copy);
    query->value_size += to_copy;

    break;
  }

  case BLE_ATT_READ_MULTIPLE_RSP: {
    if (size < 4)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_multiple_task_s *query
      = ble_att_read_multiple_task_s_from_base(txn);

    size_t to_copy = __MIN(size - 1, query->buffer_size_max - query->buffer_size);
    memcpy(query->buffer + query->buffer_size, data + 1, to_copy);
    query->buffer_size += to_copy;

    break;
  }

  case BLE_ATT_READ_BY_GROUP_TYPE_RSP: {
    if (size < 6)
      return BLE_ATT_ERR_INVALID_PDU;

    struct ble_att_read_by_group_type_task_s *query
      = ble_att_read_by_group_type_task_s_from_base(txn);

    if (query->attribute_data_size && query->attribute_data_stride != data[1])
      break;

    query->attribute_data_stride = data[1];

    for (const uint8_t *point = data + 2;
         point < data + size && query->attribute_data_size + data[1] <= query->attribute_data_size_max;
         point += data[1]) {
      struct ble_att_data_s *ad
        = (void*)(query->attribute_data + query->attribute_data_size);
      uint16_t h = endian_le16_na_load(point);
      uint16_t e = endian_le16_na_load(point + 2);

      query->start = e + 1;
      endian_16_na_store(&ad->handle, h);
      endian_16_na_store(&ad->end_group, e);
      memcpy(ad->value, point + 4, data[1] - 4);
      query->attribute_data_size += data[1];
    }

    break;
  }

  case BLE_ATT_HANDLE_VALUE_CONFIRM:
  case BLE_ATT_WRITE_RSP: {
    if (size != 1)
      return BLE_ATT_ERR_INVALID_PDU;

    break;
  }

  default:
    return BLE_ATT_ERR_INVALID_PDU;
  }

  *rtxn = txn;

  return 0;
}

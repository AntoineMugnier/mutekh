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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

/**
   @file
   @module {Core::Devices support library}
   @short NFC device driver API
   @index {NFC transceivers} {Device classes}
   @csee DRIVER_CLASS_NFC

 */

#ifndef __DEVICE_NFC_H__
#define __DEVICE_NFC_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/request.h>

#include <hexo/enum.h>

struct driver_s;
struct dev_nfc_rq_s;
struct driver_nfc_s;
struct device_nfc_s;

ENUM_DESCRIPTOR(dev_nfc_protocol_e, strip:DEV_NFC_, upper);

/**
   Protocol to implement
 */
enum dev_nfc_protocol_e
{
  DEV_NFC_14443A,
  DEV_NFC_14443B,
  DEV_NFC_15693_256,
  DEV_NFC_15693_4,
  DEV_NFC_18000_MODE1,
  DEV_NFC_18000_MODE1_EXT,
  DEV_NFC_18000_MODE2,
  DEV_NFC_FELICA,
};

ENUM_DESCRIPTOR(dev_nfc_side_e, strip:DEV_NFC_, upper);

/**
   Side we are on.
 */
enum dev_nfc_side_e
{
  DEV_NFC_PASSIVE_PCD,
  DEV_NFC_PASSIVE_PICC,
  DEV_NFC_ACTIVE,
};

#define DEV_NFC_ATQA_ANTICOLL(x) (1 << (x))
#define DEV_NFC_ATQA_UID_SIZE(x) ((((x) - 4) / 3) << 6)
#define DEV_NFC_ATQA_UID_SIZE_GET(x) ((((x) >> 6) & 0x3) * 3 + 4)
#define DEV_NFC_ATQA_PLATCONFIG(x) ((x) << 8)

#define DEV_NFC_SEL_NFCIP1 bit(6)
#define DEV_NFC_SEL_14443_4 bit(5)

#define DEV_NFC_ISO14443_CMD_REQA 0x26
#define DEV_NFC_ISO14443_CMD_WUPA 0x52
#define DEV_NFC_ISO14443_CMD_TSLOT 0x35
#define DEV_NFC_ISO14443_CMD_HLTA 0x50
#define DEV_NFC_ISO14443_CMD_SEL1 0x93
#define DEV_NFC_ISO14443_CMD_SEL2 0x95
#define DEV_NFC_ISO14443_CMD_SEL3 0x97

struct dev_nfc_peer_s
{
  uint8_t uid[10];
  uint16_t atqa;
  uint8_t uid_size;
  uint8_t sak;

  /**
     Protocol variant to implement.  This mainly changes the modulation
     and baudrate.
   */
  enum dev_nfc_protocol_e protocol : 4;
  enum dev_nfc_side_e side : 2;
  /**
     Baudrate = 13.56MHz / divier, a power of two between 16 and 2048.
   */
  uint8_t div_log2;
};

struct dev_nfc_transceiver_info_s
{
  /** Only filled if relevant (i.e. if device may act as target) */
  uint8_t uid[10];
  uint16_t atqa;
  uint8_t uid_size;
  uint8_t sak;

  struct {
    uint16_t div_log2;
    uint8_t side;
    uint8_t protocol;
  } support;
};

ENUM_DESCRIPTOR(dev_nfc_req_type_e, strip:DEV_NFC_, upper);

enum dev_nfc_req_type_e
{
  /** TX */
  DEV_NFC_TRANSMIT,
  /** RX */
  DEV_NFC_RECEIVE,
  /** Power field, Wakeup all PICCs, do anticoll and put first
      available PICC it in ACTIVE state, HALT others */
  DEV_NFC_FIND_ANY,
  /* Seek for a given PICC in field, put others in HALT state */
  DEV_NFC_SELECT,
};

ENUM_DESCRIPTOR(dev_nfc_parity_mode_e, strip:DEV_NFC_PARITY_, upper);

enum dev_nfc_parity_mode_e
{
  /** Parity is automatically inserted and checked. */
  DEV_NFC_PARITY_AUTO,
  /** Parity is transceived from/to @tt parity buffer once every 8
      data bits. */
  DEV_NFC_PARITY_RAW,
  /** No parity at all, only transceive bits from @tt data buffer. */
  DEV_NFC_PARITY_NONE,
};

struct dev_nfc_rq_s
{
  struct dev_request_s base;

  error_t error;

  enum dev_nfc_req_type_e type;

  /**
     Peer device info. May be populated automatically by request.
   */
  struct dev_nfc_peer_s *peer;

  union {
    struct {
      /** Data parity mode. @see dev_nfc_parity_mode_e. */
      bool_t parity_mode;

      /** Whether transceiver should append/check ISO 16-bit CRC.  @tt
          parity_mode must be @tt DEV_NFC_PARITY_AUTO if set. */
      bool_t auto_crc;

      /** Frame byte count. Every started byte counts. i.e. @tt size
          is the byte count of @tt data buffer. */
      uint16_t size;
      /** 0 means last byte is full with parity bit (if not raw), 8 is
          forbidden.  Only allowed to be non-zero if @tt parity_mode
          is not DEV_NFC_PARITY_AUTO or @tt size is 1. */
      uint16_t last_byte_bits;

      /** Data buffer. Data is transceived LSB first. */
      uint8_t *data;

      /** Parity buffer. Only used if @tt parity_mode is @tt DEV_NFC_PARITY_RAW. */
      uint8_t *parity;
    } data;
  };
};

STRUCT_INHERIT(dev_nfc_rq_s, dev_request_s, base);

/** @see dev_nfc_request_t */
#define DEV_NFC_REQUEST(n)                      \
  void (n)(const struct device_nfc_s *accessor, \
           struct dev_nfc_rq_s *rq)

/**
   NFC device class request() function type. Enqueue a read or write
   request.
*/
typedef DEV_NFC_REQUEST(dev_nfc_request_t);

/** @see dev_nfc_cancel_t */
#define DEV_NFC_CANCEL(n)                               \
  error_t (n)(const struct device_nfc_s *accessor,      \
              struct dev_nfc_rq_s *rq)
/**
   NFC device class cancel() function type. Cancel a request.
*/
typedef DEV_NFC_CANCEL(dev_nfc_cancel_t);

/** @see dev_nfc_info_get_t */
#define DEV_NFC_INFO_GET(n)                               \
  void (n)(const struct device_nfc_s *accessor,           \
           struct dev_nfc_transceiver_info_s *info)
/**
   NFC device class info_get() function type. Get local device information.
*/
typedef DEV_NFC_INFO_GET(dev_nfc_info_get_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_NFC, nfc,
                   dev_nfc_request_t *f_request;
                   dev_nfc_cancel_t *f_cancel;
                   dev_nfc_info_get_t *f_info_get;
                   );

/** @see driver_nfc_s */
#define DRIVER_NFC_METHODS(prefix)                              \
  ((const struct driver_class_s*)&(const struct driver_nfc_s){  \
    .class_ = DRIVER_CLASS_NFC,                                 \
    .f_request = prefix ## _request,                            \
    .f_cancel = prefix ## _cancel,                              \
    .f_info_get = prefix ## _info_get,                          \
  })

/** @This is busy wait wrapper for the @ref dev_nfc_request_t function */
BUSY_WAITING_FUNCTION
config_depend_inline(CONFIG_DEVICE_NFC,
error_t dev_nfc_spin_request(
    const struct device_nfc_s *accessor,
    struct dev_nfc_rq_s *req),
{
    struct dev_request_status_s status;

    dev_request_spin_init(&req->base, &status);
    DEVICE_OP(accessor, request, req);
    dev_request_spin_wait(&status);

    return req->error;
});

/** @This is scheduler wait wrapper for the @ref dev_nfc_request_t function */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_request(
    const struct device_nfc_s *accessor,
    struct dev_nfc_rq_s *req),
{
      struct dev_request_status_s status;

      dev_request_sched_init(&req->base, &status);

      DEVICE_OP(accessor, request, req);

      dev_request_sched_wait(&status);

      return req->error;
});

#endif

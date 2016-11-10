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

#define DEV_NFC_SAK_NFCIP1 bit(6)
#define DEV_NFC_SAK_14443_4 bit(5)

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
  /** Wakeup all PICCs, do anticoll and put first available PICC it in
      ACTIVE state, HALT others */
  DEV_NFC_SELECT_ANY,
  /** Seek for a given PICC in field, put others in IDLE/HALT state */
  DEV_NFC_SELECT,
  /** Remove power from field */
  DEV_NFC_POWEROFF,
};

ENUM_DESCRIPTOR(dev_nfc_framing_e, strip:DEV_NFC_FRAMING_, upper);

enum dev_nfc_framing_e
{
  /** Transceiver should append/check ISO 16-bit CRC (and parity). */
  DEV_NFC_FRAMING_STD,
  /** Parity is automatically inserted and checked, but CRC is not
      automatically appended/checked. */
  DEV_NFC_FRAMING_PARITY,
  /** Parity is not inserted nor checked, @tt data is the raw
      transceived data stream. */
  DEV_NFC_FRAMING_RAW,
};

struct dev_nfc_rq_data_s
{
  /** Data buffer. Data is transceived LSB first. */
  uint8_t *data;

  /** Frame byte count. Every started byte counts. i.e. @tt size
      is the byte count of @tt data buffer. */
  uint16_t size;

  /** Data framing mode. @see dev_nfc_framing_e. */
  enum dev_nfc_framing_e framing : 8;

  /** 0 means last byte is full with parity bit (if not raw), 8 is
      forbidden.  Only allowed to be non-zero if @tt framing
      is not DEV_NFC_FRAMING_STD. */
  uint8_t last_byte_bits;
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

  struct dev_nfc_rq_data_s data;
};

STRUCT_INHERIT(dev_nfc_rq_s, dev_request_s, base);

/** @see dev_nfc_request_t */
#define DEV_NFC_REQUEST(n)                      \
  void (n)(const struct device_nfc_s *accessor, \
           struct dev_nfc_rq_s *rq1, \
           struct dev_nfc_rq_s *rq2)

/**
   NFC device class request() function type. Enqueue up to 2
   requests.

   Enqueing two requests is only valid for atomic
   transmit-then-receive (a.k.a. transceive) operations.  When two
   requests are pushed, they must be of type @tt DEV_NFC_TRANSMIT and
   @tt DEV_NFC_RECEIVE, respectively.  Any other request types must be
   enqueued one-by-one.

   If only one request is pushed, second request should be set to @tt
   NULL.
*/
typedef DEV_NFC_REQUEST(dev_nfc_request_t);

/** @see dev_nfc_cancel_t */
#define DEV_NFC_CANCEL(n)                               \
  error_t (n)(const struct device_nfc_s *accessor,      \
              struct dev_nfc_rq_s *rq)
/**
   NFC device class cancel() function type. Cancels a request.
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
    struct dev_nfc_rq_s *rq),
{
    struct dev_request_status_s status;

    dev_request_spin_init(&rq->base, &status);
    DEVICE_OP(accessor, request, rq, NULL);
    dev_request_spin_wait(&status);

    return rq->error;
});

/** @This is scheduler wait wrapper for the @ref dev_nfc_request_t function */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_request(
    const struct device_nfc_s *accessor,
    struct dev_nfc_rq_s *rq),
{
      struct dev_request_status_s status;

      dev_request_sched_init(&rq->base, &status);
      DEVICE_OP(accessor, request, rq, NULL);
      dev_request_sched_wait(&status);

      return rq->error;
});

/** @This is scheduler wait wrapper for transceiving data with
    standard framing. */
config_depend_and2(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_nfc_wait_transceive_std(
    const struct device_nfc_s *accessor,
    struct dev_nfc_peer_s *peer,
    const uint8_t *tx_data, size_t tx_size,
    uint8_t *rx_data, size_t *rx_size);

#endif

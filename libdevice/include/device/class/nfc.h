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

ENUM_DESCRIPTOR(dev_nfc_modem_e, strip:DEV_NFC_MODEM_, upper);

/**
   Modem to implement
 */
enum dev_nfc_modem_e
{
  DEV_NFC_MODEM_14443A,
  DEV_NFC_MODEM_14443B,
  DEV_NFC_MODEM_15693_256,
  DEV_NFC_MODEM_15693_4,
  DEV_NFC_MODEM_18000_MODE1,
  DEV_NFC_MODEM_18000_MODE1_EXT,
  DEV_NFC_MODEM_18000_MODE2,
  DEV_NFC_MODEM_FELICA,
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

ENUM_DESCRIPTOR(dev_nfc_req_type_e, strip:DEV_NFC_, upper);

enum dev_nfc_req_type_e
{
  /** TX, takes a @tt data argument */
  DEV_NFC_TRANSMIT,
  /** RX, takes a @tt data argument */
  DEV_NFC_RECEIVE,
  /** Find first PICC in field after WUP.  PICC will be activated.
      Fills a @tt picc argument.  */
  DEV_NFC_WAKEUP_FIND,
  /** Find first PICC in field after REQ.  PICC will be activated.
      Fills a @tt picc argument. */
  DEV_NFC_REQUEST_FIND,
  /** Wait for emulated PICC to be selected.  */
  DEV_NFC_WAIT_SELECTED,
  /** Req/Sel a PICC by UID, useful for fast reconnection to last PICC
      after a power cycle. Takes a @tt picc argument.  */
  DEV_NFC_ACTIVATE,
  /** Halt currently selected PICC in field. No argument required. */
  DEV_NFC_HALT,
  /** Configure transceiver, takes a @tt config argument */
  DEV_NFC_CONFIG,
  /** Remove power from field / disable transceiver. No argument
      required. */
  DEV_NFC_SHUTDOWN,
};

ENUM_DESCRIPTOR(dev_nfc_framing_e, strip:DEV_NFC_FRAMING_, upper);

enum dev_nfc_framing_e
{
  /** Transceiver should append/check ISO 16-bit CRC (and parity).
      Initial value and polynom match depend on modem configuration. */
  DEV_NFC_FRAMING_CRC,
  /** Parity is automatically inserted and checked, but CRC is not
      automatically appended/checked. CRC (if any) will be part of
      data buffer. */
  DEV_NFC_FRAMING_PARITY,
  /** Parity is not inserted nor checked, @tt data is the raw
      transceived data stream. */
  DEV_NFC_FRAMING_RAW,
};

ENUM_DESCRIPTOR(dev_nfc_protocol_e, strip:DEV_NFC_PROTOCOL_, upper);

enum dev_nfc_protocol_e
{
  /** ISO T=CL encapsulation. Requires CRC framing. */
  DEV_NFC_PROTOCOL_ISO_DEP,
  /** Raw data is passed to framer. */
  DEV_NFC_PROTOCOL_RAW,
};

ENUM_DESCRIPTOR(dev_nfc_div_e, strip:DEV_NFC_DIV_, upper);

enum dev_nfc_div_e
{
  DEV_NFC_DIV_106K = 7,
  DEV_NFC_DIV_212K = 6,
  DEV_NFC_DIV_424K = 5,
  DEV_NFC_DIV_848K = 4,
};

struct dev_nfc_transceiver_info_s
{
  /** Only filled if relevant (i.e. if device may act as target) */
  uint8_t uid[10];
  uint16_t atqa;
  uint8_t uid_size;
  uint8_t sak;

  struct {
    // Bitfields
    uint8_t div;
    uint8_t side;
    uint16_t modem;
  } support;
};

struct dev_nfc_config_s
{
  /**
     Modem variant to implement.  This mainly changes the modulation
     and baudrate.
   */
  enum dev_nfc_modem_e modem : 8;

  /**
     Side to be on.
   */
  enum dev_nfc_side_e side : 8;
};

struct dev_nfc_data_s
{
  /** Data buffer. Data is transceived LSB first. */
  uint8_t *data;

  /** Frame byte count. Every started byte counts. i.e. @tt size
      is the byte count of @tt data buffer. */
  uint16_t size;

  /** Data framing mode. @see dev_nfc_framing_e. */
  enum dev_nfc_framing_e framing : 8;
  enum dev_nfc_div_e rate : 8;

  /** 0 means last byte is full with parity bit (if not raw), 8 is
      forbidden.  Only allowed to be non-zero if @tt framing
      is not DEV_NFC_FRAMING_STD. */
  uint8_t last_byte_bits;
};

struct dev_nfc_picc_s
{
  /** UID */
  uint8_t uid[10];

  /** UID length */
  uint8_t uid_size;

  uint8_t sak;
  
  enum dev_nfc_modem_e modem : 8;
};

struct dev_nfc_rq_s
{
  struct dev_request_s base;
  enum dev_nfc_req_type_e type : 8;

  error_t error;

  union {
    struct dev_nfc_data_s data;
    struct dev_nfc_config_s config;
    struct dev_nfc_picc_s picc;
  };
};

STRUCT_INHERIT(dev_nfc_rq_s, dev_request_s, base);


ALWAYS_INLINE
void dev_nfc_rq_bitcount_set(struct dev_nfc_rq_s *rq, size_t bits)
{
  rq->data.size = (bits + 7) / 8;
  rq->data.last_byte_bits = bits % 8;
}

ALWAYS_INLINE
size_t dev_nfc_rq_bitcount_get(const struct dev_nfc_rq_s *rq)
{
  return rq->data.size * 8
    + (rq->data.last_byte_bits ? rq->data.last_byte_bits - 8 : 0);
}


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
    const uint8_t *tx_data, size_t tx_size,
    uint8_t *rx_data, size_t *rx_size,
    enum dev_nfc_div_e div);

/** @This performs a poweroff of PCD antenna. */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_poweroff(const struct device_nfc_s *accessor)
{
  struct dev_nfc_rq_s rq;

  rq.type = DEV_NFC_POWEROFF;

  return dev_nfc_wait_request(accessor, &rq);
});

/** @This performs a poweron of PCD antenna. */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_poweron(const struct device_nfc_s *accessor)
{
  struct dev_nfc_rq_s rq;

  rq.type = DEV_NFC_POWERON;

  return dev_nfc_wait_request(accessor, &rq);
});

/** @This performs a HLT command on current PICC. */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_halt(const struct device_nfc_s *accessor)
{
  struct dev_nfc_rq_s rq;

  rq.type = DEV_NFC_HALT;

  return dev_nfc_wait_request(accessor, &rq);
});

/** @This performs a WUP, then selects first PICC. */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_wakeup_find(const struct device_nfc_s *accessor,
                                 struct dev_nfc_picc_s *picc)
{
  struct dev_nfc_rq_s rq;
  error_t err;

  rq.type = DEV_NFC_WAKEUP_FIND;

  err = dev_nfc_wait_request(accessor, &rq);

  *picc = rq.picc;

  return err;
});

/** @This performs a REQ, then selects first PICC. */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_request_find(const struct device_nfc_s *accessor,
                                  struct dev_nfc_picc_s *picc)
{
  struct dev_nfc_rq_s rq;
  error_t err;

  rq.type = DEV_NFC_REQUEST_FIND;

  err = dev_nfc_wait_request(accessor, &rq);

  *picc = rq.picc;

  return err;
});

/** @This performs a REQ, then selects given PICC. */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_activate(const struct device_nfc_s *accessor,
                              const struct dev_nfc_picc_s *picc)
{
  struct dev_nfc_rq_s rq;

  rq.type = DEV_NFC_ACTIVATE;
  rq.picc = *picc;

  return dev_nfc_wait_request(accessor, &rq);
});

/** @This performs a TRANSCEIVER configuration. */
config_depend_and2_inline(CONFIG_DEVICE_NFC, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_nfc_wait_config(const struct device_nfc_s *accessor,
                                const struct dev_nfc_config_s *config)
{
  struct dev_nfc_rq_s rq;

  rq.type = DEV_NFC_CONFIG;
  rq.config = *config;

  return dev_nfc_wait_request(accessor, &rq);
});


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


#endif

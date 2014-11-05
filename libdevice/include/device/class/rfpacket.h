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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014
    Copyright Nicolas Pouillon, <nipo@ssji.net> (c) 2014

    API for packet based Radio Frequency devices.
*/


#ifndef __DEVICE_RFPACKET_H__
#define __DEVICE_RFPACKET_H__

#include <mutek/kroutine.h>
#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/gpct_platform_hexo.h>

#include <device/driver.h>
#include <device/class/timer.h>
#include <device/request.h>

/** @This specifies modulation type */
enum dev_rfpacket_modulation_e
{
  DEV_RFPACKET_FSK,
  DEV_RFPACKET_GFSK,
  DEV_RFPACKET_ASK,
  DEV_RFPACKET_MSK,
  DEV_RFPACKET_CW,
};

enum dev_rfpacket_preamble_e
{

  DEV_RFPACKET_01,
  DEV_RFPACKET_10,
  DEV_RFPACKET_ADAPTATIVE,
};

enum dev_rfpacket_encoding_e
{
  DEV_RFPACKET_MANCHESTER,
  DEV_RFPACKET_LFSR8,
};

enum dev_rfpacket_crc_e
{
  DEV_RFPACKET_CRC_NONE,
  DEV_RFPACKET_CRC8,
  DEV_RFPACKET_CRC16,
  DEV_RFPACKET_CRC24,
  DEV_RFPACKET_CRC32,
};

struct device_rfpacket_s;
struct dev_rfpacket_rq_s;

enum dev_rfpacket_cfg_msk_e
{
  DEV_RFPACKET_FREQUENCY     = (1 <<  0),
  DEV_RFPACKET_DEVIATION_BW  = (1 <<  1),
  DEV_RFPACKET_DRATE         = (1 <<  2),
  DEV_RFPACKET_MOD           = (1 <<  3),
  DEV_RFPACKET_SYNC_WORD     = (1 <<  4),
  DEV_RFPACKET_SYMBOLS       = (1 <<  5),
  DEV_RFPACKET_RSSI_THRD     = (1 <<  6),
  DEV_RFPACKET_ENCODING      = (1 <<  7),
  DEV_RFPACKET_CRC           = (1 <<  8),
  DEV_RFPACKET_PREAMBLE_LEN  = (1 <<  9),
  DEV_RFPACKET_PREAMBLE_TYPE = (1 << 10),
  DEV_RFPACKET_SYNC_WORD_LEN = (1 << 11),
};

struct dev_rfpacket_config_s
{
  /** RF frequency in Hz */
  uint32_t                      frequency;

  union {
    /** frequency deviation in Hz (FSK modulation) */
    uint32_t                    deviation:24;
    /** bandwidth in Hz (OOK modulation) */
    uint32_t                    bw:24;
  };

  /** Datarate in bps */
  uint32_t                      drate:24;

  /** Modulation */
  enum dev_rfpacket_modulation_e  mod:8;

  /** Sync word value */
  uint32_t                      sw_value;

  /** number of symbols */
  uint8_t                      symbols;

  /** RSSI threshold in 0.125 dBm for RX */
  int16_t                       rssi_th;

  /** Packet handler configuration */
  enum dev_rfpacket_encoding_e  encoding:4;

  /** Use of CRC field */
  enum dev_rfpacket_crc_e       crc:4;

  /** Size of TX preamble in bits */
  uint8_t                      tx_pb_len;

  /** Size of RX preamble in bits */
  uint8_t                      rx_pb_len;

  /** preamble pattern: 0 is 010101..., 1 is 101010... */
  enum dev_rfpacket_preamble_e  pb_type:2;

  /** Size of sync word in bits minus one */
  uint32_t                      sw_len:5;
};

struct dev_rfpacket_statistics_s
{
  /** Number of rx request */
  uint32_t                      rx_count;
  /** Number of tx request */
  uint32_t                      tx_count;
  /** Number of rx error: CRC, Overflow */
  uint32_t                      rx_err_count;
  /** Number of tx error: Underflow */
  uint32_t                      tx_err_count;
};

/** Rf packet RX buffer */
struct dev_rfpacket_rx_s
{
  struct dev_request_s base;

  uint8_t                           *buf;           //< RX buffer, may be NULL
  uint16_t                          size;           //< Length of RX buffer, updated on packet reception
  int16_t                           rssi;           //< value of retrieved rssi in 0.125 dBm unit
  dev_timer_value_t                 timestamp;
};

enum dev_rfpacket_rq_rtype_e
{
  /* change the configuration of the transceiver. */
  DEV_RFPACKET_RQ_CONFIG,
  /* try to transmit a packet. */
  DEV_RFPACKET_RQ_TX,
  /* leave the transceiver in RX state during a given period. */
  DEV_RFPACKET_RQ_RX,
  /* put the transceiver in IDLE state. */
  DEV_RFPACKET_RQ_IDLE,
};

enum dev_rfpacket_time_anchor_e
{
  /* The request lifetime is relative to the request enqueue call. */
  DEV_RFPACKET_TIME_START,
  /* The request lifetime is relative to the request enqueue call of the previous request. */
  DEV_RFPACKET_TIME_PREV_START,
  /* The request lifetime is relative to the request end of the previous request. */
  DEV_RFPACKET_TIME_PREV_END,
};


struct dev_rfpacket_rq_s
{
  struct dev_request_s base;

  error_t                           err;            //< error code set by driver
  bool_t                            err_group:1;
  enum dev_rfpacket_rq_rtype_e      type:2;

  union {
    struct {
      uint32_t                      lifetime;      //< request timeout in 1us unit
      enum dev_rfpacket_time_anchor_e anchor:2;

      struct {
        dev_timer_value_t         timestamp;

        const uint8_t             *buf;
        uint16_t                  size;            //< length of TX packet in bytes

        int16_t                   pwr;             //< value of TX power in 0.125 dBm unit
      } tx;
    };

    struct {
      const struct dev_rfpacket_config_s *param;
      enum dev_rfpacket_cfg_msk_e         mask;
    } cfg;
  };

  const struct device_rfpacket_s    *accessor;         //< associated rfp device
};

/** @see dev_rfpacket_request_t */
#define DEV_RFPACKET_REQUEST(n)	void  (n) (const struct device_rfpacket_s *accessor, ...)

/**
  This function enqueues multiple @ref dev_rfpacket_rq_s requests
  atomically. The argument list of the function must be @tt NULL
  terminated. The first request is not started until all requests were
  properly enqueued.

  The @ref dev_rfpacket_rq_s::kr field of the request must always be
  initialized. The @ref kroutine_exec function will be called on @tt
  kr when the request ends. The @tt err field indicates the error
  status of the request.

  Some requests are designed to allow batching state changes of the
  transceiver. When there is no need to do so, there is no need to use
  @ref DEV_RFPACKET_RQ_RX and @ref DEV_RFPACKET_RQ_IDLE requests.

  The time frame during which a request remains valid is defined by
  the @refdev_rfpacket_rq_s::lifetime field. The start of this time
  frame can be relative to the previously handled request, even if
  this request doesn't exist anymore. The value of the @tt anchor
  field defines this behavior (see @ref dev_rfpacket_time_anchor_e).
  @ref DEV_RFPACKET_RQ_RX and @ref DEV_RFPACKET_RQ_TX requests which
  terminate by exceeding their lifetime have their @tt err field set
  to @ref -ETIMEDOUT.

  Multiple packets can be received during a @ref DEV_RFPACKET_RQ_RX
  request, the request is not terminated until its lifetime expires.
  Incoming frames are ignored if there are no more available RX buffers;
  this is not an error case and doesn't terminate an ongoing request.
  The actual execution time of a request may be longer
  than the expected lifetime of the request in case of an ongoing
  RX. Partially received frames, frames which do not fit the RX buffer
  and frames with a bad CRC are not reported and should not end the
  current request.

  When a @ref DEV_RFPACKET_RQ_TX request succeed, the value of the @tt
  tx.timestamp field is set to the actual date of transmission start.

  @ref DEV_RFPACKET_RQ_IDLE requests can have their @tt lifetime field
  set to zero. In this case, the request remains valid until an other
  request is enqueued. When this happen, the infinite request is
  discarded and the associated kroutine is called with the @tt err
  field of the request set to @tt -EBUSY.

  If the @tt err field is not 0 when the request ends, all subsequent
  requests with a matching @tt err_group field will be discarded with
  their @tt err field set to @tt -ECANCELED.

  If the requested configuration is not supported, the request will
  return @tt -ENOTSUP. The @ref msk provide a mask to the driver to
  quickly identify which parameter have changed. This field is set
  to @tt -1 when no mask is provided. In this case the driver must 
  either reconfigure all parameters or embed a mechanism to detect
  which parameters have changed.
  

  The transceiver device may implement the timer device class which
  wrap the timer used to handle timestamps and lifetime values.

  @table 4 {Error values}

  @item Error @item Config @item TX @item RX @item Idle
  @c---------------------------------------------------

  @item 0
  @item Success
  @item TX done
  @item Some RX done
  @item Lifetime exceeded

  @item @tt -ETIMEDOUT
  @item -
  @item Unable to TX during lifetime
  @item Lifetime exceeded without RX
  @item -

  @item @tt -ECANCELLED
  @item Has fail_with_previous and previous failed
  @item Has fail_with_previous and previous failed
  @item Has fail_with_previous and previous failed
  @item Has fail_with_previous and previous failed

  @item @tt -ENOTSUP
  @item Request with unsupported values
  @item -
  @item -
  @item -

  @item @tt -EBUSY
  @item -
  @item -
  @item -
  @item Infinite idle aborted

  @end table

*/
typedef DEV_RFPACKET_REQUEST(dev_rfpacket_request_t);


/** @see dev_rfpacket_request_t */
#define DEV_RFPACKET_RECEIVE(n) void  (n) (const struct device_rfpacket_s *accessor, \
                                            struct dev_rfpacket_rx_s *rx)
/**
  This function enqueues a @ref dev_rfpacket_rx_s buffer.

  The @ref dev_rfpacket_rx_s::kr field of the buffer must always be
  initialized.

  If the device has been started by calling @ref device_start, the transceiver
  is able to receive packets when some @ref dev_rfpacket_rx_s objects are
  available in the RX queue, even if the @ref dev_rfpacket_rq_s queue is
  empty, provided that no @ref DEV_RFPACKET_RQ_IDLE request is currently
  being executed. Packets can also be received during the CCA period of
  a TX request if the CCA is enabled in the configuration and supported
  by the transcieiver.

  When the device is not started, the transceiver is able to receive
  packets only during DEV_RFPACKET_RQ_RX requests.

  When a packet is received successfully, the next @ref
  dev_rfpacket_rx_s object is popped and its kroutine is called. The
  @tt timestamp field is set to the date of the beginning of frame.

 */
typedef DEV_RFPACKET_RECEIVE(dev_rfpacket_receive_t);

DRIVER_CLASS_TYPES(rfpacket,
                   dev_rfpacket_request_t *f_request;
                   dev_rfpacket_receive_t *f_receive;
                  );

#endif


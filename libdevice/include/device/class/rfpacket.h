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

/**
   @file
   @module {Core::Devices support library}
   @short RF transceiver packet interface driver API
   @index {RF transceiver packet interface} {Device classes}
   @csee DRIVER_CLASS_RFPACKET
*/

#ifndef __DEVICE_RFPACKET_H__
#define __DEVICE_RFPACKET_H__

#include <mutek/kroutine.h>
#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/class/timer.h>
#include <device/request.h>

#include <hexo/enum.h>

struct device_rfpacket_s;
struct dev_rfpacket_rq_s;

/** RF power in 0.125 dBm steps */
typedef int16_t dev_rfpacket_pwr_t;

/** This tracks configuration cache entry status. This is used in @ref
    dev_rfpacket_rf_cfg_s and @ref dev_rfpacket_pk_cfg_s.

    This allows reuse of a cached configuration when the id is known
    to the driver and matches the pointer to the config object. When
    configuration requires computation and caching, the application
    expects the driver to perform the cache hit test in this way:

    @code
    struct cache_entry_s *e = &cache_array[cfg->cache.id % CACHE_SIZE];
    if (e->cfg == cfg && !cfg->dirty)
      {
        // use cache entry
      }
    else
      {
        // update cache entry
        if (e->dirty)
          e->dirty = 0;
      }

    @end code 

    A configuration can not be modified while it is already in use by
    a request.

    */
struct dev_rfpacket_cfg_cache_s
{
  /** Configuration id provided by the application. It is used by the
      driver to track configuration changes and optionally store
      configuration derived values in a cache slot. */
  uint8_t                       BITFIELD(id,7);
  /** The application must set the field when the content of the
      configuration structure is updated. The driver will set this
      field to 0 only if not already clear. This allows storing
      configurations as const data. */
  uint8_t                       BITFIELD(dirty,1);
};

/***************************************** RF config */

ENUM_DESCRIPTOR(dev_rfpacket_modulation_e, strip:DEV_RFPACKET_, upper);

/** @This specifies the RF modulation type */
enum dev_rfpacket_modulation_e
{
  /** Frequency shift keying modulation
      @see dev_rfpacket_rf_cfg_fsk_s */
  DEV_RFPACKET_FSK,
  /** Gaussian frequency shift keying modulation
      @see dev_rfpacket_rf_cfg_fsk_s */
  DEV_RFPACKET_GFSK,
  /** Amplitude shift keying modulation
      @see dev_rfpacket_rf_cfg_ask_s */
  DEV_RFPACKET_ASK,
  /** Lora modulation
      @see dev_rfpacket_rf_cfg_lora_s */
  DEV_RFPACKET_LORA,
  /** Sigfox modulation
      @see dev_rfpacket_rf_cfg_sigfox_s */
  DEV_RFPACKET_SIGFOX,
  /** IEEE-Std-802.15.4z UWB HRP
      @see dev_rfpacket_rf_cfg_uwb_s */
  DEV_RFPACKET_UWB,
};

/** This stores RF and modulation configuration. This may be inherited
    depending on the value of the @tt mod field. */
struct dev_rfpacket_rf_cfg_s
{
  /** Modulation type */
  enum dev_rfpacket_modulation_e BITFIELD(mod,8);

  /** Configuration cache entry. the @tt use flag must be reset when the
      configuration changes. */
  struct dev_rfpacket_cfg_cache_s cache;

  /** During a RX, a continuous signal above this level is considered
      jamming and will be reported as an error. */
  dev_rfpacket_pwr_t            jam_rssi;

  /** Datarate in bps */
  uint32_t                      BITFIELD(drate,24);

  /** RF frequency in Hz */
  uint32_t                      frequency;

  /** RF frequency in Hz */
  uint32_t                      chan_spacing;

  /** Receiver bandwidth in Hz. Computed automatically when 0. */
  uint32_t                      rx_bw;

  /** Maximum expected frequency error of the remote
      transmitter. Expressed as @em {abs(actual tx freq - expected freq)}.

      The frequency error contribution on the receiver side due to the
      local oscilator (@em {abs(actual rx freq - expected freq)})
      should be added by the driver and not included here. */
  uint32_t                      freq_err;
};

STRUCT_DESCRIPTOR(dev_rfpacket_rf_cfg_s);

/** @This specifies the policy of fair TX */
enum dev_rfpacket_fairtx_e
{
  DEV_RFPACKET_NO_FAIRTX = 0,
  DEV_RFPACKET_LBT,
};

/** @This store the configuration associated to the policy of fair TX.
    It may be used in some modulation configuration structures. */
struct dev_rfpacket_rf_cfg_fairtx_s
{
  union {
    struct {
      /** Listening time in timer unit. Defined as the minimum time that the
          device must listen prior to determine whether the intended channel
          is available for use.**/
      dev_timer_delay_t             duration;
      /** RX RSSI threshold. During a TX listen before talk, signal below
          this level will allow start of transmission. */
      dev_rfpacket_pwr_t            rssi;
    } __attribute__((packed)) lbt;
    struct {
    } __attribute__((packed)) csma;

  } __attribute__((packed));


  enum dev_rfpacket_fairtx_e BITFIELD(mode, 8);
};

/** @This extends the @ref dev_rfpacket_rf_cfg_s object when the @ref
    DEV_RFPACKET_FSK or @ref DEV_RFPACKET_GFSK modulations are in
    use. */
struct dev_rfpacket_rf_cfg_fsk_s
{
  struct dev_rfpacket_rf_cfg_s  base;
  struct dev_rfpacket_rf_cfg_fairtx_s fairtx;

  /** frequency deviation in Hz */
  uint32_t                      BITFIELD(deviation,24);

  /** number of symbols */
  uint32_t                      BITFIELD(symbols,8);
};

STRUCT_INHERIT(dev_rfpacket_rf_cfg_fsk_s, dev_rfpacket_rf_cfg_s, base);

/** @This extends the @ref dev_rfpacket_rf_cfg_s object when the @ref
    DEV_RFPACKET_ASK modulation is in use. */
struct dev_rfpacket_rf_cfg_ask_s
{
  struct dev_rfpacket_rf_cfg_s  base;
  struct dev_rfpacket_rf_cfg_fairtx_s fairtx;

  /** number of symbols, 2 for OOK */
  uint32_t                      BITFIELD(symbols,8);
};

STRUCT_INHERIT(dev_rfpacket_rf_cfg_ask_s, dev_rfpacket_rf_cfg_s, base);

/** @This extends the @ref dev_rfpacket_cfg_s object when the @ref
 * DEV_RFPACKET_LORA modulation is in use. */
struct dev_rfpacket_rf_cfg_lora_s
{
  struct dev_rfpacket_rf_cfg_s base;

  /** spreading factor (in log2 basis). */
  uint8_t                      BITFIELD(spreading,4);

  /** inverted I/Q signals */
  bool_t                       BITFIELD(iq_inverted,1);
};

STRUCT_INHERIT(dev_rfpacket_rf_cfg_lora_s, dev_rfpacket_rf_cfg_s, base);

/**
   UWB
 */

/** @This extends the @ref dev_rfpacket_cfg_s object when the @ref
 * DEV_RFPACKET_UWB modulation is in use.
 *
 * For this mode, @tt frequency member in base configuration structure
 * is not in Hz, but in kHz (or it would overflow).
 */
struct dev_rfpacket_rf_cfg_uwb_s
{
  struct dev_rfpacket_rf_cfg_s base;
};

// See IEEE-Std-802.15.4-2020 Table 10-6
#define DEV_RFPACKET_HRP_UWB_CHANNEL_0  499200
#define DEV_RFPACKET_HRP_UWB_CHANNEL_1 3494400
#define DEV_RFPACKET_HRP_UWB_CHANNEL_2 3993600
#define DEV_RFPACKET_HRP_UWB_CHANNEL_3 4492800
#define DEV_RFPACKET_HRP_UWB_CHANNEL_4 3933600
#define DEV_RFPACKET_HRP_UWB_CHANNEL_5 6489600
#define DEV_RFPACKET_HRP_UWB_CHANNEL_6 6988800
#define DEV_RFPACKET_HRP_UWB_CHANNEL_7 6489600
#define DEV_RFPACKET_HRP_UWB_CHANNEL_8 7488000
#define DEV_RFPACKET_HRP_UWB_CHANNEL_9 7987200
#define DEV_RFPACKET_HRP_UWB_CHANNEL_10 8486400
#define DEV_RFPACKET_HRP_UWB_CHANNEL_11 7987200
#define DEV_RFPACKET_HRP_UWB_CHANNEL_12 8985600
#define DEV_RFPACKET_HRP_UWB_CHANNEL_13 9484800
#define DEV_RFPACKET_HRP_UWB_CHANNEL_14 9984000
#define DEV_RFPACKET_HRP_UWB_CHANNEL_15 9484800


// See IEEE-Std-802.15.4z-2020 Table 10-13
#define DEV_RFPACKET_LRP_UWB_CHANNEL_0 6489600
#define DEV_RFPACKET_LRP_UWB_CHANNEL_1 6988800
#define DEV_RFPACKET_LRP_UWB_CHANNEL_2 7987200
#define DEV_RFPACKET_LRP_UWB_CHANNEL_3 8486400
#define DEV_RFPACKET_LRP_UWB_CHANNEL_4 6681600
#define DEV_RFPACKET_LRP_UWB_CHANNEL_5 7334400
#define DEV_RFPACKET_LRP_UWB_CHANNEL_6 7987200
#define DEV_RFPACKET_LRP_UWB_CHANNEL_7 8640000
#define DEV_RFPACKET_LRP_UWB_CHANNEL_8 9292800
#define DEV_RFPACKET_LRP_UWB_CHANNEL_9 9945600

STRUCT_INHERIT(dev_rfpacket_rf_cfg_uwb_s, dev_rfpacket_rf_cfg_s, base);

/***************************************** packet format config */

ENUM_DESCRIPTOR(dev_rfpacket_format_e, strip:DEV_RFPACKET_, upper);

enum dev_rfpacket_format_e
{
  /* Sync word, 1 byte Len, Payload, CRC
     @see dev_rfpacket_pk_cfg_basic_s */
  DEV_RFPACKET_FMT_SLPC,

  /* UART mode. Each byte of packet has a start and stop bit added. */
  DEV_RFPACKET_FMT_IO,

  /* Sync word, optional header, Payload, CRC. This is used when
     @ref dev_rfpacket_pk_cfg_lora_s is used. */
  DEV_RFPACKET_FMT_LORA,

  /* Raw mode. Data is provided as a list of duration encoding symbols
     0 and 1. The first duration value is always for symbol 0 but might
     be set to 0 is necessary. Symbol duration is expressed in
     @ref dev_rfpacket_pk_cfg_raw_s::unit
     */
  DEV_RFPACKET_FMT_RAW,

  DEV_RFPACKET_FMT_SIGFOX,

  /* IEEE-Std-802.15.4z-2020 UWB */
  DEV_RFPACKET_FMT_UWB,
};

ENUM_DESCRIPTOR(dev_rfpacket_encoding_e, strip:DEV_RFPACKET_, upper);

enum dev_rfpacket_encoding_e
{
  DEV_RFPACKET_CLEAR,
  DEV_RFPACKET_MANCHESTER,
  DEV_RFPACKET_LFSR8,
  /* Direct Sequence Spread Spectrum */
  DEV_RFPACKET_DSSS, 
  /* Convolutional encoding, like in IEEE-Std-802.15.4z-2020 (See 15.3.3) */
  DEV_RFPACKET_CONVOLUTIONAL_K3,
  DEV_RFPACKET_CONVOLUTIONAL_K7,
};

/** This stores packet format configuration. This may be inherited
    depending on the value of the @tt format field. */
struct dev_rfpacket_pk_cfg_s
{
  /** Packet format */
  enum dev_rfpacket_format_e    BITFIELD(format,8);

  /** Data encoding and whitening */
  enum dev_rfpacket_encoding_e  BITFIELD(encoding,4);

  /** Configuration cache entry. the @tt use flag must be reset when the
      configuration changes. */
  struct dev_rfpacket_cfg_cache_s cache;
};

STRUCT_DESCRIPTOR(dev_rfpacket_pk_cfg_s);

struct dev_rfpacket_pk_cfg_basic_s
{
  struct dev_rfpacket_pk_cfg_s  base;

  /** Specifies the CRC polynomial when relevant. for instance IBM
      CRC16 has value 0x18005. Zero means that the CRC field is not
      checked. CRC is always transmitted Most significant bits first */
  uint32_t                      crc;

  /** Specifies the CRC initialization value */
  uint32_t                      crc_seed;

  /** Sync word value. Most significant bits are transmitted first */
  uint32_t                      sw_value;

  /** preamble pattern. Most significant bits are transmitted first */
  uint32_t                      pb_pattern;

  /** Size of sync word in bits minus one */
  uint32_t                      BITFIELD(sw_len,5);

  /** preamble pattern len in bits minus one */
  uint32_t                      BITFIELD(pb_pattern_len,5);

  /** Size of transmitted preamble in bits. When the requested value
      is not supported, it is rounded to a higher value by the driver. */
  uint16_t                      tx_pb_len;

  /** Size of expected preamble lenght in bits. The driver will
      configure the hardware so that a packet transmitted with a
      preamble lenght equal or greater than this is received properly. */
  uint16_t                      rx_pb_len;
};

STRUCT_INHERIT(dev_rfpacket_pk_cfg_basic_s, dev_rfpacket_pk_cfg_s, base);

/** @This specifies the integer type used to store symbols for a
    raw request. */
enum dev_rfpacket_sym_width_e
{
  /** Symbols are stored as @tt uint8_t */
  DEV_RFPACKET_RAW_8BITS,
  /** Symbols are stored as @tt uint16_t */
  DEV_RFPACKET_RAW_16BITS,
  /** Symbols are stored as @tt uint32_t */
  DEV_RFPACKET_RAW_32BITS,
};

struct dev_rfpacket_pk_cfg_raw_s
{
  struct dev_rfpacket_pk_cfg_s   base;
  /* Maximum packet size in number of symbol */ 
  uint16_t                       mps;
  /* This defines the base unit time use for symbol duration */
  struct dev_freq_s              unit;
  /* Timeout in unit used to detect end of a RX packet */
  uint32_t                       timeout;
  /** This specifies the type of integer used for symbols */
  enum dev_rfpacket_sym_width_e  sym_width;
};

STRUCT_INHERIT(dev_rfpacket_pk_cfg_raw_s, dev_rfpacket_pk_cfg_s, base);

ENUM_DESCRIPTOR(dev_rfpacket_lora_encoding_e, strip:DEV_RFPACKET_, upper);

enum dev_rfpacket_lora_encoding_e
{
  DEV_RFPACKET_LORA_CR_45 = 1,
  DEV_RFPACKET_LORA_CR_46,
  DEV_RFPACKET_LORA_CR_47,
  DEV_RFPACKET_LORA_CR_48
};

struct dev_rfpacket_pk_cfg_lora_s
{
  struct dev_rfpacket_pk_cfg_s      base;

  /** Set when using explicit header */
  bool_t                            BITFIELD(header,1);

  /** Set when payload CRC is enabled. */
  bool_t                            BITFIELD(crc,1);

  /** Coding rate of the payload. Values must be specified as 4/@tt crate. */
  enum dev_rfpacket_lora_encoding_e BITFIELD(crate,3);

  /** Sync word value. Most significant bit are transmitted first */
  uint8_t                           sw_value;

  /** Size of preamble in symbols. When the requested value
      is not supported, it must be rounded to a higher value. */
  uint16_t                          pb_len;
};

STRUCT_INHERIT(dev_rfpacket_pk_cfg_lora_s, dev_rfpacket_pk_cfg_s, base);

/**
   UWB
 */

/** Packet formats, See 15.2.1 */
enum dev_rfpacket_uwb_format_e
{
  // SYNC, SFD, PHR, Payload
  DEV_RFPACKET_FMT_UWB_0,
  // SYNC, SFD, STS, PHR, Payload
  DEV_RFPACKET_FMT_UWB_1,
  // SYNC, SFD, PHR, Payload, STS
  DEV_RFPACKET_FMT_UWB_2,
  // SYNC, SFD, STS
  DEV_RFPACKET_FMT_UWB_3,
};

/** PSDU Bit Rates, See 15.2.7.2 Table 15-9a */
enum dev_rfpacket_uwb_bit_rate_e
{
  /* PHR 975 kb/s (850 kb/s nominal), PSDU 6.8 Mb/s */
  DEV_RFPACKET_UWB_DRBM_LP,
  /* PHR 7.8 Mb/s (6.8 Mb/s nominal), PSDU 6.8 Mb/s */
  DEV_RFPACKET_UWB_DRBM_HP,
};

struct dev_rfpacket_pk_cfg_uwb_s
{
  struct dev_rfpacket_pk_cfg_s      base;

  /** See 15.2.1, Table 15-a */
  enum dev_rfpacket_uwb_format_e BITFIELD(format,2);

  /** See 15.2.5.1 Table 16-6 and 16-7, 15.2.6, Table 15-7a */
  uint32_t                   BITFIELD(sync_code,6);

  /** See 15.2.6, Table 15-7c */
  uint32_t                   BITFIELD(sfd_len,6);
  uint32_t                   sfd;

  uint8_t sts_master_key[16];
  uint8_t sts_data[16];
};

STRUCT_INHERIT(dev_rfpacket_pk_cfg_uwb_s, dev_rfpacket_pk_cfg_s, base);


/***************************************** stats */

/** @see dev_rfpacket_stats_t */
struct dev_rfpacket_stats_s
{
  /** Number of rx packet */
  uint32_t                      rx_count;
  /** Number of tx packet */
  uint32_t                      tx_count;
  /** Number of rx packet error: CRC, Overflow */
  uint32_t                      rx_err_count;
  /** Number of tx packet error: Underflow, timeout  */
  uint32_t                      tx_err_count;
};

/** @see dev_rfpacket_stats_t */
#define DEV_RFPACKET_STATS(n) error_t  (n) (const struct device_rfpacket_s *accessor, \
                                            struct dev_rfpacket_stats_s *stats)

/** @This return some stat counters. */
typedef DEV_RFPACKET_STATS(dev_rfpacket_stats_t);


/***************************************** RX packet buffers */

/** @see dev_rfpacket_rx_alloc_t */
#define DEV_RFPACKET_RX_ALLOC(n)                \
  struct dev_rfpacket_rx_s * (n)(struct dev_rfpacket_rq_s *rq, size_t size)

/** This is used in RX @ref dev_rfpacket_rq_s {requests} to allocate a
    @ref dev_rfpacket_rx_s object along with a buffer for incoming packets.

    The @tt size parameter specifies the size of the allocated payload
    buffer pointed to by the @ref dev_rfpacket_rx_s::buf field. It is
    also the initial value of the @ref dev_rfpacket_rx_s::size field
    which might be lowered later. Partially received frames, frames
    with a bad CRC or other receiving errors must be reported with a
    null size.

    The incoming packet is discarded when this function returns @tt
    NULL. In the other case, the driver always execute the @ref
    dev_rfpacket_rx_s::kr kroutine function at some point, even if the
    associated RX request has ended.
*/
typedef DEV_RFPACKET_RX_ALLOC(dev_rfpacket_rx_alloc_t);

/** RX buffer object, allocated by the @ref dev_rfpacket_rx_alloc_t
    function. */
struct dev_rfpacket_rx_s
{
  /** Time stamp of the start of received packet. */
  dev_timer_value_t                 timestamp;

  /** This is executed by the driver when the RX operation is
      complete. */
  struct kroutine_s                 kr;

  /** Received payload buffer, allocated by the @ref
      dev_rfpacket_rx_alloc_t function. */
  void                              *buf;

  /** Actual size of the received payload. This is initially set by
      the @ref dev_rfpacket_rx_alloc_t function and updated by the
      driver if the packet is actually smaller. */
  uint16_t                          size;

  /** RX signal level of radio environment. */
  dev_rfpacket_pwr_t                rssi;

  /** RX signal level of the received packet. */
  dev_rfpacket_pwr_t                carrier;

  /** RX signal power over noise power ratio. */
  dev_rfpacket_pwr_t                snr;

  /** Measured RX frequency */
  uint32_t                          frequency;

  /** Channel of the received packet. */
  uint8_t                           channel;

  /** When the opration completes and the packet is malformed,
      contains bit errors or has a bad CRC, this field is set to @tt
      -EBADDATA by the driver. The @tt -EIO error indicates that the
      driver was not able to receive and store the packet properly.

      If no data is available due to the error, the @tt size field is
      set to 0. */
  error_t                           error;
};

STRUCT_COMPOSE(dev_rfpacket_rx_s, kr);

/***************************************** requests */

enum dev_rfpacket_rq_rtype_e
{
  /** Schedule a packet transmit. This is pushed on the requests queue
      and will be processed in order. It will not run until the @ref
      dev_rfpacket_rq_s::deadline has been reached.

      This request can not be used with a deadline when a @ref
      DEV_RFPACKET_RQ_RX_CONT request is running. */
  DEV_RFPACKET_RQ_TX,

  /** Schedule a packet transmit, monitoring the channel before
      transmit. This is pushed on the requests queue and will be
      processed in order. It will not run until the @ref
      dev_rfpacket_rq_s::deadline has been reached. The request will
      terminate with the @tt -ETIMEDOUT error if the channel does not
      become clear during the whole lifetime period. The mechanism 
      used to determine if the transmission can start is defined in 
      the configuration.

      When the hardware support this, some packet may be received
      during the monitoring period if a @ref DEV_RFPACKET_RQ_RX_CONT
      request is active and share the same configuration and channel
      as this TX request. In this case the packet should not be
      transmitted, terminating the request with the @tt -EAGAIN error.

      This request can not be used with a deadline when a @ref
      DEV_RFPACKET_RQ_RX_CONT request is running. */
  DEV_RFPACKET_RQ_TX_FAIR,

  /** Schedule an RX period. This is pushed on the
      requests queue and will be processed in order. It will not run
      until the @ref dev_rfpacket_rq_s::deadline has been reached.
      The request terminates successfully when the RX lifetime is
      reached.

      Multiple packets can be received during the period, calling the
      @ref dev_rfpacket_rq_s::rx_alloc function and the @ref
      dev_rfpacket_rx_s::kr kroutine for each incoming packet. The
      request terminates successfully when its lifetime expires,
      receiving packets do not terminate the request.

      The request will terminate with the @tt -EBUSY error if the
      channel is jammed according to the value of the @ref
      dev_rfpacket_rf_cfg_s::rssi_th field.

      This request can not be used with a deadline when a @ref
      DEV_RFPACKET_RQ_RX_CONT request is running. */
  DEV_RFPACKET_RQ_RX,

  /** Put the transceiver in continuous RX state. This request is not
      pushed in the queue of scheduled requests. An other queue is
      used for @ref DEV_RFPACKET_RQ_RX_CONT and @ref
      DEV_RFPACKET_RQ_RX_TIMEOUT requests. When the hardware supports
      listening on multiple rf and pkt configurations at the same
      time, these requests are served in parallel. In the other case,
      the request with the shortest timeout are served first.

      Some scheduled requests with a null deadline can be used at the
      same time as continuous RX requests. In this case, the
      continuous RX request stays in the background and will resume
      operation as soon as the scheduled requests end.

      This request terminates with the @tt -EAGAIN error if the
      channel is jammed according to the value of the @ref
      dev_rfpacket_rf_cfg_s::rssi_th field. The @ref
      dev_rfpacket_cancel_t function can be used to end this type of
      request. */
  DEV_RFPACKET_RQ_RX_CONT,

  /** Put the transceiver in continuous RX state for a limited period
      of time. This is similar to a @ref DEV_RFPACKET_RQ_RX_CONT
      request but the @ref dev_rfpacket_rq_s::deadline and @ref
      dev_rfpacket_rq_s::lifetime fields are not ignored. Instead they
      are used to specify the when the RX must end.

      If the @tt lifetime field is not zero, the @tt deadline field
      will be computed automatically. In the other case, an absolute
      deadline timer value must be provided in this field. */
  DEV_RFPACKET_RQ_RX_TIMEOUT,
};

enum dev_rfpacket_timestamp_anchor_e
{
  /* Timestamp is relative to the effective start of packet date */
  DEV_RFPACKET_TIMESTAMP_START,
  /* Timestamp is relative to the end of packet date */
  DEV_RFPACKET_TIMESTAMP_END,
};

struct dev_rfpacket_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** This specifies the RF configuration to use during the request. */
  const struct dev_rfpacket_rf_cfg_s *rf_cfg;
  /** This specifies the packet format configuration to use during the
      request. */
  const struct dev_rfpacket_pk_cfg_s *pk_cfg;

  union {
    /** This specifies when a RX or TX operation must start. The request
        will be started immediately if the current timer value is
        greater than this value. This field is not used by @ref
        DEV_RFPACKET_RQ_RX_CONT requests. */
    dev_timer_value_t               deadline;

    /** This field is updated by the driver to the actual start of
        transmit time for @ref DEV_RFPACKET_RQ_TX and @ref
        DEV_RFPACKET_RQ_TX_FAIR requests. */
    dev_timer_value_t               tx_timestamp;
  };

  /** For @ref DEV_RFPACKET_RQ_RX requests, this field defines the
      duration of the RX period. Multiple packets may be received during
      the request.
      For @ref DEV_RFPACKET_RQ_TX_LBT requests, this field specifies
      how long to wait for the channel to become clear before aborting
      the transmit.
      In both cases the actual execution time of a request may be
      longer than the lifetime because an ongoing RX may start at the
      end of the period and will not be aborted.
      This field is not used by @ref DEV_RFPACKET_RQ_RX_CONT and @ref
      DEV_RFPACKET_RQ_TX requests. */
  dev_timer_delay_t                 lifetime;

  /** This specifies an frequency offset for this request according to
      the @ref dev_rfpacket_rf_cfg_s::frequency and @ref
      dev_rfpacket_rf_cfg_s::chan_spacing values. */
  int16_t                           channel;

  /** Request completion error, set by the driver */

  /** When a request in the queue terminates with an error or get
      canceled, all subsequent requests with the same error groups are
      terminated along with the @tt -ECANCELED error code. */
  bool_t                            BITFIELD(err_group,1);

  /* This defines the timestamp anchor */
  enum dev_rfpacket_timestamp_anchor_e   BITFIELD(anchor,1); 

  /** This specifies the type of the request. */
  enum dev_rfpacket_rq_rtype_e      BITFIELD(type,3);

  union {
    struct {
      /** This is used to allocate a RX buffers to store the payload of
          the incoming packet when the request is running. The received
          packet is discarded if the function return @tt NULL. */
      dev_rfpacket_rx_alloc_t         *rx_alloc;
      /** This is a channel mask used in @ref DEV_RFPACKET_RQ_RX_CONT
          operations to perform scanning on multiple channels. Least
          significant bit of this mask corresponds to
          @ref dev_rfpacket_rq_s::channel and must always be set.
          When a  packet is received, @ref dev_rfpacket_rx_s::channel
          corresponds to the receiving channel. */
      uint32_t                        rx_chan_mask;
    };

    struct {
      /** This contains the payload to transmit. */
      const void                    *tx_buf;
      /** This specifies the size of the payload to transmit. */
      uint16_t                      tx_size;
      /** This specifies the TX power. */
      dev_rfpacket_pwr_t            tx_pwr;
    };
  };
};

DEV_REQUEST_INHERIT(rfpacket);
DEV_REQUEST_QUEUE_OPS(rfpacket);

GCT_CONTAINER_KEY_TYPES(dev_request_queue, CUSTOM, SCALAR,
                        dev_rfpacket_rq_s_cast(dev_request_queue_item)->deadline,
                        dev_rfpacket_queue);

GCT_CONTAINER_KEY_FCNS(dev_request_queue, ASC, inline, __dev_rfpacket_queue,
                       dev_rfpacket_queue,
                       insert);

/** @This insert a new rfpacket request in the queue in deadline
    order. */
ALWAYS_INLINE void
dev_rfpacket_rq_insert(dev_request_queue_root_t *q,
                       struct dev_rfpacket_rq_s *rq)
{
  assert(rq->base.pushed == 0xdead);
  __dev_rfpacket_queue_insert(q, &rq->base);
}

/** @see dev_rfpacket_request_t */
#define DEV_RFPACKET_REQUEST(n)	void  (n) (const struct device_rfpacket_s *accessor, ...)

/**
  This function enqueues multiple @ref dev_rfpacket_rq_s requests
  atomically. The argument list of the function must be @tt NULL
  terminated.

  Depending on the @ref dev_rfpacket_rq_type_e {type of request} and
  @ref dev_rfpacket_rq_s::deadline, the request might not run
  immediately.

  The configuration of the transceiver is updated according the
  request when the request actually runs. The configuration will not
  be updated if the @ref dev_rfpacket_rq_s::rf_cfg and @ref
  dev_rfpacket_rq_s::pk_cfg field are identical to the previously
  running request and the @ref dev_rfpacket_cfg_cache_s fields allow
  caching.

  The @ref kroutine_exec function will be called on @tt
  kr when the request ends. The @tt err field indicates the error
  status of the request.

  Some error codes are specific to a type of request. Other possible
  errors are:

  @list
  @item @tt -ECANCELED is used for contiguous requests in the queue
    with the same @ref dev_rfpacket_rq_s::err_group {group} as a
    canceled or failed request.
  @item @tt -ENOTSUP is used if the configuration is not
    supported by the driver.
  @item @tt -EIO is used when an hardware error or
    fifo underflow/overflow occurs.
  @end list

  The kroutine of the request may be executed from within this
  function. Please read @xref {Nested device request completion}.

  The driver of the transceiver may also implement the timer device
  class when an internal timer is available.
*/
typedef DEV_RFPACKET_REQUEST(dev_rfpacket_request_t);


/** @see dev_rfpacket_request_t */
#define DEV_RFPACKET_CANCEL(n)	error_t (n) (const struct device_rfpacket_s *accessor, \
                                             struct dev_rfpacket_rq_s *rq)

/*  @This forces early termination of a request which have previously
    been passed to the @ref dev_rfpacket_request_t function.

    The function returns 0 if the request has been canceled.
    In this case, the request kroutine is not executed
    and the @ref dev_rfpacket_rq_s can be reused immediately by the caller.
    
    This function returns @tt -EBUSY if the request has already ended
    normally or will terminate soon. In the later case, no particular
    error is reported on request completion. In the later case, some
    packets may still be received or transmitted before the request
    kroutine is executed.
*/
typedef DEV_RFPACKET_CANCEL(dev_rfpacket_cancel_t);


/******************************************/

DRIVER_CLASS_TYPES(DRIVER_CLASS_RFPACKET, rfpacket,
                   dev_rfpacket_request_t *f_request;
                   dev_rfpacket_cancel_t *f_cancel;
                   dev_rfpacket_stats_t *f_stats;
                  );

/** @see driver_rfpacket_s */
#define DRIVER_RFPACKET_METHODS(prefix)                            \
  ((const struct driver_class_s*)&(const struct driver_rfpacket_s){     \
    .class_ = DRIVER_CLASS_RFPACKET,                               \
    .f_request = prefix ## _request,                               \
    .f_cancel = prefix ## _cancel,                                 \
    .f_stats = prefix ## _stats,                                   \
  })

config_depend_and2_inline(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_rfpacket_wait_rq(
       const struct device_rfpacket_s *accessor,
       struct dev_rfpacket_rq_s *rq),
{
    if (rq->type == DEV_RFPACKET_RQ_RX_CONT)
      return -ENOTSUP;

    struct dev_request_status_s status;
    dev_request_sched_init(&rq->base, &status);
    DEVICE_OP(accessor, request, rq, NULL);
    dev_request_sched_wait(&status);
    return rq->error;
});

/** @This is a context used by blocking helper functions for the
    rfpacket class of devices. @see dev_rfpacket_wait_init */
struct dev_rfpacket_wait_ctx_s
{
#ifdef CONFIG_MUTEK_CONTEXT_SCHED
  const struct device_rfpacket_s *rf_dev;
  lock_t lock;
  struct sched_context_s *sched_ctx;
  struct dev_rfpacket_rq_s tx_rq;
  struct dev_rfpacket_rq_s rx_rq;
  struct dev_rfpacket_rx_s rx;
#endif
};

/** @This initializes a context for use by rfpacket blocking helper
    functions.  @csee dev_rfpacket_wait_init @csee
    dev_rfpacket_start_rx @csee dev_rfpacket_stop_rx @csee
    dev_rfpacket_wait_rx @csee dev_rfpacket_wait_tx @csee
    dev_rfpacket_wait_cleanup */
config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rfpacket_wait_init(struct dev_rfpacket_wait_ctx_s *ctx,
                               const struct device_rfpacket_s *rf_dev,
                               const struct dev_rfpacket_rf_cfg_s *rf_cfg,
                               const struct dev_rfpacket_pk_cfg_s *pk_cfg);

/** @This can be used to put the transceiver in RX state. This
    function does not block. The @ref dev_rfpacket_wait_rx must then
    be called in order to receive some packets.

    If the @tt timeout parameter is not 0, the RX will stop after the
    specified delay has elapsed. It can also be stopped by calling the
    @ref dev_rfpacket_stop_rx function. The RX may as well end at any
    time in case of error (bad config, jamming, ...).

    The function returns @tt -EBUSY if the transceiver is already in
    RX state. The function returns 0 in order to acknowledge the
    request to start the RX.

    The configuration objects passed to the @ref dev_rfpacket_wait_init
    function are used. If the configuration objects need to be updated,
    the RX must be stopped and started again.

    The @tt ctx object storage must remain valid until the RX is stopped. */
config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rfpacket_start_rx(struct dev_rfpacket_wait_ctx_s *ctx,
                              uint_fast16_t channel, dev_timer_delay_t timeout);

/** @This waits for an incoming packet and stores the received packet
    in the provided buffer. The @ref dev_rfpacket_start_rx function
    must have been called previously.

    The current scheduler context is stopped waiting for an incoming
    packet.

    @This may return the following errors:
    @list
      @item @tt -EBUSY if the transceiver is not currently in RX state.
      @item @tt -ETIMEDOUT if the timeout delay has elapsed while
        waiting for a packet.
      @item Any other error code reported by the transceiver error
        while waiting for a packet. This does not mean end of RX.
      @item @tt 0 if a packet has been received and stored in the
        provided buffer.
    @end list

    The @tt size parameter must initially indicate the size of the
    buffer and is updated with the size of the received
    packet. Packets too large to fit the provided buffer are ignored.

    Because receiving a packet does not remove the transceiver from RX
    state, this function can be called multiple times. */
config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rfpacket_wait_rx(struct dev_rfpacket_wait_ctx_s *ctx,
                             uint8_t *buffer, size_t *size);

/** @This provides a buffer in advance to store the next incoming
    packet. The @ref dev_rfpacket_wait_rx function must then be used
    with the same buffer. This helps being ready to catch a reply
    right after sending a packet.

    Once this function has been called, it is not allowed to call the
    @ref dev_rfpacket_stop_rx function without first calling the @ref
    dev_rfpacket_wait_rx function. */
config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rfpacket_prepare_rx(struct dev_rfpacket_wait_ctx_s *ctx,
                                uint8_t *buffer, size_t size);

/** @This can be used to remove the transceiver from RX state.
    @see dev_rfpacket_start_rx

    The current scheduler context is stopped until the transceiver RX
    has stopped. */
config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rfpacket_stop_rx(struct dev_rfpacket_wait_ctx_s *ctx);

/** @This transmit a packet. If the transceiver is currently in RX
    state, the RX will resume after transmitting the packet.  When the
    @tt timeout parameter is not 0, a fair TX is used.

    The current scheduler context is stopped until the transceiver
    has transmitted the packet. */
config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
error_t dev_rfpacket_wait_tx(struct dev_rfpacket_wait_ctx_s *ctx,
                             const uint8_t *data, size_t size,
                             uint_fast16_t channel, dev_rfpacket_pwr_t pwr,
                             dev_timer_delay_t timeout);

/** @This release resource used by the blocking rfpacket context. */
config_depend_and2(CONFIG_DEVICE_RFPACKET, CONFIG_MUTEK_CONTEXT_SCHED)
void dev_rfpacket_wait_cleanup(struct dev_rfpacket_wait_ctx_s *ctx);

#endif

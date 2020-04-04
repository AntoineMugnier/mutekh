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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2016
*/

/**
   @file
   @module {Core::Devices support library}
   @short USB device controller driver API
   @index {USB device} {Device classes}
   @csee DRIVER_CLASS_USBDEV
*/

#ifndef __USBDEV_H__
#define __USBDEV_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/enum.h>

#include <device/driver.h>
#include <mutek/kroutine.h>
#include <device/request.h>
#include <device/usb/usb.h>

#include <gct/container_clist.h>

struct usb_ctrl_setup_s;

/* This informs stack on USB event */

ENUM_DESCRIPTOR(dev_usbdev_event_e, strip:USBDEV_EVENT_, upper, or);

enum dev_usbdev_event_e
{
  /** Device Attached */
  USBDEV_EVENT_CONNECT     = 1,
  /* Device dettached from a host */
  USBDEV_EVENT_DISCONNECT  = 2,
  /** Reset on bus */
  USBDEV_EVENT_RESET       = 4,
  /** No bus traffic for more than 3 ms */
  USBDEV_EVENT_IDLE        = 8,
  /** Wake up when suspended */
  USBDEV_EVENT_WAKEUP      = 16,
  /** Device stopped */
  USBDEV_EVENT_STOP        = 32
};

/** Endpoint maps */
typedef uint16_t dev_usbdev_ep_map_t;
/** Endpoint address */
typedef uint8_t dev_usbdev_ep_addr_t;
/** Endpoint configuration revision number */
typedef int8_t dev_usbdev_cfgrev_t;

ENUM_DESCRIPTOR(dev_usbdev_rq_type_e, strip:DEV_USBDEV_, upper);

enum dev_usbdev_rq_type_e
{
  /** No transfer. Get event on USB device controller. The @ref
      dev_usbdev_rq_s::event field of the request is used to return
      one or more events of type @ref enum dev_usbdev_event_e */
  DEV_USBDEV_EVENT,
  /** Get SETUP stage packet from host. The request returns only when a setup
      packet has been entirely received. This is only used with CONTROL
      endpoints */
  DEV_USBDEV_CTRL_SETUP,
  /** Send data to host. The request is terminated only when the whole data
      buffer is sent. This can be achieved in multiple DATA IN transactions
      on the USB bus. When used in conjunction with the
      dev_usbdev_rq_s::size field set to 0, a Zero Length Packet is
      generated. */
  DEV_USBDEV_DATA_IN,
  /** Send data to host. The request does not terminate until at least one
      byte of data has been sent to the host. When used in conjunction with the
      dev_usbdev_rq_s::size field set to 0, a Zero Length Packet is
      generated. */
  DEV_USBDEV_PARTIAL_DATA_IN,
  /** Send a STALL PID in response to the next DATA IN stage transaction. */
  DEV_USBDEV_DATA_IN_STALL,
  /** Get data from host. The request is terminated when either the
      @ref dev_usbdev_rq_s::buffer is full or when a short packet is
      received. For CONTROL endpoint, the request is also terminated when
      @ref the dev_usbdev_rq_s::rem field is null which means that the
      CONTROL data stage is terminated.*/
  DEV_USBDEV_DATA_OUT,
  /** Get data from host. The request does not terminated until at least one
      byte has been received from host. */
  DEV_USBDEV_PARTIAL_DATA_OUT,
  /** Send a STALL PID in response to the next DATA OUT stage transaction. */
  DEV_USBDEV_DATA_OUT_STALL,
  /** Send a Zero Length Packet in STATUS IN stage. This is only used with
      CONTROL endpoints */
  DEV_USBDEV_CTRL_STATUS_IN,
  /** Send a STALL PID in STATUS IN stage. This is only used with CONTROL
      endpoints */
  DEV_USBDEV_CTRL_STATUS_IN_STALL,
  /** Send a ACK PID in STATUS OUT stage. This is only used with CONTROL
      endpoints */
  DEV_USBDEV_CTRL_STATUS_OUT,
  /** Send a STALL PID in STATUS OUT stage. This is only used with CONTROL
      endpoints */
  DEV_USBDEV_CTRL_STATUS_OUT_STALL,
};

struct device_usbdev_s;

/** @This is the USB device request structure. From an USB point of view and
    except for @ref DEV_USBDEV_EVENT, requests are used as response to USB
    transactions. This type of request are appropriated to device controller
    exposing a transaction level interface.
 */
struct dev_usbdev_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** Size of the @tt data buffer. For receiving data, size must be a multiple
      of the max packet size of the endpoint. */
  uint32_t                      size;
  /** Used for control transfer data stage only. Indicate the remaining data to
      send or received. This migth be modified by driver. Driver can use this
      field to know when data stage is terminated  */
  uint16_t                      rem;

  /** Endpoint revision. This field is used by USB device stack and must not be
      modified by driver */
  dev_usbdev_cfgrev_t           rev;
  /** Endpoint number. Endpoint number 0 is the default CONTROL endpoint.
      Others value are endpoint number sent to host in configuration descriptor*/
  uint8_t                       ep:4;
  /** Request type. Determines also the transaction data transfer direction */
  enum dev_usbdev_rq_type_e     type:8;

  /** Buffer for transmission and reception of data. Most of USB controller
      embedding an internal DMA constrain size and alignement of source and
      destination buffer. Thus buffer must be allocated and freed with the
      @ref dev_usbdev_alloc_t and @ref dev_usbdev_free_t functions. Value of
      data is undefined when a request is terminated. Buffer is only used for
      setup and data stage. */
  void                          *data;
  /** For endpoint 0 and only for this endpoint, when used in conjunction with
      the @tt error field set to -EIO, this field contains one or more USB events
      that occured on controller.*/
  uint8_t                       event;
};

DEV_REQUEST_INHERIT(usbdev); DEV_REQUEST_QUEUE_OPS(usbdev);

struct dev_usbdev_context_s;

#define DEV_USBDEV_REQUEST(n)	error_t (n) (struct dev_usbdev_context_s *ctx, struct dev_usbdev_rq_s *tr)

/**
  USB device class request() function type. Post a request on an USB device
  driver. There can be only one single request active at the same time on an
  endpoint.

  When a request is posted on driver, this function must return 0 if request
  processing is done immediately. If the request is processed later or if an
  interrupt must be waited to terminate the request, this function must return
  -EAGAIN. In this case, when the request will be terminated, the driver will
  have to call the @ref usbdev_stack_request_done function.

  A USB CONTROL transfer involves multiple stages and multipe transactions.
  Up to 3 stages are necessary. The first stage will retrieve setup packet.
  If there is a data stage, the next stage will retrieve or send data packet(s)
  depending on direction of control transfer. The last stage is the status stage.

   No data stage:
@code R
SETUP | STATUS IN

SETUP | STATUS IN STALL
@end code
   With DATA IN stage
@code R
SETUP | DATA IN | DATA IN | ... | STATUS OUT

SETUP | DATA IN | DATA IN | ... | STATUS OUT STALL

SETUP | DATA IN | ... | DATA IN STALL
@end code

   With DATA OUT stage
@code R
SETUP | DATA OUT | DATA OUT | ... | STATUS IN

SETUP | DATA OUT | DATA OUT | ... | STATUS IN STALL

SETUP | DATA OUT | ... | DATA OUT STALL
@end code

  All other transfer types involve only one DATA stage consisting of one or more
  USB transactions. For ISOCHRONOUS transfers the @ref DEV_USBDEV_DATA_OUT_STALL
  and @ref DEV_USBDEV_DATA_IN_STALL can not be used.

  For endpoint 0 and only for this endpoint, each on-going transfer can be
  terminated prematurely if an event is detected on bus or if the device is
  stopped. In this case, request is terminated with -EIO error. It is the
  responsability of the driver to end properly the on-going transfer on endpoint
  0 when a event occurs so that any new transfer on this endpoint can be processed
  normally.

  If a transfer is running on an endpoint and this endpoint is unconfigured due
  to a switch to an interface alternate configuration, the @ref
  usbdev_stack_request_done is called and @ref dev_usbdev_rq_s::error field is
  set to -EAGAIN.

  Each control endpoint must always have a active request for incoming setup
  packet. If no SETUP request is active when a SETUP packet arrives, it is the
  responsability of the driver to keep the packet until the next SETUP request.
  At device start-up, a request is pushed on control endpoint 0 to retrieve initial
  setup packet from host.

  For CONTROL DATA stage, BULK and INTERRUPT transactions, if there is no active
  request on the corresponding endpoint, the driver is in charge of sending the
  NAK token in response to the DATA stage transaction from host.

  For isochronous endpoints, if a packet has been dropped because no request
  has been posted in time, @ref dev_usbdev_rq_s::error field will be set to
  @tt -ETIMEDOUT on the next request posted on this endpoint.

  @list
    @item @tt -EAGAIN: If returned when posting request, this means that request
    will terminate later. If returned with @ref usbdev_stack_request_done for
    endpoints other than 0, this means that endpoint has been deconfigured and
    transfer aborted.
    @item @tt -EIO: Only for endpoint 0. A event has occured on device.
    @item @tt -ETIMEDOUT: Returned with @ref usbdev_stack_request_done only. A
    packet has been dropped on an isochronous endpoint.
    @item @tt -EPIPE: Returned with @ref usbdev_stack_request_done only. USB
    connection has been lost and transfer is aborted.
  @end list
*/

typedef DEV_USBDEV_REQUEST(dev_usbdev_request_t);

config_depend(CONFIG_DEVICE_USBDEV)
void usbdev_stack_request_done(struct dev_usbdev_context_s *ctx, struct dev_usbdev_rq_s *tr);

enum dev_usbdev_cfg_e
{
  /** This is used either to configure endpoint 0 or when a SET CONFIGURATION
      command from host is received. Because some USB controllers have an
      internal RAM that must be allocated and partionned by driver for each
      endpoint, changing dynamically the size of an endpoint due to a @ref
      DEV_USBDEV_CHANGE_INTERFACE command could lead to a complete reallocation
      of the whole memory and thus prevent other interfaces from working. If one
      or more alternate setting are used for an interface, it migth be
      attractive to known in advance these different settings to prevent a
      whole reallocation of the memory. That is why, the driver have access to
      all aternate settings of each interface during this operation. Parsing
      all settings of an interface and all the corresponding endpoints can be
      facilitated by @ref #USBDEV_FOREACH_INTERFACE and @ref #USBDEV_FOREACH_ENDPOINT
      macros */
  DEV_USBDEV_CONFIGURE            = 1,
  /** Unconfigure all endpoint except endpoint 0. Unconfigure operation is used
      when a set configuration or an event happen on the USB device. All controller
      endpoints except endpoint zero must be unconfigured in this case.
      When unconfiguring controller, all on-going transfer must be terminated with
      @ref dev_usbdev_config_s::error field set to @tt -EPIPE. @ref
      dev_usbdev_rq_s::size field of on-going transfer might not be updated
      in this case. */
  DEV_USBDEV_UNCONFIGURE          = 2,
  /** Set controller address. This is used when a SET ADDRESS command from host
      is received. The usb device must not change its device address until after
      the status stage of this control transfer is completed successfully. However
      some controller need to configure the new address before sending the status
      stage. For this reason, this is called before sending status stage and it is
      the responsability of the driver to delay or not to the end of status stage
      the address configuration.*/
  DEV_USBDEV_SET_ADDRESS          = 4,
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  /** Change configuration of an interface. This is used when SET INTERFACE
      command from host is received. This occurs when device has shown
      alternate settings for an interface during enumeration. The driver must
      unconfigure each endpoint that are not used anymore. This must be done
      neatly since no data of on-going transfer must be lost except on
      isochronous endpoints. The @ref dev_usbdev_rq_s::size field of any
      on-going request must be updated an the request must be terminated with
      @tt -EAGAIN error code.
      At this point, the new interface configuration can be set by the driver.
      During these two operations other interfaces must go on working
      normally.*/
  DEV_USBDEV_CHANGE_INTERFACE     = 8,
#endif
};

struct usbdev_interface_s;

enum dev_usbdev_interface_e
{
  DEV_USBDEV_INTF_ENABLE      = 0,
  DEV_USBDEV_INTF_DISABLE     = 1,
};

struct dev_usbdev_interface_cfg_s
{
  /* Interface information */
  const struct usbdev_interface_s *i;
  /* Endpoint map. This contains correspondance between a local endpoint
     address and its global address as provided to host during enumeration.
     All endpoint address provided with @ref dev_usbdev_config_t are local and
     must be translated by driver to global address. This can be achieved with
     the @ref usbdev_stack_get_ep_addr function */
  dev_usbdev_ep_map_t  *epi;
  dev_usbdev_ep_map_t  *epo;
};

struct dev_usbdev_config_s
{
  error_t                      error;

  enum dev_usbdev_cfg_e        type;

  union
    {
    /** This is a list of interface. When used in conjunction with the @ref
        DEV_USBDEV_CHANGE_INTERFACE, this list contains two entries. One
        with information about interface to disable and the other one with
        information about interface to enable.
        When used in conjunction with the @ref DEV_USBDEV_CONFIGURE, this list
        of interface is NULL terminated and contains all interfaces of the
        configuration that must be enable. When NULL, endpoint 0 must be
        configured. */
      struct dev_usbdev_interface_cfg_s *intf;
    /* Address to set */
      uint16_t  addr;
    };
};

#define DEV_USBDEV_CONFIG(n) error_t (n) (struct dev_usbdev_context_s *ctx, struct dev_usbdev_config_s *cfg)

/**
  USB device class configure() function type. Perform several configuration
  on USB device controller.

  When a configuration is posted on driver, if its processing is done immediately
  without differed interrupt, this function must return 0. If the configuration is
  processed later or if an interrupt must be waited to terminate the operation, this
  function must return -EAGAIN. In this case, when the configuration is terminated,
  the driver will have to call the  @ref usbdev_stack_config_done function. If an
  error is detected when posting a configuration, this function must return the
  associated error code.

  If an event occurs during a configuration operation, it should not be stopped
  or cancelled and should be terminated normaly if possible. In any case, @ref
  usbdev_stack_config_done must be called without error. Event will be reported
  later on the next request on endpoint 0.

  If an configuration is not supported by a controller @tt -ENOTSUP error must
  be returned.
**/
typedef DEV_USBDEV_CONFIG(dev_usbdev_config_t);

config_depend(CONFIG_DEVICE_USBDEV)
void usbdev_stack_config_done(struct dev_usbdev_context_s *ctx);

#define DEV_USBDEV_ALLOC(n) void * (n)(struct dev_usbdev_context_s *ctx, size_t size)

/**
   USB device class allocate() function type. Allocate a buffer
   compliant with USB device controller. Most of USB device controller
   need special alignement or minimum size for receive or send
   buffer. Returns 0 when the required allocation failed otherwise
   returns a pointer on the buffer. Allocated size must always be
   multiple of the Maximum Packet Size of the endpoint for which the
   buffer is allocated.
*/
typedef DEV_USBDEV_ALLOC(dev_usbdev_alloc_t);

#define DEV_USBDEV_FREE(n) void (n)(struct dev_usbdev_context_s *ctx, void * ptr)

/**
   @This frees an allocated buffer.
*/
typedef DEV_USBDEV_FREE(dev_usbdev_free_t);


struct usbdev_endpoint_s
{
  dev_request_queue_root_t queue;
  dev_usbdev_cfgrev_t rev;
  bool_t busy:1;
  bool_t halted:1;
  bool_t disabled:1;
};

#define DEV_USBDEV_ENDPOINT(n) struct usbdev_endpoint_s * (n)(struct dev_usbdev_context_s *ctx, enum usb_endpoint_dir_e dir, uint8_t address)

/**
   USB device class endpoint() function type. Return endpoint
   structure of a specified endpoint allocated in the USB controller
   device private data. Each USB device controller specifies a number
   of endpoint that it is able to support in addition to endpoint
   0. For each of these additional endpoints, it must allocate a @ref
   struct usbdev_endpoint_s structure. Endpoint 0 must not be
   allocated by driver. Return NULL if the requested endpoint does not
   exist.
 */
typedef DEV_USBDEV_ENDPOINT(dev_usbdev_endpoint_t);

DRIVER_CTX_CLASS_TYPES(DRIVER_CLASS_USBDEV, usbdev, );

# define DRIVER_USBDEV_METHODS(prefix)                   \
  ((const struct driver_class_s*)&(const struct driver_usbdev_s){                      \
    .class_ = DRIVER_CLASS_USBDEV,                       \
    .ctx_offset = offsetof(driver_pv_t , usbdev_ctx),                 \
  })

# define USBDEV_FOREACH_INTERFACE(intf, ... /* loop body */ )            \
  do {                                                                  \
    const struct usbdev_interface_default_s * _i                        \
      = (const struct usbdev_interface_default_s *)intf;                 \
    uint_fast8_t _intfidx = 0;                                           \
    while(1)                                                            \
      {                                                                 \
        { __VA_ARGS__ }                                                 \
        if (_intfidx == _i->alt_cnt)                                     \
          goto _end;                                                    \
        intf = (const struct usbdev_interface_s *)_i->alt[_intfidx++];    \
      }                                                                 \
  _end:;                                                                \
  } while(0)

# define USBDEV_FOREACH_ENDPOINT(intf, mapin, mapout, ... /* loop body */)         \
  do {                                                                            \
    const struct usbdev_interface_s *_intf = (intf);                                \
    const dev_usbdev_ep_map_t *_mapin = (mapin);                                 \
    const dev_usbdev_ep_map_t *_mapout = (mapout);                               \
    const struct usb_interface_descriptor_s *_d = &(_intf->desc);                  \
    for (uint_fast8_t _epidx = 0; _epidx < usb_interface_ep_count_get(_d) ; _epidx++)     \
      {                                                                           \
        uint8_t _idx = usb_interface_alt_get(_d);                                       \
        const struct usb_endpoint_descriptor_s *epdesc = _intf->ep[_epidx];       \
        dev_usbdev_ep_addr_t epaddr                                              \
          = usbdev_stack_get_ep_addr(epdesc, _mapin[_idx], _mapout[_idx]);       \
        { __VA_ARGS__ }                                                           \
      }                                                                           \
  } while(0)

/* @This is a description of the USB device */
struct usbdev_device_info_s
{
  /* Device descriptor */
  struct usb_device_descriptor_s desc;

  /* Configuration descriptor information */
  uint8_t  configuration;
  uint8_t  power;
  uint8_t  iconfig;

  /* String containing all device descriptor */
  const char * string;
  size_t str_cnt;
};

struct dev_usbdev_driver_ops_s
{
  dev_usbdev_request_t  *f_transfer;
  dev_usbdev_config_t   *f_config;
  dev_usbdev_alloc_t    *f_alloc;
  dev_usbdev_free_t     *f_free;
  dev_usbdev_endpoint_t *f_endpoint;
};

/** @This initializes an instance of the @ref dev_usbdev_context_s structure.
    @tt epi_msk and @tt epo_msk are two masks of supported IN and OUT endpoints
    including endpoint 0. */
config_depend(CONFIG_DEVICE_USBDEV)
error_t usbdev_stack_init(struct device_s *dev,
                          struct dev_usbdev_context_s *ctx,
                          uint16_t epi_msk, uint16_t epo_msk,
                          const struct dev_usbdev_driver_ops_s *ops);

config_depend(CONFIG_DEVICE_USBDEV)
error_t usbdev_stack_cleanup(struct dev_usbdev_context_s *ctx);

struct usbdev_interface_s
{
  /* Current configuration */
  const struct usb_interface_descriptor_s desc;
  /* Table of endpoint descriptor */
  const struct usb_endpoint_descriptor_s *const* ep;
};

struct usbdev_interface_default_s
{
  struct usbdev_interface_s intf;
  /* Table of interface alternate setting */
  const struct usbdev_interface_s *const* alt;
  /* Number of alternate setting */
  uint8_t alt_cnt;
};

struct dev_usbdev_desc_iterator_s;
struct usbdev_service_s;

/** @This is provided by service with some class specific
    descriptors. This function is used by stack when building global
    configuration descriptor to dynamically replace some fields of
    descriptor.

    Buffer passed in @tt dst contains a copy of @tt src, and @this
    should patch bytes between @tt begin (included) and @tt end
    (excluded) only.  Others may be invalid.

    @tt src is the original constant descriptor, its address is the
    one from the descriptor array in @ref
    {usbdev_service_descriptor_s} {service descriptor}.
*/
#define USBDEV_REPLACE(n) void (n)(           \
  const struct dev_usbdev_desc_iterator_s *it,\
  const struct usb_descriptor_header_s *src,  \
  struct usb_descriptor_header_s *dst,        \
  size_t begin, size_t end)

typedef USBDEV_REPLACE(usbdev_replace_t);

/** @This is provided by service to dynamically get strings.
*/
#define USBDEV_GET_STRING(n) const char * (n)(   \
  const struct usbdev_service_s *service,        \
  uint8_t idx)

typedef USBDEV_GET_STRING(usbdev_get_string_t);

USBDEV_REPLACE(usbdev_descriptor_replace);
USBDEV_REPLACE(usbdev_cdc_descriptor_replace);

struct usbdev_service_descriptor_s
{
  /* Table of descriptor */
  const struct usb_descriptor_header_s *const* desc;
  /* Size of previous table */
  uint8_t desc_cnt;
  /* String describing service */
  const char *string;
  /* Number of string descriptor in string */
  uint8_t str_cnt;
  /* Table of interfaces */
  const struct usbdev_interface_default_s *const* intf;
  /* Number of interface used by service. */
  uint8_t intf_cnt;
  /** Service dynamic patcher function */
  usbdev_replace_t *replace;
  /** Service dynamic string getter */
  usbdev_get_string_t *get_string;
};

/* Container algorithm used for service */
#define GCT_CONTAINER_ALGO_usbdev_service CLIST

/** @This is type of request sent by a service on USB device stack */
enum usbdev_service_req_type_e
{
/** @This is used in response to a @ref USBDEV_PROCESS_CONTROL command from
    stack. A service call the @ref usbdev_stack_request with this value to send
    or receive DATA stage of a CONTROL transfer. Direction of transfer is
    defined by setup packet. If the number of byte to send/receive is greater
    than buffer size provided by stack, the service will have to perfom multiple
    data transfers to/from the stack. */
  USBDEV_TRANSFER_DATA     = 1,
/** @This is used by service to retrieve the next command from stack. At
    start-up, stack is waiting for all services to post this type of request
    before performing any operation. When processing a control transfer, a
    service must use this value to inform stack that DATA stage of control
    transfer is terminated. If usbdev_service_rq_s::error is set to -EINVAL,
    a STALL pid will be generated either in DATA or STATUS stage of CONTROL
    transfer depending of the remaining data to send/receive to/from host. */
  USBDEV_GET_COMMAND       = 2,
};

enum usbdev_service_cmd_type_e
{
/** @This is used by stack when a class-specific request is received for a
    service. the usbdev_service_rq_s::ctrl::setup field is the 8 byte SETUP
    packet to process. When a service needs to send/receive data for
    processing setup, it must use the provided @ref usbdev_service_rq_s::buffer
    data buffer. The @ref usbdev_service_rq_s::size is the size of the provided
    buffer. When the request is not handled by service or when there is a error
    in request @tt -EINVAL is returned and a request error wil be generated to
    host. */
  USBDEV_PROCESS_CONTROL   = 1,
/** @This is used to disable a service. This function is called when a reset or
    a disconnect occurs on USB bus. It is also called when device is stoppped
    or when a SET CONFIGURATION command with a configuration that dos not include
    the service is sent by host. If endpoint transfers are pending when this
    kroutine is called, they will terminate later with @tt -EPIPE error. A
    service must not push any endpoint transfer after this. */
  USBDEV_DISABLE_SERVICE   = 2,
/** @This is used to enable a service after a set configuration occured on device.
    After this, service is authorized to transfer data on its endpoints. */
  USBDEV_ENABLE_SERVICE    = 3,
/** @This is used to inform a service that one of its interfaces has been
    reconfigured on usb controller due to a SET INTERFACE command on usb device.
    All pending endpoint transfers that do not use the new configuration ends with
    @tt -EAGAIN error. The usbdev_service_rq_s::intf is the local target interface.
    The usbdev_service_rq_s::alternate is the alternate seting number. */
  USBDEV_CHANGE_INTERFACE  = 4,
};


/** @This is used by service with @ref usbdev_stack_request to retrieve control
    commands from stack or to transfer data on endpoint 0. At initialisation USB
    stack is waiting for all registered services to post this request. */
struct usbdev_service_rq_s
{
  /** The @ref kroutine_exec function is called on this kroutine when
      a request ends. This kroutine should not be immediate. If this kroutine
      is immediate, a new request must not be pushed to the stack in this
      kroutine */
  struct kroutine_s               kr;
  /** Request type. See @ref usbdev_service_req_type_e */
  enum usbdev_service_req_type_e  type;
  /** Request command. See @ref usbdev_service_cmd_type_e */
  enum usbdev_service_cmd_type_e  cmd;
  /** Callback private data */
  void                            *pvdata;

  error_t                         error;

  uint8_t                         intf;

  union
    {
      uint8_t alternate;
      /** @This is set by stack when a SETUP token must be processed by a
          service. @tt bufer must be used for data stage if present. @tt size
          is the size of buffer. If data stage implies more bytes than buffer
          size, multiple transfers must be done. */
      struct
        {
          struct usb_ctrl_setup_s *setup;
          uint8_t                 *buffer;
          size_t                  size;
        } ctrl;
    };
};

struct usbdev_service_index_s
{
  /* Endpoint map */
  dev_usbdev_ep_map_t epi[1 + CONFIG_USBDEV_MAX_ALTERNATE_COUNT];
  dev_usbdev_ep_map_t epo[1 + CONFIG_USBDEV_MAX_ALTERNATE_COUNT];
  /* Service id */
  uint8_t id;
  /* Start index for interface */
  uint8_t intf;
  /* Start index for string */
  uint8_t str;
};

/** @This must be allocated by each service and must be attached to stack by @ref
    usbdev_stack_service_register at service initialisation. */
struct usbdev_service_s
{
  GCT_CONTAINER_ENTRY(usbdev_service, entry);
  /* Pointer to constant service descriptor */
  const struct usbdev_service_descriptor_s *desc;
  /* Start index */
  struct usbdev_service_index_s start;
  /* Control request for endpoint 0 */
  struct usbdev_service_rq_s *rq;
  /* Pointer to service private data */
  void *pv;
};

/** @This is called by services to retrieve a control command or to transfer
    data on control endpoint 0. */
config_depend(CONFIG_DEVICE_USBDEV)
error_t usbdev_stack_request(const struct device_usbdev_s *dev, struct usbdev_service_s *service,
                             struct usbdev_service_rq_s *rq);

/** @This is called by services to transfer data on control endpoint other than
    endpoint 0.

    When a SET INTERFACE command from host change the setting of an interface, all
    the associated endpoints have their revision number changed. If the @ref
    dev_usbdev_rq_s::rev field of request is different from the current
    endpoint revision, @tt -EAGAIN is returned either by this function or by
    kroutine.
    When a service post a request while it should not, @tt -EPIPE error is returned
    either by this function or by kroutine. In this case, the
    @ref dev_usbdev_rq_s::size might not be updated. */
config_depend(CONFIG_DEVICE_USBDEV)
error_t usbdev_stack_transfer(const struct device_usbdev_s *dev,
                              struct usbdev_service_s *service,
                              struct dev_usbdev_rq_s *tr,
                              const struct usb_endpoint_descriptor_s *desc);

#define USBDEV_SERVICE_DESCRIPTOR(...)   \
   .desc = ((const struct usb_descriptor_header_s *[]){__VA_ARGS__}),  \
   .desc_cnt = sizeof((const struct usb_descriptor_header_s *[]){__VA_ARGS__})/sizeof((const struct usb_descriptor_header_s *[]){__VA_ARGS__}[0])

#define USBDEV_INTERFACE(...)   \
   .intf = ((const struct usbdev_interface_default_s *[]){__VA_ARGS__}),  \
   .intf_cnt = sizeof((const struct usbdev_interface_default_s *[]){__VA_ARGS__})/sizeof((const struct usbdev_interface_default_s *[]){__VA_ARGS__}[0])

#define USBDEV_INTERFACE_ALTERNATE(...)   \
   .alt = ((const struct usbdev_interface_s *[]){__VA_ARGS__}),  \
   .alt_cnt = sizeof((const struct usbdev_interface_s *[]){__VA_ARGS__})/sizeof((const struct usbdev_interface_s *[]){__VA_ARGS__}[0])

#define USBDEV_ENDPOINT(...)   \
   .ep = ((const struct usb_endpoint_descriptor_s *[]){__VA_ARGS__})

GCT_CONTAINER_TYPES(usbdev_service, struct usbdev_service_s *, entry);
GCT_CONTAINER_FCNS(usbdev_service, inline, usbdev_service,
                   init, destroy, push, remove, head, next);

/** @internal */
struct dev_usbdev_desc_iterator_s
{
  /* Current service */
  struct usbdev_service_s * service;
  /* Current byte in descriptor */
  size_t desc_offset:16;
  /* Number of bytes sent */
  size_t data_pkt_size:16;
  /* Current descriptor in service */
  size_t desc_index:8;
  /* Current interface in service */
  size_t intf_index:8;
  /* Transfer done */
  bool_t done;
};

ENUM_DESCRIPTOR(dev_usbdev_ctrl_ep_state_e, strip:EP0_, upper);

/** @internal */
enum dev_usbdev_ctrl_ep_state_e
{
  EP0_IDLE,
  EP0_SETUP_WAIT,
  EP0_PROCESS_CLASS,
  EP0_PROCESS_STANDARD,
  EP0_CTRL_SET_ADDRESS_WAIT,
  EP0_SET_INTERFACE,
  EP0_SET_INTERFACE_WAIT,
  EP0_CTRL_SET_INTERFACE_WAIT,
  EP0_SRVC_SET_INTERFACE_WAIT,
  EP0_SET_INTERFACE_END_TRANSFER,
  EP0_CTRL_CONFIGURATION_WAIT,
  EP0_SRVC_ENABLE_WAIT,
  EP0_CTRL_UNCONFIGURATION_WAIT,
  EP0_SRVC_DISABLE_WAIT,
  EP0_SRVC_DATA_IN,
  EP0_SRVC_DATA_IN_WAIT,
  EP0_DATA_IN,
  EP0_DATA_IN_ZERO_WAIT,
  EP0_DATA_IN_WAIT,
  EP0_DATA_IN_ZERO,
  EP0_SRVC_DATA_OUT,
  EP0_SRVC_DATA_OUT_WAIT,
  EP0_DATA_OUT,
  EP0_DATA_OUT_WAIT,
  EP0_STATUS_IN,
  EP0_SRVC_STATUS_IN,
  EP0_STATUS_OUT,
  EP0_STATUS_WAIT,
  EP0_STALL_WAIT,
};

/** @This stucture hold the usb device stack context associated to an
    usb device controller. USB device controller driver usually
    allocate this structure in the device private context and perform
    its initialization by calling the @ref dev_usbdev_context_init
    function. */
struct dev_usbdev_context_s
{
  const struct dev_usbdev_driver_ops_s *ops;

  /** Accessor on usb device controller */
  struct device_s *dev;

  /* USB device information */
  const struct usbdev_device_info_s * devinfo;
  /* Iterator used for enumeration */
  struct dev_usbdev_desc_iterator_s it;

  /** Device state */
  enum dev_usbdev_state_e state;

  /** Number of services registerd on USB device */
  uint8_t service_cnt;
  uint32_t service_state;

  /** List of usb services */
  usbdev_service_root_t service;
  struct kroutine_s kr;

  /** Endpoint 0 setup packet */
  uint32_t setup[2];
  /** Endpoint 0 state */
  enum dev_usbdev_ctrl_ep_state_e ep0_state:8;

  union {
    /** Configuration request for USB driver */
    struct dev_usbdev_config_s cfg;
    /** Endpoint 0 transfer */
    struct dev_usbdev_rq_s tr;
  };

  /** Endpoint buffer */
  uint8_t *data;
  bool_t halted;

  /** Controller endpoint mask */
  uint16_t epi_msk;
  uint16_t epo_msk;

  /** Pending Bus event */
  uint8_t event;
#if (CONFIG_USBDEV_MAX_ALTERNATE_COUNT > 0)
  uint8_t intf_cfg[CONFIG_USBDEV_MAX_INTERFACE_COUNT];
#endif

  struct dev_usbdev_interface_cfg_s intf[CONFIG_USBDEV_MAX_INTERFACE_COUNT];
};

STRUCT_COMPOSE(dev_usbdev_context_s, kr);



/** @This attach an USB service to an USB device controller. This is called
    either by a service that wants to be registered or by a main application
    implementing its own service. @This can only be called when device is
    stopped. Otherwise it returns -EBUSY. */
config_depend(CONFIG_DEVICE_USBDEV)
error_t usbdev_stack_service_register(const struct device_usbdev_s *dev,
                                      struct usbdev_service_s *service);

/** @This detach an USB service from an USB device controller. This is called
    either by a service that wants to be unregistered or by a main application
    implementing its own service. @This can only called when device is stopped.
    Otherwise it returns -EBUSY */
config_depend(CONFIG_DEVICE_USBDEV)
error_t usbdev_stack_service_unregister(const struct device_usbdev_s *dev,
                                        struct usbdev_service_s *service);

/** @This allocates a buffer compliant with USB device controller. Most
    of USB device controller need special alignement or minimum size for receive
    or send buffer. Returns 0 when the required allocation failed otherwise
    returns a pointer on the buffer. */
config_depend(CONFIG_DEVICE_USBDEV)
void * usbdev_stack_allocate(const struct device_usbdev_s *dev, size_t size);

/** @This frees an allocated buffer. */
config_depend(CONFIG_DEVICE_USBDEV)
void usbdev_stack_free(const struct device_usbdev_s *dev, void * ptr);

/* @This attach an USB device description to an USB device controller.*/
config_depend(CONFIG_DEVICE_USBDEV)
void usbdev_stack_set_device_info(const struct device_usbdev_s *dev,
                                  const struct usbdev_device_info_s *info);

/* @This returns endpoint 0 max packet size */
config_depend_alwaysinline(CONFIG_DEVICE_USBDEV,
uint8_t usbdev_stack_get_ep0_mps(struct dev_usbdev_context_s *ctx),
{
  return ctx->devinfo->desc.bMaxPacketSize0;
});

/* @This returns a global endpoint address from an endpoint descriptor. */
config_depend(CONFIG_DEVICE_USBDEV)
uint8_t usbdev_stack_get_ep_addr(const struct usb_endpoint_descriptor_s *desc,
                                 dev_usbdev_ep_map_t mapi,
                                 dev_usbdev_ep_map_t mapo);


config_depend(CONFIG_DEVICE_USBDEV)
enum usb_transfert_direction_e
dev_usbdev_get_transfer_dir(struct dev_usbdev_rq_s *tr);

config_depend(CONFIG_DEVICE_USBDEV)
error_t dev_res_get_usbdev_epmap(struct device_s *dev,
                                 struct usbdev_service_s * service);

#ifdef CONFIG_DEVICE_USBDEV
/** @This specifies a USBDEV resource entry in a static
    device resources table declaration.
    The @tt mapin and @tt mapout parameters map local input and output
    endpoints of a service to endpoints of USB controller.

    A service might owns one or several interfaces with alternate settings.
    Endpoint mapping can be different for each of this setting. The @tt config
    specifies the altenate setting number associated to the @tt mapin and @tt
    mapout parameters. When no alternate setting is used in a service, the @tt
    config parameter is set to 0.
*/
# define DEV_STATIC_RES_USBDEV_EP_MAP(config_, mapin_, mapout_)  \
  {                                                       \
      .type = DEV_RES_USBDEV,                             \
        .u = { .usbdev = {                                \
          .config = (config_),                            \
          .mapin = (mapin_),                              \
          .mapout = (mapout_),                            \
        } }                                               \
  }
#else
# define DEV_STATIC_RES_USBDEV_EP_MAP(config_, mapin_, mapout_)  \
  {                                                      \
    .type = DEV_RES_UNUSED,                              \
  }
#endif

#endif

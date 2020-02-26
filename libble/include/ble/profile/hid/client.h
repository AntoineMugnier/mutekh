#ifndef HID_CLIENT_H_
#define HID_CLIENT_H_

#include <net/layer.h>
#include <enums.h>

#define HID_CLIENT_CHAR_MAX 16

struct net_layer_delegate_vtable_s;
struct net_scheduler_s;
struct net_layer_s;

struct hid_report_s
{
  uint16_t handle;
  uint16_t cccd_handle;
  uint8_t report_id;
  uint8_t report_type;
};

struct hid_client_descriptor_s
{
  char vendor[20];
  char product[20];
  char serial[20];
  uint16_t vendor_id;
  uint16_t product_id;
  uint16_t hid_version;
  const uint8_t *descriptor;
  size_t descriptor_size;
  const struct hid_report_s *report;
  size_t report_count;
};

struct hid_client_delegate_vtable_s
{
  struct net_layer_delegate_vtable_s base;

  void (*descriptor_discovered)(void *delegate, struct net_layer_s *layer,
                                const struct hid_client_descriptor_s *desc);

  void (*discovery_error)(void *delegate, struct net_layer_s *layer);

  void (*should_encrypt)(void *delegate, struct net_layer_s *layer);

  void (*report_changed)(void *delegate, struct net_layer_s *layer,
                         uint8_t report_id,
                         const void *data, size_t size);

  void (*report_read_done)(void *delegate, struct net_layer_s *layer,
                           const void *data, size_t size);

  void (*report_read_error)(void *delegate, struct net_layer_s *layer);
};

STRUCT_COMPOSE(hid_client_delegate_vtable_s, base);

struct hid_client_handler_s
{
  struct net_layer_handler_s base;

  void (*descriptor_discover)(struct net_layer_s *layer);

  void (*descriptor_serialize)(struct net_layer_s *layer,
                               void *target, size_t size);

  void (*subscribe)(struct net_layer_s *layer);

  void (*report_set)(struct net_layer_s *layer,
                     uint8_t report_id,
                     const void *data, size_t size);

  void (*report_read)(struct net_layer_s *layer,
                     uint8_t report_id);
};

STRUCT_COMPOSE(hid_client_handler_s, base);

struct hid_client_params_s
{
  const void *descriptor;
  size_t descriptor_size;
};

error_t hid_client_create(struct net_scheduler_s *scheduler,
                              const void *params,
                              void *delegate,
                              const struct net_layer_delegate_vtable_s *delegate_vtable,
                              struct net_layer_s **layer);

ALWAYS_INLINE
void hid_client_descriptor_discover(struct net_layer_s *layer)
{
  const struct hid_client_handler_s *handler
      = const_hid_client_handler_s_from_base(layer->handler);
  handler->descriptor_discover(layer);
}

ALWAYS_INLINE
void hid_client_descriptor_serialize(struct net_layer_s *layer,
                               void *target, size_t size)
{
  const struct hid_client_handler_s *handler
      = const_hid_client_handler_s_from_base(layer->handler);
  handler->descriptor_serialize(layer, target, size);
}

ALWAYS_INLINE
void hid_client_subscribe(struct net_layer_s *layer)
{
  const struct hid_client_handler_s *handler
      = const_hid_client_handler_s_from_base(layer->handler);
  handler->subscribe(layer);
}

ALWAYS_INLINE
void hid_client_report_set(struct net_layer_s *layer,
                     uint8_t report_id,
                     const void *data, size_t size)
{
  const struct hid_client_handler_s *handler
      = const_hid_client_handler_s_from_base(layer->handler);
  handler->report_set(layer, report_id, data, size);
}

ALWAYS_INLINE
void hid_client_report_read(struct net_layer_s *layer,
                            uint8_t report_id)
{
  const struct hid_client_handler_s *handler
      = const_hid_client_handler_s_from_base(layer->handler);
  handler->report_read(layer, report_id);
}

#endif

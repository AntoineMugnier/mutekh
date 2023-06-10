#ifndef OAD_SERVICE_H_
#define OAD_SERVICE_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/mem.h>
#include <device/class/timer.h>
#include "ids.h"

struct oad_service_handler_s;

struct oad_service_s
{
  struct ble_gattdb_registry_s reg;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  const struct oad_service_handler_s *handler;

  dev_timer_delay_t delay;

  struct oad_info_s info;
  struct payload_status_s status;
  size_t storage_size;
  size_t payload_size;
};

struct oad_service_handler_s
{
  error_t (*start)(struct wpt_profile_s *pro,
                   uintptr_t expected_size);
  error_t (*chunk_write)(struct wpt_profile_s *pro,
                         uintptr_t offset,
                         const void *data,
                         size_t size);
  error_t (*commit)(struct wpt_profile_s *pro);
};

error_t oad_service_register(
  struct oad_service_s *oad,
  const struct oad_service_handler_s *handler,
  uint8_t update_per_sec,
  size_t storage_size,
  struct device_timer_s *timer,
  struct ble_gattdb_s *db);

void oad_service_cleanup(struct oad_service_s *oad);

#endif

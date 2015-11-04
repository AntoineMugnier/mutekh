#ifndef DIS_H_
#define DIS_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>

error_t dis_service_register(struct ble_gattdb_registry_s *reg,
                             struct ble_gattdb_s *db);

#endif

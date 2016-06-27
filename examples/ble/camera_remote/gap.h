#ifndef GAP_H_
#define GAP_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>

error_t gap_service_register(struct ble_gattdb_registry_s *dbs,
                             struct ble_gattdb_s *db);

#endif

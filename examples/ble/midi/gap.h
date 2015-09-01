#ifndef GAP_H_
#define GAP_H_

#include <ble/gatt/db.h>
#include <ble/gatt/service.h>

error_t gap_service_register(struct ble_gatt_db_service_s *dbs,
                             struct ble_gatt_db_s *db);

#endif

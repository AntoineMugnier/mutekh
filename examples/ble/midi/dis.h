#ifndef DIS_H_
#define DIS_H_

#include <ble/gatt/db.h>
#include <ble/gatt/service.h>

error_t dis_service_register(struct ble_gatt_db_service_s *dbs,
                             struct ble_gatt_db_s *db);

#endif

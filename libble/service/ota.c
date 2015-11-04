#include <ble/gatt/db.h>
#include <ble/gatt/service.h>
#include <ble/protocol/gatt/service.h>
#include <ble/protocol/gatt/characteristic.h>
#include <ble/protocol/gatt/descriptor.h>
#include <ble/gatt/service/ota.h>
#include <mutek/printk.h>

#include "ota.h"

/*
  Design:

  5 chars:

  * Upload parameters (static, read only)
    * Current version build date
    * Current version build index
    * Flash page size (log2 ?)
    * Flash free page count
    * Fragment size

  * Upload state (notification)
    * Upload state
      * Setup expected
      * Fragments expected
      * Commit expected
      * Rebooting
    * Next expected fragment number
    * Current CRC

  * Firmware update setup (write only, req)
    * Total fragment count
    * Key

  * Firmware payload fragment (write only, cmd)
    * Fragment number
    * Fragment data

  * firmware update commit (write only, req)
    * Expected CRC
    * reboot if ok
 */

static
uint8_t ota_on_actuator_subscribe(struct ble_gatt_db_service_s *service, uint8_t charid,
                                  bool_t subscribed)
{
    struct gamepad_otas_s *otas = gamepad_otas_s_from_reg(service);

    printk("Actuator subscribed: %d\n", subscribed);

    gamepad_actuators_enable(otas->gamepad, subscribed);

    return 0;
}

static
uint8_t ota_on_motion_subscribe(struct ble_gatt_db_service_s *service, uint8_t charid,
                                bool_t subscribed)
{
    struct gamepad_otas_s *otas = gamepad_otas_s_from_reg(service);

    (void)otas;

    printk("Motion subscribed: %d\n", subscribed);

    gamepad_sensors_enable(otas->gamepad, subscribed);

    return 0;
}

static
uint8_t ota_on_vibrator_changed(struct ble_gatt_client_s *client,
                                struct ble_gatt_db_service_s *service, uint8_t charid)
{
    struct gamepad_otas_s *otas = gamepad_otas_s_from_reg(service);

    printk("Vibrator write: %P\n", vibrator_report_data, sizeof(vibrator_report_data));

    gamepad_vib_run(otas->gamepad, 0, vibrator_report_data.duration0);
    gamepad_vib_run(otas->gamepad, 1, vibrator_report_data.duration1);

    return 0;
}

static
uint8_t ota_on_battery_subscribe(struct ble_gatt_db_service_s *service, uint8_t charid,
                                 bool_t subscribed)
{
    struct gamepad_otas_s *otas = gamepad_otas_s_from_reg(service);

    (void)otas;

    printk("Battery level subscribed: %d\n", subscribed);

    return 0;
}

enum ota_char_id
{
    CHAR_ID_REPORT_MAP,
    CHAR_ID_ACTUATOR,
    CHAR_ID_MOTION,
    CHAR_ID_VIBRATOR,
    CHAR_ID_BATTERY,
};

BLE_GATT_SERVICE_DECL(ota_service, 1, BLE_UUID_SHORT_P(BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE),
                      [CHAR_ID_REPORT_MAP] = BLE_GATT_CHAR_CONSTANT_BLOB(BLE_UUID_SHORT_P(BLE_UUID_REPORT_MAP_CHAR),
                                                                         report_map,
                                                                         sizeof(report_map)),

                      [CHAR_ID_ACTUATOR] = BLE_GATT_CHAR(BLE_UUID_SHORT_P(BLE_UUID_REPORT_CHAR),
                                                         BLE_GATT_NOTIFIABLE | BLE_GATT_PERM_ENC_READ,
                                                         BLE_GATT_CHAR_DATA_PLAIN(&actuator_report_data,
                                                                                  sizeof(actuator_report_data),
                                                                                  ota_on_actuator_subscribe, NULL),
                                                         BLE_GATT_DESCRIPTORS(BLE_OTA_INPUT_REPORT(1))),

                      [CHAR_ID_MOTION] = BLE_GATT_CHAR(BLE_UUID_SHORT_P(BLE_UUID_REPORT_CHAR),
                                                       BLE_GATT_NOTIFIABLE | BLE_GATT_PERM_ENC_READ,
                                                       BLE_GATT_CHAR_DATA_PLAIN(&motion_report_data,
                                                                                sizeof(motion_report_data),
                                                                                ota_on_motion_subscribe, NULL),
                                                       BLE_GATT_DESCRIPTORS(BLE_OTA_INPUT_REPORT(2))),

                      [CHAR_ID_VIBRATOR] = BLE_GATT_CHAR(BLE_UUID_SHORT_P(BLE_UUID_REPORT_CHAR),
                                                         BLE_GATT_PERM_ENC_WRITE,
                                                         BLE_GATT_CHAR_DATA_PLAIN(&vibrator_report_data,
                                                                                  sizeof(vibrator_report_data),
                                                                                  NULL, ota_on_vibrator_changed),
                                                         BLE_GATT_DESCRIPTORS(BLE_OTA_INPUT_REPORT(3))),

                      [CHAR_ID_BATTERY] = BLE_GATT_CHAR(BLE_UUID_SHORT_P(BLE_UUID_REPORT_CHAR),
                                                        BLE_GATT_NOTIFIABLE | BLE_GATT_PERM_ENC_READ,
                                                        BLE_GATT_CHAR_DATA_PLAIN(&battery_report_data,
                                                                                 sizeof(battery_report_data),
                                                                                 ota_on_battery_subscribe, NULL),
                                                        BLE_GATT_DESCRIPTORS(BLE_OTA_INPUT_REPORT(4))),
                      );



void gamepad_otas_actuator_set(struct gamepad_otas_s *otas,
                               const struct gamepad_actuator_s *act)
{
    actuator_report_data = *act;
    ble_gatt_db_char_changed(otas->reg.db, &otas->reg, CHAR_ID_ACTUATOR, &actuator_report_data, sizeof(actuator_report_data));
}

void gamepad_otas_motion_set(struct gamepad_otas_s *otas,
                             const struct gamepad_motion_s *act)
{
    motion_report_data = *act;
    ble_gatt_db_char_changed(otas->reg.db, &otas->reg, CHAR_ID_MOTION, &motion_report_data, sizeof(motion_report_data));
}

error_t gamepad_otas_init(struct gamepad_otas_s *otas,
                          struct gamepad_s *gamepad,
                          struct ble_gatt_db_s *db)
{
    otas->gamepad = gamepad;

    return ble_gatt_db_service_register(&otas->reg, db, &ota_service);
}

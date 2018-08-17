#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>
#include <mutek/printk.h>
#include "midi.h"

/* anchor write_handler */
static
uint8_t on_midi_data_write(struct ble_gattdb_client_s *client,
                           struct ble_gattdb_registry_s *reg, uint8_t charid,
                           const void *data, size_t size)
{
/* anchor end */
  struct midi_s *midi = midi_s_from_reg(reg);

  (void)midi;

/* anchor write_handler */
  printk("midi data write: %P\n", data, size);

  return 0;
}

/* anchor end */
/* anchor service_decl */
BLE_GATTDB_SERVICE_DECL(
  midi_service,
  BLE_GATTDB_SERVICE_PRIMARY | BLE_GATTDB_SERVICE_ADVERTISED,
  BLE_UUID_P(0x03B80E5A, 0xEDE8, 0x4B33, 0xA751, 0x6CE34EC4C700ULL),
  NULL,
  BLE_GATTDB_CHAR(
    BLE_UUID_P(0x7772E5DB, 0x3868, 0x4112, 0xA1A9, 0xF2669D106BF3ULL),
/* anchor details */
    // PERM_AUTH_READ implies pairing before subscribing,
    // but as there is not read handler, reading value
    // will not be permitted.
/* anchor service_decl */
    BLE_GATTDB_PERM_AUTH_WRITE | BLE_GATTDB_PERM_AUTH_READ | BLE_GATTDB_NOTIFIABLE,
/* anchor details */
    // Dont implement read
    // Call on_midi_data_write on write
    // Dont notify code on subscription
/* anchor service_decl */
    BLE_GATTDB_CHAR_DATA_DYNAMIC(NULL, on_midi_data_write, NULL)),
  );
/* anchor end */

static KROUTINE_EXEC(midi_button_pressed)
{
  struct midi_s *midi = KROUTINE_CONTAINER(kr, *midi, keyboard_rq.base.kr);
  uint16_t cur = midi->keyboard_state;
  uint16_t pressed = cur & ~midi->last_keyboard_state;
  uint16_t released = ~cur & midi->last_keyboard_state;

  printk("%s %04x pressed %04x released %04x\n", __FUNCTION__, midi->keyboard_state, pressed, released);

  midi->last_keyboard_state = midi->keyboard_state;
  midi->keyboard_rq.type = DEVICE_VALIO_WAIT_UPDATE;
  dev_valio_rq_init(&midi->keyboard_rq.base.kr, midi_button_pressed);
  DEVICE_OP(&midi->keyboard, request, &midi->keyboard_rq);

  uint8_t msg[20];
  uint8_t ptr = 0;

  msg[ptr++] = 0x80;

  while (pressed) {
    uint8_t button = __builtin_ctz(pressed);

    msg[ptr++] = 0x80;
    msg[ptr++] = 0x90;
    msg[ptr++] = 36 + button;
    msg[ptr++] = 0x7f;

    pressed &= ~(1 << button);
  }

  while (released) {
    uint8_t button = __builtin_ctz(released);

    msg[ptr++] = 0x80;
    msg[ptr++] = 0x80;
    msg[ptr++] = 36 + button;
    msg[ptr++] = 0x00;

    released &= ~(1 << button);
  }

  if (ptr > 1)
    ble_gattdb_char_changed(&midi->reg, 0, 1, msg, ptr);
    
}

#if defined(CONFIG_DRIVER_MPU6505)

static KROUTINE_EXEC(midi_motion)
{
  struct midi_s *midi = KROUTINE_CONTAINER(kr, *midi, motion_rq.base.kr);
  dev_timer_value_t now;

  DEVICE_OP(&midi->timer, get_value, &now, 0);
  if (now - midi->last_motion > midi->timeout) {
    midi->vert_accel_lp = midi->motion_state.axis[VALIO_MS_ACCEL_Z] << 4;
    midi->horiz_pos = 4096;
  } else {
    midi->vert_accel_lp
      += midi->motion_state.axis[VALIO_MS_ACCEL_Z]
      - (midi->vert_accel_lp >> 4);
    midi->horiz_pos -= midi->motion_state.axis[VALIO_MS_GYRO_Z];

    if (midi->horiz_pos < 0)
      midi->horiz_pos = 0;

    if (midi->horiz_pos > 8192)
      midi->horiz_pos = 8192;
  }

  midi->last_motion = now;

  uint8_t msg[20];
  uint8_t ptr = 0;

  msg[ptr++] = 0x80;

  int32_t vert_accel_hp
    = (midi->vert_accel_lp >> 4) - midi->motion_state.axis[VALIO_MS_ACCEL_Z];

  printk("\x1b[2KHorizontal position: % 5d", midi->horiz_pos);

  printk(" Vertical Accelerator Hipass: %d", vert_accel_hp);

  if (vert_accel_hp > 500) {
    msg[ptr++] = 0x80;
    msg[ptr++] = 0x90;
    msg[ptr++] = 36 + midi->horiz_pos / 1024;
    msg[ptr++] = 0x7f;

    msg[ptr++] = 0x80;
    msg[ptr++] = 0x80;
    msg[ptr++] = 36 + midi->horiz_pos / 1024;
    msg[ptr++] = 0x7f;
  }

  printk("\r");

  if (ptr > 1)
    ble_gatt_db_char_changed(&midi->reg, 0, 1, msg, ptr);

  midi->motion_rq.type = DEVICE_VALIO_WAIT_UPDATE;
  dev_valio_rq_init(&midi->motion_rq.base.kr, midi_motion);
  DEVICE_OP(&midi->motion_sensor, request, &midi->motion_rq);
}

#endif

error_t midi_service_register(struct midi_s *midi,
                              struct ble_gattdb_s *db)
{
  if (device_get_accessor_by_path(&midi->keyboard, NULL, "keyboard", DRIVER_CLASS_VALIO)) {
    printk("Keyboard not found\n");
    return -ENOENT;
  }

  dev_valio_rq_init(&midi->keyboard_rq.base.kr, midi_button_pressed);
  midi->keyboard_rq.type = DEVICE_VALIO_READ;
  midi->keyboard_rq.attribute = VALIO_KEYBOARD_MAP;
  midi->keyboard_rq.data = &midi->keyboard_state;

  DEVICE_OP(&midi->keyboard, request, &midi->keyboard_rq);

#if defined(CONFIG_DRIVER_MPU6505)
  if (device_get_accessor_by_path(&midi->motion_sensor, NULL, "accgyr0", DRIVER_CLASS_VALIO)) {
    printk("Motion sensor not found\n");
    return -ENOENT;
  }

  if (device_get_accessor_by_path(&midi->timer, NULL, "rtc1", DRIVER_CLASS_TIMER)) {
    printk("Timer not found\n");
    return -ENOENT;
  }

  dev_valio_rq_init(&midi->motion_rq.base.kr, midi_motion);
  midi->motion_rq.type = DEVICE_VALIO_READ;
  midi->motion_rq.attribute = VALIO_MS_STATE;
  midi->motion_rq.data = &midi->motion_state;

  DEVICE_OP(&midi->motion_sensor, request, &midi->motion_rq);

  dev_timer_init_sec(&midi->timer, &midi->timeout, 0, 2, 10);
  midi->last_motion = 0;
#endif

  return ble_gattdb_service_register(&midi->reg, db, &midi_service);
}

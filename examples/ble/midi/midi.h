#ifndef MIDI_H_
#define MIDI_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/timer.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>
#if defined(CONFIG_DRIVER_MPU6505)
# include <device/valio/motion_sensor.h>
#endif

struct midi_s
{
  struct ble_gattdb_registry_s reg;

  struct device_valio_s keyboard;
  struct dev_valio_rq_s keyboard_rq;
  uint16_t keyboard_state;
  uint16_t last_keyboard_state;

#if defined(CONFIG_DRIVER_MPU6505)
  struct device_valio_s motion_sensor;
  struct dev_valio_rq_s motion_rq;
  struct valio_ms_state_s motion_state;

  int32_t vert_accel_lp;
  int32_t horiz_pos;

  struct device_timer_s timer;
  dev_timer_delay_t timeout;
  dev_timer_value_t last_motion;
#endif
};

STRUCT_COMPOSE(midi_s, reg);

error_t midi_service_register(struct midi_s *midi,
                             struct ble_gattdb_s *db);

#endif

#ifndef LED_H_
#define LED_H_

#include <hexo/types.h>
#include <hexo/decls.h>
#include <device/class/timer.h>
#include <device/class/gpio.h>

struct led_s
{
  struct device_gpio_s gpio;

  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  dev_timer_delay_t interval;

  bool_t scheduled;
  bool_t polarity;
  uint8_t pin;

  dev_timer_value_t last_switch;
  dev_timer_delay_t on_time;
  dev_timer_delay_t off_time;
  uint32_t repeat_count;
  bool_t cur_state;
};

STRUCT_COMPOSE(led_s, timer_rq);

error_t led_init(struct led_s *leds, uint8_t pin, bool_t polarity);
void led_cleanup(struct led_s *leds);
void led_blink(struct led_s *leds,
               uint32_t on_time, uint32_t off_time,
               uint32_t repeat_count);

#endif

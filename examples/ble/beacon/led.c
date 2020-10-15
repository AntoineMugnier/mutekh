#include <hexo/types.h>
#include <device/class/timer.h>
#include <mutek/printk.h>

#include "led.h"

#define dprintk(...) do{}while(0)
//#define dprintk printk

static KROUTINE_EXEC(led_delay_done);

error_t led_init(struct led_s *led, uint8_t pin, bool_t polarity)
{
  error_t err;

  err = device_get_accessor_by_path(&led->gpio.base, NULL, "gpio", DRIVER_CLASS_GPIO);
  if (err) {
    dprintk("Error getting GPIO device: %d\n", err);
    return -ENOENT;
  }

  err = device_get_accessor_by_path(&led->timer.base, NULL, "rtc1", DRIVER_CLASS_TIMER);
  if (err) {
    dprintk("Error getting rtc device: %d\n", err);
    device_put_accessor(&led->gpio.base);
    return -ENOENT;
  }

  device_start(&led->timer.base);

  dev_timer_rq_init(&led->timer_rq, led_delay_done);

  led->polarity = polarity;
  led->pin = pin;

  const uint8_t *mask = polarity ? dev_gpio_mask0 : dev_gpio_mask1;
  
  DEVICE_OP(&led->gpio, set_output, led->pin, led->pin, mask, mask);
  
  DEVICE_OP(&led->gpio, set_mode, led->pin, led->pin, dev_gpio_mask1,
            polarity ? DEV_PIN_OPENSOURCE : DEV_PIN_OPENDRAIN);

  return 0;
}

void led_cleanup(struct led_s *led)
{
  DEVICE_OP(&led->timer, cancel, &led->timer_rq);

  DEVICE_OP(&led->gpio, set_mode, led->pin, led->pin,
            dev_gpio_mask1, DEV_PIN_DISABLED);

  device_stop(&led->timer.base);

  device_put_accessor(&led->timer.base);
  device_put_accessor(&led->gpio.base);
}

static void led_schedule_next(struct led_s *led, dev_timer_value_t next);

static void led_update(struct led_s *led)
{
  dev_timer_value_t now;
  const uint8_t *value;
  dev_timer_value_t next_switch;

  if (led->scheduled) {
    error_t err = DEVICE_OP(&led->timer, cancel, &led->timer_rq);
    if (err)
      return;
    led->scheduled = 0;
  }

  DEVICE_OP(&led->timer, get_value, &now, 0);

  dprintk("led update at %lld\n", now);

  next_switch = led->last_switch;
  
  while (next_switch <= now) {
    led->cur_state = !led->cur_state;
    if (led->cur_state)
      next_switch = next_switch + led->on_time;
    else
      next_switch = next_switch + led->off_time;

    dprintk(" led state %d next switch %lld\n", led->cur_state, next_switch);

    if (!led->cur_state) {
      led->repeat_count--;
      
      if (!led->repeat_count)
        break;
    }
  }

  led->last_switch = next_switch;

  value = led->cur_state == led->polarity ? dev_gpio_mask1 : dev_gpio_mask0;

  DEVICE_OP(&led->gpio, set_output, led->pin, led->pin, value, value);
  
  if (!led->repeat_count)
    return;

  dprintk(" next at %lld\n", next_switch);

  led_schedule_next(led, next_switch);
}

static void led_schedule_next(struct led_s *led, dev_timer_value_t next)
{
  if (led->scheduled)
    return;

  led->timer_rq.deadline = next;
  led->timer_rq.delay = 0;

  led->scheduled = 1;

  error_t err = DEVICE_OP(&led->timer, request, &led->timer_rq);

  dprintk("led schdule at %lld: %d\n", next, err);

  if (err == -ETIMEDOUT)
    kroutine_exec(&led->timer_rq.base.kr);
}

static KROUTINE_EXEC(led_delay_done)
{
  struct led_s *led = KROUTINE_CONTAINER(kr, *led, timer_rq.base.kr);

  led->scheduled = 0;

  dprintk("led delay done\n");

  led_update(led);
}

void led_blink(struct led_s *led,
               uint32_t on_time, uint32_t off_time,
               uint32_t repeat_count)
{
  const uint8_t *mask;

  dev_timer_init_sec(&led->timer, &led->on_time, 0, on_time, 1000);
  dev_timer_init_sec(&led->timer, &led->off_time, 0, off_time, 1000);
  DEVICE_OP(&led->timer, get_value, &led->last_switch, 0);
  led->cur_state = 0;
  led->repeat_count = repeat_count;

  mask = led->polarity ? dev_gpio_mask1 : dev_gpio_mask0;
  DEVICE_OP(&led->gpio, set_output, led->pin, led->pin, mask, mask);

  dprintk("led flash %d/%dx%d\n", on_time, off_time, repeat_count);

  led_schedule_next(led, led->last_switch + led->on_time);
}

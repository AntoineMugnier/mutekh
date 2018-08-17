#include <hexo/bit.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>

#include <hexo/enum.h>

struct app_s
{
  uint64_t old_state;
  uint64_t cur_state;
  struct dev_valio_rq_s kbd_rq;
  struct device_valio_s keyboard;
};

static KROUTINE_EXEC(kbd_done)
{
  struct app_s *app = KROUTINE_CONTAINER(kr, *app, kbd_rq.base.kr);
  //printk("Keyboard state: %llx\n", app->cur_state);

  uint64_t pressed = app->cur_state & ~app->old_state;
  uint64_t released = ~app->cur_state & app->old_state;

  if (pressed) {
    printk("Pressed:");
    while (pressed) {
      uint8_t key = bit_ctz64(pressed);

      printk(" %d", key);

      BIT_CLEAR(pressed, key);
    }
    printk("\n");
  }

  if (released) {
    printk("Released:");
    while (released) {
      uint8_t key = bit_ctz64(released);

      printk(" %d", key);

      BIT_CLEAR(released, key);
    }
    printk("\n");
  }

  app->old_state = app->cur_state;

  DEVICE_OP(&app->keyboard, request, &app->kbd_rq);
}

void app_start(void)
{
  error_t err;

  struct app_s *app;

  app = mem_alloc(sizeof(*app), mem_scope_sys);
  assert(app);

  memset(app, 0, sizeof(*app));

  err = device_get_accessor_by_path(&app->keyboard.base, NULL, "keyboard", DRIVER_CLASS_VALIO);
  if (err) {
    printk("no keyboard\n");
    abort();
  }

  app->kbd_rq.attribute = VALIO_KEYBOARD_MAP;
  app->kbd_rq.data = &app->cur_state;
  app->kbd_rq.type = DEVICE_VALIO_WAIT_EVENT;
  dev_valio_rq_init(&app->kbd_rq, kbd_done);

  DEVICE_OP(&app->keyboard, request, &app->kbd_rq);
}

#include <hexo/bit.h>

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/class/pcm.h>
#include <device/class/cmu.h>
#include <device/class/valio.h>
#include <device/valio/keyboard.h>

#include <math.h>

#define RATE 16000
#define FPS 20

struct app_s
{
  uint64_t old_state;
  uint64_t cur_state;
  struct dev_valio_rq_s kbd_rq;
  struct device_valio_s keyboard;

  struct device_pcm_s pcm;
  struct dev_pcm_rq_s pcm_rq;
  struct dev_pcm_stream_s stream[1];

  bool_t pcm_run;
};

static KROUTINE_EXEC(pcm_frame)
{
  struct app_s *app = KROUTINE_CONTAINER(kr, *app, pcm_rq.base.kr);

  static const char vu[] = "================================================================================";

  const int16_t *lb = app->pcm_rq.stream[0].buffer[app->pcm_rq.offline_buffer_index];
  size_t ls = app->pcm_rq.stream[0].stride / 2;
  uint64_t power = 0;

  if (flags & DEV_PCM_END_FLAG) {
    printk("PCM Done\n");
    return;
  }

  if (app->pcm_run)
    atomic_set(&app->pcm_rq.frames_left, 3);

  for (size_t i = 0; i < app->pcm_rq.sample_count; ++i) {
    power += lb[i * ls] * lb[i * ls] / 8;
  }

  uint8_t db = (uint32_t)(10 * log10(power));

  printk("Power: %d dB %s\n", db, &vu[sizeof(vu) - (db - 40)]);
}

static KROUTINE_EXEC(kbd_done)
{
  struct app_s *app = KROUTINE_CONTAINER(kr, *app, kbd_rq.base.kr);
  error_t err;

  uint64_t pressed = app->cur_state & ~app->old_state;
  uint64_t released = ~app->cur_state & app->old_state;

  printk("Keyboard state: %llx, pressed %llx, released %llx\n",
         app->cur_state, pressed, released);

  if (pressed) {
    app->pcm_run = !app->pcm_run;

    if (app->pcm_run) {
      err = DEVICE_OP(&app->pcm, request, &app->pcm_rq);
      if (err) {
        printk("cannot start stream: %d\n", err);
        app->pcm_run = 0;
      }
    }
  }

  app->old_state = app->cur_state;

  DEVICE_OP(&app->keyboard, request, &app->kbd_rq);
}

void app_start(void)
{
  error_t err;

  struct app_s *app;
  struct device_cmu_s clock;

  app = mem_alloc(sizeof(*app), mem_scope_sys);
  assert(app);

  memset(app, 0, sizeof(*app));

  err = device_get_accessor_by_path(&app->keyboard.base, NULL, "keyboard", DRIVER_CLASS_VALIO);
  if (err) {
    printk("no keyboard\n");
    abort();
  }

  err = device_get_accessor_by_path(&app->pcm.base, NULL, "pcm*", DRIVER_CLASS_PCM);
  if (err) {
    printk("no pcm\n");
    abort();
  }

  err = device_get_accessor_by_path(&clock.base, NULL, "clock*", DRIVER_CLASS_CMU);
  assert(!err);
  DEVICE_OP(&clock, app_configid_set, 2);
  device_put_accessor(&clock.base);

  app->kbd_rq.attribute = VALIO_KEYBOARD_MAP;
  app->kbd_rq.data = &app->cur_state;
  app->kbd_rq.type = DEVICE_VALIO_WAIT_EVENT;
  dev_valio_rq_init(&app->kbd_rq, kbd_done);

  app->pcm_rq.sample_rate = RATE;
  app->pcm_rq.stream_count = 1;
  app->pcm_rq.sample_count = RATE / FPS;
  app->pcm_rq.stream[0].sample_type = DEV_PCM_DT_INT16LE;
  app->pcm_rq.stream[0].direction = DEV_PCM_DIR_INPUT;
  app->pcm_rq.stream[0].channel_id = 0;
  atomic_set(&app->pcm_rq.frames_left, 3);
  dev_pcm_rq_init(&app->pcm_rq, pcm_frame);

  app->pcm_run = 0;

  DEVICE_OP(&app->keyboard, request, &app->kbd_rq);
}

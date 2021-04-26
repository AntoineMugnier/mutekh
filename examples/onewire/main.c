#define LOGK_MODULE_ID "main"

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>
#include <device/class/onewire.h>
#include <device/class/timer.h>
#include <device/driver.h>

#define DEVICE_COUNT_MAX 32

struct device_entry {
  struct dev_onewire_rom_s rom, collision;
};
  
struct app_s
{
  struct device_onewire_s onewire;
  struct dev_onewire_rq_s onewire_rq;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  struct dev_onewire_transfer_s transfer[2];
  uint8_t buffer[32];
  struct device_entry device[DEVICE_COUNT_MAX];
  size_t device_count;
  size_t device_next;
  size_t device_cur;
};

STRUCT_COMPOSE(app_s, onewire_rq);
STRUCT_COMPOSE(app_s, timer_rq);

static void polling_next(struct app_s *app);
static void polling_start(struct app_s *app);
  
static KROUTINE_EXEC(scratchpad_read_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct app_s *app = app_s_from_onewire_rq(rq);
  int16_t temp;

  if (app->onewire_rq.error) {
    logk("Read scratchpad failure: %s", strerror(-app->onewire_rq.error));
    polling_next(app);
    return;
  }

  temp = endian_le16_na_load(app->buffer+1);
  
  logk("Scratchpad: %P, temp: %d.%04d", app->buffer+1, 9,
       temp / 16, 625 * ((temp < 0 ? 0x10 - (temp & 0xf) : (temp & 0xf))));

  polling_next(app);
}

static KROUTINE_EXEC(convert_wait_done)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct app_s *app = app_s_from_timer_rq(rq);

  dev_onewire_rq_init(&app->onewire_rq, scratchpad_read_done);
  app->onewire_rq.data.rom = &app->device[app->device_cur].rom;
  app->onewire_rq.data.transfer = app->transfer;
  app->onewire_rq.data.transfer_count = 2;
  app->onewire_rq.type = DEV_ONEWIRE_DATA;
  app->transfer[0].direction = DEV_ONEWIRE_WRITE;
  app->transfer[0].data = &app->buffer[0];
  app->buffer[0] = 0xbe;
  app->transfer[0].size = 1;
  app->transfer[1].direction = DEV_ONEWIRE_READ;
  app->transfer[1].data = &app->buffer[1];
  app->transfer[1].size = 9;
  
  DEVICE_OP(&app->onewire, request, &app->onewire_rq);
  
}
  
static KROUTINE_EXEC(convert_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct app_s *app = app_s_from_onewire_rq(rq);

  if (app->onewire_rq.error) {
    logk("Convert command failure: %s", strerror(-app->onewire_rq.error));
    polling_next(app);
    return;
  }

  dev_timer_rq_init(&app->timer_rq, convert_wait_done);
  dev_timer_init_sec(&app->timer, &app->timer_rq.delay, NULL, 780, 1000);

  DEVICE_OP(&app->timer, request, &app->timer_rq);
}

static KROUTINE_EXEC(wait_start_done)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct app_s *app = app_s_from_timer_rq(rq);

  polling_start(app);
}

static void polling_start_later(struct app_s *app)
{
  dev_timer_rq_init(&app->timer_rq, wait_start_done);
  dev_timer_init_sec(&app->timer, &app->timer_rq.delay, NULL, 1, 1);

  DEVICE_OP(&app->timer, request, &app->timer_rq);
}

static void temp_convert(struct app_s *app)
{
  dev_onewire_rq_init(&app->onewire_rq, convert_done);
  app->onewire_rq.data.rom = &app->device[app->device_cur].rom;
  app->onewire_rq.data.transfer = app->transfer;
  app->onewire_rq.data.transfer_count = 1;
  app->onewire_rq.type = DEV_ONEWIRE_DATA;
  app->transfer[0].direction = DEV_ONEWIRE_WRITE;
  app->transfer[0].data = &app->buffer[0];
  app->transfer[0].size = 1;
  app->buffer[0] = 0x44;
  
  DEVICE_OP(&app->onewire, request, &app->onewire_rq);
}

static void polling_next(struct app_s *app)
{
  for (size_t i = app->device_next; i < app->device_count; ++i) {
    if (app->device[i].rom.structured.family == 0x28) {
      app->device_cur = i;
      app->device_next = i+1;

      temp_convert(app);

      return;
    }
  }

  polling_start_later(app);
}

static void polling_start(struct app_s *app)
{
  app->device_next = 0;

  polling_next(app);
}

static KROUTINE_EXEC(search_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct app_s *app = app_s_from_onewire_rq(rq);

  if (app->onewire_rq.error) {
    logk("ROM lookup failure: %s", strerror(-app->onewire_rq.error));
    goto done;
  }

  logk("Found ROM address %016llx, collision %016llx",
       app->onewire_rq.search.rom.raw, app->onewire_rq.search.collision.raw);

  app->device[app->device_count].rom = app->onewire_rq.search.rom;
  app->device[app->device_count].collision = app->onewire_rq.search.collision;
  app->device_count++;

  for (size_t i = 0; i < app->device_count; ++i) {
    struct device_entry *e = app->device + i;

    if (e->collision.raw) {
      if (app->device_count >= DEVICE_COUNT_MAX) {
        logk("ROM array overflow");
        break;
      }

      uint8_t cb = bit_lsb_index(e->collision.raw);

      app->onewire_rq.search.rom.raw = e->rom.raw ^ bit(cb);
      app->onewire_rq.search.discover_after = cb+1;
      app->onewire_rq.type = DEV_ONEWIRE_SEARCH;

      logk("Starting enumeration after %016llx/%d",
           e->rom.raw, cb);
      e->collision.raw &= ~bit(cb);

      DEVICE_OP(&app->onewire, request, &app->onewire_rq);
      return;
    }
  }

 done:
  logk("Enumeration done, %d device found", app->device_count);

  if (app->device_count)
    polling_start(app);
  
  return;
}

static void enumeration_start(struct app_s *app)
{
  logk("Starting enumeration");

  app->device_count = 0;

  dev_onewire_rq_init(&app->onewire_rq, search_done);
  app->onewire_rq.search.rom.raw = 0;
  app->onewire_rq.search.discover_after = 0;
  app->onewire_rq.search.alarm_only = 0;
  app->onewire_rq.type = DEV_ONEWIRE_SEARCH;

  DEVICE_OP(&app->onewire, request, &app->onewire_rq);
}

void app_start(void)
{
  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  error_t err;

  memset(app, 0, sizeof(*app));
  
  logk("1-Wire test");

  err = device_get_accessor_by_path(&app->onewire.base, NULL, "1wire", DRIVER_CLASS_ONEWIRE);
  ensure(!err && "Error getting 1-Wire device");

  err = device_get_accessor_by_path(&app->timer.base, NULL, "rtc* timer*", DRIVER_CLASS_TIMER);
  ensure(!err && "Error getting Timer device");

  enumeration_start(app);
}

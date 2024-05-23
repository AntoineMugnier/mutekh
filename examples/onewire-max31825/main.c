#include <stdint.h>
#include <string.h>
#define LOGK_MODULE_ID "main"

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>
#include <device/class/onewire.h>
#include <device/class/timer.h>
#include <device/driver.h>
#include <device/class/gpio.h>


struct app_s
{
  struct device_onewire_s onewire;
  struct device_timer_s timer;
  struct dev_onewire_rq_s onewire_rq;
    struct dev_timer_rq_s timer_rq;
  struct dev_onewire_transfer_s transfer[2];
  uint8_t tx_data[10];
  uint8_t rx_data[10];
  struct device_gpio_s led_gpio;

};

STRUCT_COMPOSE(app_s, onewire_rq);
STRUCT_COMPOSE(app_s, timer_rq);

#define SELECT_ADRESS_CMD 0x70
#define DETECT_ADRESS_CMD 0x88
#define READ_SCRATCHPAD_CMD 0xBE
#define CONVERT_T_CMD 0x44
#define ADDR0 0b110110
#define ADDR1 0b111111
void start_read_scratchpad(struct app_s *app);
void print_sp(uint8_t rx_data[]);


void convert_temp(int16_t raw_temp){
  logk(" RAW VALUE IS %d", raw_temp);
  logk(" TEMPERATURE VALUE IS  %d.%04d ",
   raw_temp / 16, 625 * ((raw_temp < 0 ? 0x10 - (raw_temp & 0xf) : (raw_temp & 0xf))));

}

static KROUTINE_EXEC(conversion_done)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct app_s *app = app_s_from_timer_rq(rq);
  start_read_scratchpad(app);

}
static KROUTINE_EXEC(temperature_measured)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct app_s *app = app_s_from_onewire_rq(rq);

  dev_timer_rq_init(&app->timer_rq, conversion_done);
  dev_timer_init_sec(&app->timer, &app->timer_rq.delay, 0, 2, 1);
  DEVICE_OP(&app->timer, request, &app->timer_rq);

}

static void start_temperature_request(struct app_s *app)
{

  logk("Starting temperature request");

  dev_onewire_rq_init(&app->onewire_rq, temperature_measured);

  app->onewire_rq.data.transfer = app->transfer;
  app->onewire_rq.data.transfer_count = 1;
  app->onewire_rq.type = DEV_ONEWIRE_RAW;


  app->transfer[0].direction = DEV_ONEWIRE_WRITE;
  app->tx_data[0] = SELECT_ADRESS_CMD;
  app->tx_data[1] = ADDR0;
  app->tx_data[2] = CONVERT_T_CMD;
  app->transfer[0].data = app->tx_data;
  app->transfer[0].size = 3;
  
  DEVICE_OP(&app->onewire, request, &app->onewire_rq);
}

void print_sp(uint8_t rx_data[]){
  logk("Scratchpad is:");
  logk("Temperature LSB %x", rx_data[0]);
  logk("Temperature MSB %x", rx_data[1]);
  logk("Status [TH, TL state, address] %x", rx_data[2]);
  logk("Configuration %x", rx_data[3]);
  logk("TH MSB  %x", rx_data[4]);
  logk("TH LSB  %x", rx_data[5]);
  logk("TL LSB %x", rx_data[6]);
  logk("TL MSB  %x", rx_data[7]);
  logk("CRC %x", rx_data[8]);
  logk("adress  %x", rx_data[2] & 0x3f );
  uint16_t raw_temp = (rx_data[1] << 8) | rx_data[0];
  convert_temp(raw_temp);
}

static KROUTINE_EXEC(scratchpad_read_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct app_s *app = app_s_from_onewire_rq(rq);
  print_sp(app->rx_data);
  static int counter = 0;
  if (counter == 0) start_temperature_request(app);
  counter++;
}

void start_read_scratchpad(struct app_s *app){

  logk("Starting scratchpad read");

  dev_onewire_rq_init(&app->onewire_rq, scratchpad_read_done);

  app->onewire_rq.data.transfer = app->transfer;
  app->onewire_rq.data.transfer_count = 2;
  app->onewire_rq.type = DEV_ONEWIRE_RAW;


  app->transfer[0].direction = DEV_ONEWIRE_WRITE;
  app->tx_data[0] = SELECT_ADRESS_CMD;
  app->tx_data[1] = ADDR0;
  app->tx_data[2] = READ_SCRATCHPAD_CMD;
  app->transfer[0].data = app->tx_data;
  app->transfer[0].size = 3;
  memset(app->rx_data, 0, 9);
  app->transfer[1].direction = DEV_ONEWIRE_READ;
  app->transfer[1].data = app->rx_data;
  app->transfer[1].size = 9;

  DEVICE_OP(&app->onewire, request, &app->onewire_rq);
}

static KROUTINE_EXEC(detect_adress_done)
{

  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct app_s *app = app_s_from_onewire_rq(rq);
  start_read_scratchpad(app);
}


static void enumeration_start(struct app_s *app)
{

  logk("Starting enumeration");

  dev_onewire_rq_init(&app->onewire_rq, detect_adress_done);

  app->onewire_rq.data.transfer = app->transfer;
  app->onewire_rq.data.transfer_count = 1;
  app->onewire_rq.data.rom=0;
  app->onewire_rq.type = DEV_ONEWIRE_DATA;


  app->transfer[0].direction = DEV_ONEWIRE_WRITE;
  app->tx_data[0] = DETECT_ADRESS_CMD;
  app->transfer[0].data = app->tx_data;
  app->transfer[0].size = 1;
  
  DEVICE_OP(&app->onewire, request, &app->onewire_rq);
}

static KROUTINE_EXEC(device_power_on)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct app_s *app = app_s_from_timer_rq(rq);
  enumeration_start(app);

}

void app_start(void)
{

  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  error_t err;

  memset(app, 0, sizeof(*app));
  
  logk("1-Wire test");
  
  err = device_get_accessor_by_path(&app->led_gpio.base, NULL, "gpio", DRIVER_CLASS_GPIO);
  ensure(!err && "Error getting GPIO device");

  err = device_get_accessor_by_path(&app->onewire.base, NULL, "R", DRIVER_CLASS_ONEWIRE);
  ensure(!err && "Error getting 1-Wire device");

  err = device_get_accessor_by_path(&app->timer.base, NULL, "rtc* timer*", DRIVER_CLASS_TIMER);
  ensure(!err && "Error getting Timer device");


 // Only one GPIO is driven high to provide power to the OneWire Devices
  //DEVICE_OP(&app->led_gpio, set_mode, 11,11, dev_gpio_mask1, DEV_PIN_PUSHPULL);
  //DEVICE_OP(&app->led_gpio, set_output, 11, 11, dev_gpio_mask1, dev_gpio_mask1);

  DEVICE_OP(&app->led_gpio, set_mode, 13, 13, dev_gpio_mask1, DEV_PIN_PUSHPULL);
  DEVICE_OP(&app->led_gpio, set_output, 13,13, dev_gpio_mask1, dev_gpio_mask1);



  dev_timer_rq_init(&app->timer_rq, device_power_on);
  dev_timer_init_sec(&app->timer, &app->timer_rq.delay, 0, 3, 1);

  DEVICE_OP(&app->timer, request, &app->timer_rq);
  logk("Powering device ON");

}

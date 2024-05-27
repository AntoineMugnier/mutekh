/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) 2017, Nicolas Pouillon <nipo@ssji.net>
*/

#include <stdint.h>
#define LOGK_MODULE_ID "3182"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/valio.h>
#include <device/valio/temperature.h>
#include <device/class/spi.h>
#include <device/class/timer.h>
#include <device/class/onewire.h>
#include <device/class/gpio.h>

enum max31825_state_e
{
  MAX31825_INITIALIZING,
  MAX31825_IDLE,
  MAX31825_CONVERTING_TEMP,
  MAX31825_READING_TEMP,

};

#define SELECT_ADRESS_CMD 0x70
#define DETECT_ADRESS_CMD 0x88
#define READ_SCRATCHPAD_CMD 0xBE
#define CONVERT_T_CMD 0x44

struct max31825_context_s
{
  struct device_gpio_s pull_up_gpio;
  struct device_onewire_s onewire;
  struct dev_onewire_rq_s onewire_rq;
  struct device_timer_s timer;
  struct dev_timer_rq_s timer_rq;
  struct dev_onewire_transfer_s transfer[2];
  struct dev_valio_rq_s * current_user_rq;

  //Dynamic parameters
  uintptr_t init_charging_time_us;
  uintptr_t device_address;
  uint8_t tx_data[10];
  uint8_t rx_data[10];

  dev_request_queue_root_t queue;

  enum max31825_state_e state;
  
  int32_t temp;

  gpio_id_t pull_up_gpio_id;
};

STRUCT_COMPOSE(max31825_context_s, onewire_rq);
STRUCT_COMPOSE(max31825_context_s, timer_rq);

DRIVER_PV(struct max31825_context_s);

void handle_request(struct max31825_context_s *pv);

void process_next_request(struct max31825_context_s *pv){
  pv->current_user_rq = dev_valio_rq_pop(&pv->queue);

  if(pv->current_user_rq){
    handle_request(pv);
  };
  
}

void convert_temp(int16_t raw_temp){
  logk(" RAW VALUE IS %d", raw_temp);
  logk(" TEMPERATURE VALUE IS  %d.%04d ",
   raw_temp / 16, 625 * ((raw_temp < 0 ? 0x10 - (raw_temp & 0xf) : (raw_temp & 0xf))));

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
  struct max31825_context_s *pv = max31825_context_s_from_onewire_rq(rq);

  print_sp(pv->rx_data);

  dev_valio_rq_done(pv->current_user_rq);
  
  //TODO Spin lock here ?
  pv->state = MAX31825_IDLE;
  process_next_request(pv);

}

void start_read_scratchpad(struct max31825_context_s *pv){

  logk("Starting scratchpad read");
  pv->state = MAX31825_READING_TEMP;

  dev_onewire_rq_init(&pv->onewire_rq, scratchpad_read_done);

  pv->onewire_rq.data.transfer = pv->transfer;
  pv->onewire_rq.data.transfer_count = 2;
  pv->onewire_rq.type = DEV_ONEWIRE_RAW;


  pv->transfer[0].direction = DEV_ONEWIRE_WRITE;
  pv->tx_data[0] = SELECT_ADRESS_CMD;
  pv->tx_data[1] = pv->device_address;
  pv->tx_data[2] = READ_SCRATCHPAD_CMD;
  pv->transfer[0].data = pv->tx_data;
  pv->transfer[0].size = 3;
  memset(pv->rx_data, 0, 9);
  pv->transfer[1].direction = DEV_ONEWIRE_READ;
  pv->transfer[1].data = pv->rx_data;
  pv->transfer[1].size = 9;

  DEVICE_OP(&pv->onewire, request, &pv->onewire_rq);
}

static KROUTINE_EXEC(conversion_done)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct max31825_context_s *pv = max31825_context_s_from_timer_rq(rq);
  start_read_scratchpad(pv);
}

static KROUTINE_EXEC(performing_conversion)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct max31825_context_s *pv = max31825_context_s_from_onewire_rq(rq);

  dev_timer_rq_init(&pv->timer_rq, conversion_done);
  dev_timer_init_sec(&pv->timer, &pv->timer_rq.delay, 0, 2, 1);
  DEVICE_OP(&pv->timer, request, &pv->timer_rq);

}

static void start_temperature_request(struct max31825_context_s *pv)
{

  logk("Starting temperature request");
  pv->state = MAX31825_CONVERTING_TEMP;

  dev_onewire_rq_init(&pv->onewire_rq, performing_conversion);

  pv->onewire_rq.data.transfer = pv->transfer;
  pv->onewire_rq.data.transfer_count = 1;
  pv->onewire_rq.type = DEV_ONEWIRE_RAW;


  pv->transfer[0].direction = DEV_ONEWIRE_WRITE;
  pv->tx_data[0] = SELECT_ADRESS_CMD;
  pv->tx_data[1] = pv->device_address;
  pv->tx_data[2] = CONVERT_T_CMD;
  pv->transfer[0].data = pv->tx_data;
  pv->transfer[0].size = 3;
  
  DEVICE_OP(&pv->onewire, request, &pv->onewire_rq);
}


void handle_request(struct max31825_context_s *pv){

  start_temperature_request(pv);
}

static
DEV_VALIO_REQUEST(max31825_request)
{
  struct device_s *dev = accessor->dev;
  struct max31825_context_s *pv = dev->drv_pv;

  if (rq->type == DEVICE_VALIO_READ &&
  rq->attribute == VALIO_TEMPERATURE_VALUE){

    rq->error = 0;
    LOCK_SPIN_IRQ(&dev->lock);
    dev_valio_rq_pushback(&pv->queue, rq);

    if (pv->state == MAX31825_IDLE){
      process_next_request(pv);  
    }
    LOCK_RELEASE_IRQ(&dev->lock);


  }
  else{
        rq->error = -ENOTSUP;
        dev_valio_rq_done(rq);
  }
}

static
DEV_VALIO_CANCEL(max31825_cancel)
{

}

#define max31825_use dev_use_generic


static KROUTINE_EXEC(detect_adress_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct max31825_context_s *pv = max31825_context_s_from_onewire_rq(rq);
  process_next_request(pv);
}



static
KROUTINE_EXEC(device_power_on)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct max31825_context_s *pv = max31825_context_s_from_timer_rq(rq);
  
  logk_trace("Starting enumeration");

  dev_onewire_rq_init(&pv->onewire_rq, detect_adress_done);

  pv->onewire_rq.data.transfer = pv->transfer;
  pv->onewire_rq.data.transfer_count = 1;
  pv->onewire_rq.data.rom=0;
  pv->onewire_rq.type = DEV_ONEWIRE_DATA;


  pv->transfer[0].direction = DEV_ONEWIRE_WRITE;
  pv->tx_data[0] = DETECT_ADRESS_CMD;
  pv->transfer[0].data = pv->tx_data;
  pv->transfer[0].size = 1;

  DEVICE_OP(&pv->onewire, request, &pv->onewire_rq);

}

static DEV_INIT(max31825_init)
{
  struct max31825_context_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  pv->state = MAX31825_INITIALIZING;
  dev->drv_pv = pv;

  err = device_get_param_dev_accessor(dev, "gpio",  &pv->pull_up_gpio.base, DRIVER_CLASS_GPIO);
  if(err){
    return err;
  }

  err = device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER);
  if(err){
    return err;
  }
  gpio_width_t width;
  err = device_gpio_get_setup(&pv->pull_up_gpio, dev, "pull_up", &pv->pull_up_gpio_id, &width);
  if(err){
    return err;
  }

  err = device_get_param_dev_accessor(dev, "onewire", &pv->onewire.base, DRIVER_CLASS_ONEWIRE);
  if(err){
    return err;
  }
  err = device_get_param_uint(dev, "init_charging_time_us", &pv->init_charging_time_us);
  if(err){
    return err;
  }

  err = device_get_param_uint(dev, "device_address", &pv->device_address);
  if(err){
    return err;
  }

  dev_rq_queue_init(&pv->queue);

  DEVICE_OP(&pv->pull_up_gpio, set_mode, pv->pull_up_gpio_id, pv->pull_up_gpio_id, dev_gpio_mask1, DEV_PIN_PUSHPULL);
  DEVICE_OP(&pv->pull_up_gpio, set_output, pv->pull_up_gpio_id, pv->pull_up_gpio_id, dev_gpio_mask1, dev_gpio_mask1);

  dev_timer_rq_init(&pv->timer_rq, device_power_on);
  dev_timer_init_sec(&pv->timer, &pv->timer_rq.delay, 0, pv->init_charging_time_us, 1000000);

  DEVICE_OP(&pv->timer, request, &pv->timer_rq);
  
  return 0;
}

static DEV_CLEANUP(max31825_cleanup)
{

  return 0;
}

DRIVER_DECLARE(max31825_drv, 0, "max31825", max31825,
               DRIVER_VALIO_METHODS(max31825));

DRIVER_REGISTER(max31825_drv);

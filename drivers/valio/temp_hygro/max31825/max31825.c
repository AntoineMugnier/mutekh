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
#include <string.h>
#define LOGK_MODULE_ID "3182"

#include <hexo/types.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/valio.h>
#include <device/valio/temperature.h>
#include <device/class/onewire.h>
#include <device/class/gpio.h>

enum max31825_state_e
{
  MAX31825_INITIALIZING,
  MAX31825_IDLE,
  MAX31825_CONVERTING_TEMP,
  MAX31825_READING_TEMP

};

// Chip function commands
#define SELECT_ADRESS_CMD 0x70
#define DETECT_ADRESS_CMD 0x88
#define READ_SCRATCHPAD_CMD 0xBE
#define CONVERT_T_CMD 0x44

// Conversion time specified for every temperature sensor resolution
#define MAX31825_CONV_TIME_8_BITS_MS 20
#define MAX31825_CONV_TIME_9_BITS_MS 40
#define MAX31825_CONV_TIME_10_BITS_MS 80
#define MAX31825_CONV_TIME_12_BITS_MS 320

#define SELECT_ADDRESS_CONV_TIME_US 3000 

struct max31825_context_s
{
  struct device_onewire_s onewire;
  struct dev_onewire_rq_s onewire_rq;
  struct dev_onewire_transfer_s transfer[2];
  struct dev_valio_rq_s * current_user_rq;

  // Parameters collected from device tree
  uintptr_t init_charging_time_us;
  uintptr_t device_address;

  // Buffer used for commucation with the chip
  uint8_t tx_data[10];
  uint8_t rx_data[10];

  dev_request_queue_root_t queue;

  enum max31825_state_e state;
};

STRUCT_COMPOSE(max31825_context_s, onewire_rq);

DRIVER_PV(struct max31825_context_s);

static void handle_request(struct max31825_context_s *pv);


static void process_next_request(struct max31825_context_s *pv){
  pv->current_user_rq = dev_valio_rq_pop(&pv->queue);

  if(pv->current_user_rq){
    handle_request(pv);
  };
  
}

// For debug
static void print_sp(uint8_t rx_data[]){
  logk_trace("Scratchpad is:");
  logk_trace("Temperature LSB %x", rx_data[0]);
  logk_trace("Temperature MSB %x", rx_data[1]);
  logk_trace("Status [TH, TL state, address] %x", rx_data[2]);
  logk_trace("Configuration %x", rx_data[3]);
  logk_trace("TH MSB  %x", rx_data[4]);
  logk_trace("TH LSB  %x", rx_data[5]);
  logk_trace("TL LSB %x", rx_data[6]);
  logk_trace("TL MSB  %x", rx_data[7]);
  logk_trace("CRC %x", rx_data[8]);
  logk_trace("address  %x", rx_data[2] & 0x3f );
}



static uint8_t uint8_reverse(uint8_t val)
{
    uint8_t ret = 0;
    for (size_t i = 0; i < 8; i++)
    {
        if (val & 0x80)
        {
            ret |= (1 << i);
        }
        val <<= 1;
    }
    return ret;
}

// Special checksum calculation reversing input data and output CRC result
static uint8_t calculate_crc8_maxim(uint8_t const * data, size_t data_size, uint8_t poly)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < data_size; i++)
    {
        crc = crc ^ uint8_reverse(data[i]);
        for (size_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return uint8_reverse(crc);
}


static KROUTINE_EXEC(scratchpad_read_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct max31825_context_s *pv = max31825_context_s_from_onewire_rq(rq);

  print_sp(pv->rx_data);
  
  // Check that device address matches to requested
  uint8_t inspected_device_adress = pv->rx_data[2] & 0x3f;
  if(inspected_device_adress != pv->device_address){
    logk_error("Device address targeted : %x, got: %x",pv->device_address, inspected_device_adress);
    pv->current_user_rq->error = -EIO;
  }

  // Check data integrity by comparing device and calculated CRCs
  uint8_t device_crc = pv->rx_data[8];
  // Polynomial is "X8 + X5 + X4 + 1"
  uint8_t calculated_crc = calculate_crc8_maxim(pv->rx_data, 8, 0x31);
  if(device_crc != calculated_crc){
      logk_error("Wrong MAX31825 scratchpad CRC, calculated : %x, in device ROM: %x",calculated_crc, device_crc);
      pv->current_user_rq->error = -EIO;
  }


  // Process data if no error is present
  if(!pv->current_user_rq->error){

    int16_t raw_temp = ((pv->rx_data[1] << 8) | pv->rx_data[0]);

    uint32_t temp_milikelvin = (raw_temp*625)/10 + 273150;
    struct valio_temperature_s* rq_data = (struct valio_temperature_s*) pv->current_user_rq->data;
    
    logk_trace(" Temperature read is :  %d.%04d C or %d mK",
    raw_temp / 16, 625 * ((raw_temp < 0 ? 0x10 - (raw_temp & 0xf) : (raw_temp & 0xf))), temp_milikelvin);

    rq_data->temperature = temp_milikelvin;
  }

  dev_valio_rq_done(pv->current_user_rq);

  pv->state = MAX31825_IDLE;

  process_next_request(pv);
}

static void start_read_scratchpad(struct max31825_context_s *pv){

  logk_trace("Starting scratchpad read");
  pv->state = MAX31825_READING_TEMP;

  dev_onewire_rq_init(&pv->onewire_rq, scratchpad_read_done);

  pv->onewire_rq.data.transfer = pv->transfer;
  pv->onewire_rq.data.transfer_count = 2;
  pv->onewire_rq.type = DEV_ONEWIRE_RAW;
  pv->onewire_rq.delay_before_communication_us = 0;
  pv->onewire_rq.delay_after_communication_us = 0;

  pv->transfer[0].direction = DEV_ONEWIRE_WRITE;
  pv->tx_data[0] = SELECT_ADRESS_CMD;
  pv->tx_data[1] = pv->device_address;
  pv->tx_data[2] = READ_SCRATCHPAD_CMD;
  pv->transfer[0].data = pv->tx_data;
  pv->transfer[0].size = 3;
  pv->transfer[1].direction = DEV_ONEWIRE_READ;
  pv->transfer[1].data = pv->rx_data;
  pv->transfer[1].size = 9;
  memset(pv->rx_data, 0, pv->transfer[1].size);
  DEVICE_OP(&pv->onewire, request, &pv->onewire_rq);
}

static KROUTINE_EXEC(conversion_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct max31825_context_s *pv = max31825_context_s_from_onewire_rq(rq);
  start_read_scratchpad(pv);
}

static void start_temperature_request(struct max31825_context_s *pv)
{

  logk_trace("Starting temperature request");
  pv->state = MAX31825_CONVERTING_TEMP;

  dev_onewire_rq_init(&pv->onewire_rq, conversion_done);

  pv->onewire_rq.data.transfer = pv->transfer;
  pv->onewire_rq.data.transfer_count = 1;
  pv->onewire_rq.type = DEV_ONEWIRE_RAW;
  pv->onewire_rq.delay_before_communication_us = 0;
  pv->onewire_rq.delay_after_communication_us = MAX31825_CONV_TIME_12_BITS_MS*1000;

  pv->transfer[0].direction = DEV_ONEWIRE_WRITE;
  pv->tx_data[0] = SELECT_ADRESS_CMD;
  pv->tx_data[1] = pv->device_address;
  pv->tx_data[2] = CONVERT_T_CMD;
  pv->transfer[0].data = pv->tx_data;
  pv->transfer[0].size = 3;
  
  DEVICE_OP(&pv->onewire, request, &pv->onewire_rq);
}


static void handle_request(struct max31825_context_s *pv){

    // Only one possible request available
    start_temperature_request(pv);
}

static
DEV_VALIO_REQUEST(max31825_request)
{
  struct device_s *dev = accessor->dev;
  struct max31825_context_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (rq->type == DEVICE_VALIO_READ &&
  rq->attribute == VALIO_TEMPERATURE_VALUE){

    rq->error = 0;
    dev_valio_rq_pushback(&pv->queue, rq);

    if (pv->state == MAX31825_IDLE){
      process_next_request(pv);  
    }
  }
  else{
        rq->error = -ENOTSUP;
        dev_valio_rq_done(rq);
  }
}


#define max31825_use dev_use_generic


static KROUTINE_EXEC(detect_adress_done)
{
  struct dev_onewire_rq_s *rq = dev_onewire_rq_from_kr(kr);
  struct max31825_context_s *pv = max31825_context_s_from_onewire_rq(rq);
  process_next_request(pv);
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

  // Request all MAX31825 devices on the bus to measure the adress defined by
  // the resistance value on their ADD0 pin 
  dev_onewire_rq_init(&pv->onewire_rq, detect_adress_done);

  pv->onewire_rq.data.transfer = pv->transfer;
  pv->onewire_rq.data.transfer_count = 1;
  pv->onewire_rq.data.rom=0;
  pv->onewire_rq.type = DEV_ONEWIRE_DATA;
  pv->onewire_rq.delay_before_communication_us = pv->init_charging_time_us;
  pv->onewire_rq.delay_after_communication_us = SELECT_ADDRESS_CONV_TIME_US;

  pv->transfer[0].direction = DEV_ONEWIRE_WRITE;
  pv->tx_data[0] = DETECT_ADRESS_CMD;
  pv->transfer[0].data = pv->tx_data;
  pv->transfer[0].size = 1;

  DEVICE_OP(&pv->onewire, request, &pv->onewire_rq);
  
  return 0;
}

static
DEV_VALIO_CANCEL(max31825_cancel)
{
  return -ENOTSUP;
}

static DEV_CLEANUP(max31825_cleanup)
{
  struct max31825_context_s *pv = dev->drv_pv;

  if (!dev_rq_queue_isempty(&pv->queue))
    return -EBUSY;

  dev_rq_queue_destroy(&pv->queue);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(max31825_drv, 0, "max31825", max31825,
               DRIVER_VALIO_METHODS(max31825));

DRIVER_REGISTER(max31825_drv);

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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) 2018 Sebastien CERDAN <sebcerdan@gmail.com>

    This is a test application for rfpacket API. It tests the 
    REQUEST and CANCEL methods of a rfpacket driver. Each operation 
    is randomly generated and pushed on rfpacket driver.

    The RFP_TEST_SLEEP macro when enabled introduces a sleep delay
    after each end of request.

    The RFP_TEST_TX_FAIR macro when enabled allows generation of
    TX fair request.

    The RFP_TEST_RX_CONTINOUS when enabled macro allows generation
    of RX continous requests and CANCEL operations.
*/

#include <mutek/printk.h>
#include <mutek/kroutine.h>
#include <mutek/slab.h>

#include <hexo/decls.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <device/resources.h>
#include <device/class/timer.h>
#include <device/class/rfpacket.h>
#include <device/class/spi.h>
#include <arch/efm32/pin.h>

#define RFP_TEST_RQ_NUMBER 1
#define MAX_PACKET_SIZE 256
#define RFP_TEST_DEVICE_0_PATH "rfpacket"

#define TEST_BASE_TIME_MS 1000  /** us */

#define RFP_TEST_TX_FAIR
#define RFP_TEST_RX_CONTINOUS
#define RFP_TEST_SLEEP

#if (RFP_TEST_RQ_NUMBER == 1) && defined(RFP_TEST_RX_CONTINOUS)
#endif

//#define TX_TEST

static const struct dev_rfpacket_pk_cfg_basic_s pkcfg = {
    .base = {
        .format = DEV_RFPACKET_FMT_SLPC,
        .encoding = DEV_RFPACKET_CLEAR,
        .cache = 
          {
            .id = 0,
            .dirty = 0
          },
    },
    .crc = 0x8005,
    .crc_seed = 0xFFFF,
    .sw_value = 0xC7AE,
    .sw_len = 15,
    .pb_pattern = 0x2,
    .pb_pattern_len = 1,
    .tx_pb_len = 96,
    .rx_pb_len = 16,
};

static const struct dev_rfpacket_rf_cfg_fsk_s rfcfg = 
{
 .base =
   {
     .mod = DEV_RFPACKET_GFSK,
     .cache = 
       {
         .id = 0,
         .dirty = 0
       },
     .drate = 38400,
     .jam_rssi = (-90) << 3,  
     .frequency = 865046875,
     .chan_spacing = 93750,
     .rx_bw = 0,
     .freq_err = 868 * 20 /* ppm */,
   },
 .fairtx = 
   {
     .mode = DEV_RFPACKET_LBT,
     .lbt.rssi = (-95) << 3,
     .lbt.duration = 5000, /** us */
   },
 .deviation = 19200, 
 .symbols = 2,
};

struct rfp_test_rq_s
{
  struct dev_rfpacket_rq_s rq;
#ifdef RFP_TEST_SLEEP
  struct dev_timer_rq_s trq;
#endif
};
STRUCT_COMPOSE(rfp_test_rq_s, rq);
#ifdef RFP_TEST_SLEEP
STRUCT_COMPOSE(rfp_test_rq_s, trq);
#endif

struct rfp_test_pv_s
{
  struct device_rfpacket_s rfp;
  struct device_timer_s timer;
  struct device_spi_ctrl_s spi;
  struct device_gpio_s gpio;

  struct rfp_test_rq_s rq[RFP_TEST_RQ_NUMBER];

  struct dev_rfpacket_rq_s *rx_cont;

  struct dev_timer_rq_s timer_rq;
  struct dev_spi_ctrl_transfer_s  spi_tr;

  uint8_t txdata[MAX_PACKET_SIZE];

  uint32_t base_time;
};

static struct slab_s rxbuffer_slab;
static struct slab_s rx_slab;

static size_t rfp_test_grow_rxbuffer(struct slab_s *slab, size_t current)
{
  return 4;
}

static size_t rfp_test_grow_rx(struct slab_s *slab, size_t current)
{
  return 4;
}

struct rfp_test_rx_s
{
  struct rfp_test_pv_s *pv;
  struct dev_rfpacket_rx_s rx;
};

STRUCT_COMPOSE(rfp_test_rx_s, rx);

static KROUTINE_EXEC(rfp_test_rx_packet_callback)
{
  struct dev_rfpacket_rx_s *rx = dev_rfpacket_rx_s_from_kr(kr);
  struct rfp_test_rx_s *base = rfp_test_rx_s_from_rx(rx);

  if (!rx->error)
    {
      uint8_t  * p = (uint8_t *)rx->buf;
      printk("rx chan %d %P\n", rx->channel, p, rx->size);

      for (uint16_t i = 0; i< rx->size; i++)
        {
          if (p[i] != i)
            printk("rx error %d %d\n", i, p[i]);
        }
    }

  slab_free(&rxbuffer_slab, rx->buf);
  slab_free(&rx_slab, base);
}

static struct dev_rfpacket_rx_s *rfp_test_rx_alloc(struct dev_rfpacket_rq_s *rq, size_t size)
{
  struct rfp_test_pv_s *pv = rq->pvdata;

  if ((rand() % 16) == 0)
    return NULL;

  struct rfp_test_rx_s *base = (struct rfp_test_rx_s *)slab_alloc(&rx_slab);

  if (base == NULL)
    return NULL;

  base->pv = pv;
  struct dev_rfpacket_rx_s *rx = &base->rx;

  rx->buf = (uint8_t *)slab_alloc(&rxbuffer_slab);

  if (rx->buf == NULL)
    {
      slab_free(&rx_slab, base);
      return NULL;
    }

  kroutine_init_deferred(&rx->kr, &rfp_test_rx_packet_callback);
  rx->size = size;

  return rx;
}

static KROUTINE_EXEC(rfp_test_rx_callback)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);
  struct rfp_test_pv_s *pv = rq->pvdata;

  if (rq->error == -ENOTSUP)
    printk("Bad RX configuration\n");

  if (rq == pv->rx_cont)
    {
      pv->rx_cont = NULL;
      if (rq->error ==  -EBUSY)
          printk("Jamming \n");
    }
}

static void rfp_test_wait(struct rfp_test_pv_s *pv);
static KROUTINE_EXEC(rfp_test_wait_kr)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, base.kr);
  rfp_test_wait(trq->pvdata); 
}

static void rfp_test_wait(struct rfp_test_pv_s *pv)
{
  struct rfp_test_rq_s *base = pv->rq;
  struct dev_timer_rq_s *trq = &base->trq;

  trq->delay = pv->base_time;
  trq->rev = 0;
  trq->pvdata = pv;

  dev_timer_rq_init(trq, rfp_test_wait_kr);

  error_t err = DEVICE_OP(&pv->timer, request, trq);

  switch (err)
    {
    case 0:
      break;
    default:
      printk("err\n");
      break;
    }
}

static struct rfp_test_pv_s pv;

void spi_flash_deep_sleep(struct rfp_test_pv_s *pv)
{
    static const struct dev_spi_ctrl_config_s spi_cfg = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .cs_pol   = DEV_SPI_ACTIVE_LOW,
    .bit_rate1k = 6000000 >> 10,
    .word_width = 8,
  };

  ensure(DEVICE_OP(&pv->spi, config, &spi_cfg) == 0);

  uint8_t set = 0;
  uint8_t clear = 0;
  uint8_t mask = 0xFF;

  DEVICE_OP(&pv->gpio, set_mode, EFM32_PA4, EFM32_PA4, &mask, DEV_PIN_PUSHPULL);

  DEVICE_OP(&pv->gpio, set_output, EFM32_PA4, EFM32_PA4, &set, &clear);

  struct dev_spi_ctrl_transfer_s * tr = &pv->spi_tr;

  uint8_t cmd = 0xB9;

  tr->data.out_width = 1;
  tr->data.out = &cmd;
  tr->data.in_width = 0;
  tr->data.in = NULL;
  tr->data.count = 1;
 
  tr->cs_op = DEV_SPI_CS_NOP_NOP;

  ensure(dev_spi_wait_transfer(&pv->spi, tr) == 0);

  set = 1;
  clear = 1;

  DEVICE_OP(&pv->gpio, set_output, EFM32_PA4, EFM32_PA4, &set, &clear);
}

static void rfp_test_push_rx_ldc(struct rfp_test_pv_s *pv, struct dev_rfpacket_rq_s *rq)
{
  rq->err_group = 0;
  rq->pvdata = pv;
  rq->anchor = DEV_RFPACKET_TIMESTAMP_END;
  rq->pk_cfg = &pkcfg.base;
  rq->rf_cfg = &rfcfg.base;

  /* Start RX continous */
  rq->type = DEV_RFPACKET_RQ_RX_CONT;
  rq->channel = 24;
  rq->deadline = 0;
  rq->lifetime = 0;
  rq->rx_alloc = &rfp_test_rx_alloc;
  pv->rx_cont = rq;
  dev_rfpacket_rq_init(rq, &rfp_test_rx_callback);
  DEVICE_OP(&pv->rfp, request, rq, NULL);
}

void main(void)
{
  memset(&pv, 0, sizeof(pv));

  ensure(!device_get_accessor_by_path(&pv.rfp.base, NULL, RFP_TEST_DEVICE_0_PATH, DRIVER_CLASS_RFPACKET));
  ensure(!device_get_accessor_by_path(&pv.spi.base,  NULL, "spi", DRIVER_CLASS_SPI_CTRL));
  ensure(!device_get_accessor_by_path(&pv.gpio.base,  NULL, "gpio", DRIVER_CLASS_GPIO));

#if (CONFIG_EFM32_BOARD == pcb4001_xg12)  
  spi_flash_deep_sleep(&pv);
#endif

#ifdef CONFIG_DRIVER_EFM32_RTCC
  ensure(!device_get_accessor_by_path(&pv.timer.base,  NULL, "rtcc", DRIVER_CLASS_TIMER));
  dev_timer_init_sec(&pv.timer, &pv.base_time, 0, TEST_BASE_TIME_MS, 1000);
  return rfp_test_wait(&pv);
#endif

  slab_init(&rxbuffer_slab, MAX_PACKET_SIZE, &rfp_test_grow_rxbuffer, mem_scope_sys);
  slab_init(&rx_slab, sizeof(struct rfp_test_rx_s), &rfp_test_grow_rx, mem_scope_sys);

  rfp_test_push_rx_ldc(&pv, &pv.rq[0].rq);
}

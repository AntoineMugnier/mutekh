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

#define RFP_TEST_RQ_NUMBER 2
#define MAX_PACKET_SIZE 256
#define RFP_TEST_TIMER_PATH    "rfpacket0"
#define RFP_TEST_DEVICE_0_PATH "rfpacket0"

#define TEST_BASE_TIME_US 1000  /** us */

#define RFP_TEST_TX_FAIR
#define RFP_TEST_RX_CONTINOUS
#define RFP_TEST_SLEEP

#if (RFP_TEST_RQ_NUMBER == 1) && defined(RFP_TEST_RX_CONTINOUS)
#undef RFP_TEST_RX_CONTINOUS
#warning RFP_TEST_RX_CONTINOUS can not be defined when only one request in use
#endif

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
    .sw_value = 0xB42B,
    .sw_len = 15,
    .pb_pattern = 0x1,
    .pb_pattern_len = 1, 
    .tx_pb_len = 64,
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
     .frequency = 868000000,
     .chan_spacing = 100000,
     .rx_bw = 0,
     .freq_err = 868 * 20 /* ppm */,
   },
 .fairtx = 
   {
     .mode = DEV_RFPACKET_LBT,
     .lbt.rssi = (-95) << 3,
     .lbt.duration = 5000, /** us */
   },
 .deviation = 35000, 
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

  struct rfp_test_rq_s rq[RFP_TEST_RQ_NUMBER];

  struct dev_rfpacket_rq_s *rx_cont;

  struct dev_timer_rq_s timer_rq;

  uint8_t txdata[MAX_PACKET_SIZE];

  uint32_t base_time;
};

static struct slab_s rxbuffer_slab;
static struct slab_s rx_slab;

#ifdef RFP_TEST_SLEEP
static void rfp_test_wait_before_push(struct rfp_test_pv_s *pv, struct dev_rfpacket_rq_s *rq);
#endif
static void rfp_test_push_random_req(struct rfp_test_pv_s *pv, struct dev_rfpacket_rq_s *rq);

static size_t rfp_test_grow_rxbuffer(struct slab_s *slab, size_t current)
{
  return 4;
}

static size_t rfp_test_grow_rx(struct slab_s *slab, size_t current)
{
  return 4;
}

static uint64_t rfp_test_set_random_deadline(struct rfp_test_pv_s *pv)
{
#ifdef RFP_TEST_RX_CONTINOUS
  return 0;
#else
  dev_timer_value_t t;

  if (DEVICE_OP(&pv->timer, get_value, &t, 0))
    abort();

  return t + (rand()%32) * pv->base_time;
#endif
}

static uint32_t rfp_test_set_random_lifetime(struct rfp_test_pv_s *pv)
{
  return ((64 + rand()%0xFF) * pv->base_time);
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
  
#ifdef RFP_TEST_SLEEP
  return rfp_test_wait_before_push(pv, rq);
#else
  return rfp_test_push_random_req(pv, rq);
#endif
}

static inline uint8_t rfp_test_set_rand_size()
{
  return 1 + (rand() % (MAX_PACKET_SIZE - 2));
}

static int16_t rfp_test_set_rand_power()
{
  int16_t p = (16 - rand() % 16) << 3;
  return p;
}

static KROUTINE_EXEC(rfp_test_tx_callback)
{
  struct dev_rfpacket_rq_s *rq = dev_rfpacket_rq_from_kr(kr);
  struct rfp_test_pv_s *pv = rq->pvdata;

  if (rq->error == -ENOTSUP)
    printk("Bad TX configuration\n");

  if (rq->error == -ETIMEDOUT)
    printk("TX timeout\n");

#ifdef RFP_TEST_SLEEP
  return rfp_test_wait_before_push(pv, rq);
#else
  return rfp_test_push_random_req(pv, rq);
#endif
}

#ifdef RFP_TEST_SLEEP
static KROUTINE_EXEC(rfp_test_wait_before_push_cb)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, base.kr);
  struct rfp_test_rq_s *base = rfp_test_rq_s_from_trq(trq);
  rfp_test_push_random_req(trq->pvdata, &base->rq);
}

static void rfp_test_wait_before_push(struct rfp_test_pv_s *pv, struct dev_rfpacket_rq_s *rq)
{
  struct rfp_test_rq_s *base = rfp_test_rq_s_from_rq(rq);
  struct dev_timer_rq_s *trq = &base->trq;

  trq->delay = pv->base_time * 256;
  trq->rev = 0;
  trq->pvdata = pv;

  dev_timer_rq_init(trq, rfp_test_wait_before_push_cb);

  error_t err = DEVICE_OP(&pv->timer, request, trq);

  switch (err)
    {
    case -ETIMEDOUT:
      kroutine_exec(&trq->base.kr);
    case 0:
    default:
      break;
    }
}
#endif

static void rfp_test_push_random_req(struct rfp_test_pv_s *pv, struct dev_rfpacket_rq_s *rq)
{
  rq->err_group = 0;
  rq->pvdata = pv;
  rq->anchor = DEV_RFPACKET_TIMESTAMP_END;

  rq->pk_cfg = &pkcfg.base;
  rq->rf_cfg = &rfcfg.base;

  rq->channel = 0;

  uint8_t type =  rand() % 16;

  switch (type)
    {
    case 0 ... 1:
#ifdef RFP_TEST_RX_CONTINOUS
      /* Start RX continous */
      rq->type = DEV_RFPACKET_RQ_RX_CONT;
      rq->deadline = 0;
      rq->lifetime = 0;
      rq->rx_alloc = &rfp_test_rx_alloc;
      pv->rx_cont = rq;
      break;
#endif
    case 2 ... 3:
#ifdef RFP_TEST_RX_CONTINOUS
      /* Cancel RX continous */
      if (pv->rx_cont && DEVICE_OP(&pv->rfp, cancel, pv->rx_cont) == 0)
        dev_rfpacket_rq_done(pv->rx_cont);
#endif
    case 4 ... 7:
#ifdef RFP_TEST_TX_FAIR
      rq->type = DEV_RFPACKET_RQ_TX_FAIR;
      rq->deadline = rfp_test_set_random_deadline(pv);
      rq->lifetime = 8 * pv->base_time;
      rq->tx_size = rfp_test_set_rand_size();
      rq->tx_buf = pv->txdata;
      rq->tx_pwr = rfp_test_set_rand_power();
      break;
#endif
    case 8 ... 11:
      rq->type = DEV_RFPACKET_RQ_TX;
      rq->deadline = rfp_test_set_random_deadline(pv);
      rq->lifetime = 0;
      rq->tx_size = rfp_test_set_rand_size();
      rq->tx_buf = pv->txdata;
      rq->tx_pwr = rfp_test_set_rand_power();
      break;
    case 12 ... 15:
      rq->type = DEV_RFPACKET_RQ_RX;
      rq->deadline = rfp_test_set_random_deadline(pv);
      rq->lifetime = rfp_test_set_random_lifetime(pv);
      rq->rx_alloc = &rfp_test_rx_alloc;
      break;
    }

  switch (rq->type)
    {
    case DEV_RFPACKET_RQ_TX:
#ifdef RFP_TEST_TX_FAIR
    case DEV_RFPACKET_RQ_TX_FAIR:
#endif
      dev_rfpacket_rq_init(rq, &rfp_test_tx_callback);
      break;
    case DEV_RFPACKET_RQ_RX:
    case DEV_RFPACKET_RQ_RX_CONT:
      dev_rfpacket_rq_init(rq, &rfp_test_rx_callback);
      break;
    }

  DEVICE_OP(&pv->rfp, request, rq, NULL);
}


static struct rfp_test_pv_s pv;

void app_start(void)
{
  memset(&pv, 0, sizeof(pv));

  ensure(!device_get_accessor_by_path(&pv.rfp.base, NULL, RFP_TEST_DEVICE_0_PATH, DRIVER_CLASS_RFPACKET));
  ensure(!device_get_accessor_by_path(&pv.timer.base,  NULL, RFP_TEST_TIMER_PATH, DRIVER_CLASS_TIMER));

  dev_timer_init_sec(&pv.timer, &pv.base_time, 0, TEST_BASE_TIME_US, 1000000);

  for(uint16_t i = 0; i < MAX_PACKET_SIZE; i++)
    pv.txdata[i] = i;

  slab_init(&rxbuffer_slab, MAX_PACKET_SIZE, &rfp_test_grow_rxbuffer, mem_scope_sys);
  slab_init(&rx_slab, sizeof(struct rfp_test_rx_s), &rfp_test_grow_rx, mem_scope_sys);

  for (uint8_t i = 0; i< RFP_TEST_RQ_NUMBER; i++)
    rfp_test_push_random_req(&pv, &pv.rq[i].rq);
}

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

    This is a basic test that can be used to validate READ/WRITE
    requests type on an UART driver. This test involves two boards.

    The UART TX line of the first board is connected to the UART RX
    line of the second board.

    The UART RX line of the first board is connected to the UART TX
    line of the second board.

    At startup, the first RX is deffered in order to be sure that 
    startup of the other board does not generate glitches on RX line.

    This test implements a flow control method. Indeed, in this test,
    each board informs the other board of the amount of bytes it is
    able to receive. Thus the other board send the corresponding
    amount of data plus the next amount of bytes it is able to
    receive.
*/



#include <mutek/printk.h>
#include <mutek/kroutine.h>
#include <hexo/endian.h>
#include <hexo/decls.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/class/iomux.h>
#include <device/class/cmu.h>
#include <device/class/char.h>

#define TEST_MAX_VALUE   200
#define CHAR_TEST_BUFFER_SIZE 256
#define TEST_UART_DEV "uart*"
#define TEST_TIMER_DEV "timer"

enum chat_test_e
{
  CHAR_TEST_IDLE,
  CHAR_TEST_WAIT,
  CHAR_TEST_WAIT_READ,
  CHAR_TEST_WAIT_WRITE,
};

struct char_test_pv_s
{
  struct dev_timer_rq_s trq;
  struct device_timer_s timer;

  struct device_char_s cdev;
  struct dev_char_rq_s wrq;
  struct dev_char_rq_s rrq;

  uint8_t rdata[CHAR_TEST_BUFFER_SIZE];
  uint8_t wdata[CHAR_TEST_BUFFER_SIZE];

  uint8_t rlast;
  uint8_t wlast;
  uint16_t rsize;
  uint16_t wsize;
  uint16_t cnt;
  enum chat_test_e state;
};
STRUCT_COMPOSE(char_test_pv_s, trq);

static void char_test_start_write(struct char_test_pv_s *pv);
static void char_test_start_read(struct char_test_pv_s *pv);

static void char_set_data(struct char_test_pv_s * pv, size_t wsize)
{
  uint8_t *p = pv->wdata;

  /* Set next receive size */
  *p++ = pv->rsize >> 8;
  *p++ = pv->rsize;

  size_t seed = pv->wlast;

  while (wsize--)
  {
    *p++ = seed;
    seed = (seed + 1) % TEST_MAX_VALUE;
  }

  pv->wlast = seed;
}

static bool_t char_check_data(struct char_test_pv_s * pv)
{
  uint8_t *p = pv->rdata + 2;

  uint32_t ref = pv->rlast;

  if ((pv->cnt++ % 128) == 0)
    writek(".", 1);

  for (uint32_t i = 0; i < pv->rsize - 2; i++)
  {
    if (p[i] != ref)
    {
      printk("Read error data[%d] = 0x%x ref = 0x%x\n", i, p[i], ref);
      abort();
    }

    ref = (ref + 1) % TEST_MAX_VALUE;
  }

  pv->rlast = ref;
  return 0;
}

static void char_test_cycle(struct char_test_pv_s *pv)
{
  assert(pv->state == CHAR_TEST_IDLE);

  pv->state = CHAR_TEST_WAIT;
  /* Post RX request */
  char_test_start_read(pv);
  /* Post TX request */
  char_test_start_write(pv);
}

static KROUTINE_EXEC(char_test_tx_callback)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct char_test_pv_s *pv = rq->pvdata;

  if (rq->error)
    printk("TX error %d\n", rq->error);

  switch (pv->state)
    {
      case CHAR_TEST_WAIT:
        pv->state = CHAR_TEST_WAIT_READ;
        break;
      case CHAR_TEST_WAIT_WRITE:
        pv->state = CHAR_TEST_IDLE;
        char_test_cycle(pv);
        break;
      default:
        abort();
    }
}

static KROUTINE_EXEC(char_test_rx_callback)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct char_test_pv_s *pv = rq->pvdata;

  if (rq->error)
    printk("RX error %d\n", rq->error);

  assert(rq->size == 0);

  /* Check receive data */
  char_check_data(pv);

  /* Lenght of next TX including next RX size */
  pv->wsize = pv->rdata[0] << 8 | pv->rdata[1];
  pv->rsize = 2 + rand() % (CHAR_TEST_BUFFER_SIZE - 2);

  if (pv->wsize < 2 || pv->wsize > CHAR_TEST_BUFFER_SIZE)
    assert(0);

  switch (pv->state)
    {
      case CHAR_TEST_WAIT:
        pv->state = CHAR_TEST_WAIT_WRITE;
        break;
      case CHAR_TEST_WAIT_READ:
        pv->state = CHAR_TEST_IDLE;
        char_test_cycle(pv);
        break;
      default:
        abort();
    }
}

static void char_test_start_write(struct char_test_pv_s *pv)
{
  struct dev_char_rq_s *rq = &pv->wrq;

  char_set_data(pv, pv->wsize - 2);

  rq->type = DEV_CHAR_WRITE;
  rq->pvdata = pv;
  rq->size = pv->wsize;
  rq->data = pv->wdata;

  dev_char_rq_init(rq, &char_test_tx_callback);

  DEVICE_OP(&pv->cdev, request, rq);

}
static void char_test_start_read(struct char_test_pv_s *pv)
{
  struct dev_char_rq_s *rq = &pv->rrq;

  rq->type = DEV_CHAR_READ;
  rq->pvdata = pv;
  rq->size = pv->rsize;
  rq->data = pv->rdata;

  dev_char_rq_init(rq, &char_test_rx_callback);
  DEVICE_OP(&pv->cdev, request, rq);
}

static struct char_test_pv_s pv;

/* The delay is elapsed. We consider that the other board is ready to RX */
static KROUTINE_EXEC(delay_elapsed)
{
  struct dev_timer_rq_s *trq = KROUTINE_CONTAINER(kr, *trq, base.kr);
  struct char_test_pv_s *pv = char_test_pv_s_from_trq(trq);

  /* Lenght of next RX */
  pv->rsize = 2;
  pv->wsize = 2;

  char_test_cycle(pv);
}

/* Start the timer */
static void wait_before_start(struct char_test_pv_s *pv)
{
  struct dev_timer_rq_s *trq = &pv->trq;

  trq->rev = 0;
  trq->pvdata = pv;

  dev_timer_init_sec(&pv->timer, &trq->delay, 0, 2, 1);

  /* Kroutine called when delay is elapsed */ 
  dev_timer_rq_init(trq, delay_elapsed);
  ensure(DEVICE_OP(&pv->timer, request, trq) == 0);
}

void app_start(void)
{
  memset(&pv, 0, sizeof(pv));

  ensure(!device_get_accessor_by_path(&pv.cdev.base, NULL, TEST_UART_DEV, DRIVER_CLASS_CHAR));
  ensure(!device_get_accessor_by_path(&pv.timer.base, NULL, TEST_TIMER_DEV, DRIVER_CLASS_TIMER));

  for (uint32_t i = 0; i < CHAR_TEST_BUFFER_SIZE; i++)
    pv.wdata[i] = 0xAA;

  /* This is used to deffer test start in order to be sure that the other board is
   * ready */
  wait_before_start(&pv);
}

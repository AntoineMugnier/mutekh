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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014-2015
*/

#include <mutek/buffer_pool.h>

#include <net/task.h>
#include <net/scheduler.h>
#include <net/layer.h>

#include <device/class/cmu.h>

#include <arch/nrf5x/ppi.h>
#include <arch/nrf5x/radio.h>
#include <arch/nrf5x/timer.h>
#include <arch/nrf5x/rtc.h>
#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/uart.h>
#include <arch/nrf5x/clock.h>
#include <arch/nrf5x/power.h>

#include <string.h>

#include <ble/protocol/radio.h>

#include "ble.h"
#include "ble_debug.h"
#include "ble_trx_gpio.h"

void nrf5x_ble_rtc_start(struct nrf5x_ble_private_s *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_gate(&pv->clock_sink[NRF5X_BLE_CLK_SLEEP], DEV_CLOCK_EP_CLOCK);
  dev_clock_sink_throttle(&pv->clock_sink[NRF5X_BLE_CLK_SLEEP], NRF5X_BLE_MODE_WAIT);
#endif

  nrf5x_ble_backlog(pv->current, "RTC start", 0);

  nrf_it_disable_mask(BLE_RTC_ADDR, -1);
  nrf_it_enable(BLE_RTC_ADDR, NRF_RTC_OVERFLW);
  nrf_evt_disable_mask(BLE_RTC_ADDR, -1);
  nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_START);
}

void nrf5x_ble_rtc_stop(struct nrf5x_ble_private_s *pv)
{
#if defined(CONFIG_DEVICE_CLOCK)
  dev_clock_sink_gate(&pv->clock_sink[NRF5X_BLE_CLK_SLEEP], DEV_CLOCK_EP_NONE);
  dev_clock_sink_throttle(&pv->clock_sink[NRF5X_BLE_CLK_SLEEP], NRF5X_BLE_MODE_IDLE);
#endif

  nrf5x_ble_backlog(pv->current, "RTC stop", 0);

  nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_STOP);
  nrf_it_disable_mask(BLE_RTC_ADDR, -1);
  nrf_evt_disable_mask(BLE_RTC_ADDR, -1);
}

dev_timer_value_t nrf5x_ble_rtc_value_get(struct nrf5x_ble_private_s *pv)
{
  uint32_t counter;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  counter = nrf_reg_get(BLE_RTC_ADDR, NRF_RTC_COUNTER);
  if (!(counter & 0x800000)
      && nrf_event_check(BLE_RTC_ADDR, NRF_RTC_OVERFLW))
    counter += 0x1000000;
  CPU_INTERRUPT_RESTORESTATE;

  return pv->base + counter;
}

void nrf5x_ble_rtc_boundary_clear(void)
{
  nrf_it_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
  nrf_ppi_disable(PPI_RTC_REQUEST_END_DISABLE);
  nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
}

void nrf5x_ble_rtc_boundary_set(dev_timer_value_t value, bool_t stop)
{
  nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_CC(RTC_REQUEST_BOUNDARY), value);
  nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
  nrf_it_enable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
  if (stop)
    nrf_ppi_enable(PPI_RTC_REQUEST_END_DISABLE);
}

void nrf5x_ble_radio_init(void)
{
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_POWER, 0);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_POWER, 1);

  for (uint16_t i = 0; i < 32; ++i)
    asm volatile("");

  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_DCDCEN,
              _CONFIG_NRF5X_RADIO_DCDC ? NRF_POWER_DCDCEN_ENABLE : 0);

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);

  if ((~cpu_mem_read_32(NRF_FICR_OVERRIDEEN)) & (1 << NRF_FICR_OVERRIDE_BLE_1MBIT)) {
    for (uint8_t i = 0; i < 5; ++i)
      nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_OVERRIDE(i),
                  cpu_mem_read_32(NRF_FICR_OVERRIDE(BLE_1MBIT, i)));
  }

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TXPOWER, 0);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_CRCPOLY, 0x100065B);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_CRCCNF, 0
              | NRF_RADIO_CRCCNF_SKIPADDR
              | (3 << NRF_RADIO_CRCCNF_LEN_OFFSET));
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TIFS, BLE_T_IFS);

  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_STOP);
  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_DISABLE);
  nrf5x_ble_trx_gpio_off();
}

void nrf5x_ble_rtc_init(void)
{
    nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_STOP);
    nrf_reg_set(BLE_RTC_ADDR, NRF_RTC_PRESCALER, 0);
    nrf_task_trigger(BLE_RTC_ADDR, NRF_RTC_CLEAR);
}

void nrf5x_ble_timer_init(void)
{
  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_PRESCALER, 4);
  nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_MODE, NRF_TIMER_MODE_TIMER);
  nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_BITMODE, NRF_TIMER_BITMODE_16);
  nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_IFS_TIMEOUT),
              BLE_T_IFS + BLE_PACKET_TIME(0)
              + RADIO_IRQ_LATENCY_US + RADIO_RX_CHAIN_DELAY_US);
  nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_PIPELINE_RESET), 3);
}

void nrf5x_ble_ppi_init(void)
{
  nrf_ppi_setup(PPI_RTC_REQUEST_END_DISABLE,
                BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY),
                BLE_RADIO_ADDR, NRF_RADIO_DISABLE);

  nrf_ppi_setup(PPI_RTC_MATCH_START,
                  BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START),
                  BLE_RADIO_ADDR, NRF_RADIO_START);

  nrf_ppi_setup(PPI_END_TIMER_START,
                BLE_RADIO_ADDR, NRF_RADIO_END,
                BLE_TIMER_ADDR, NRF_TIMER_START);

  nrf_ppi_setup(PPI_ADDRESS_TIMER_STOP,
                BLE_RADIO_ADDR, NRF_RADIO_ADDRESS,
                BLE_TIMER_ADDR, NRF_TIMER_STOP);

  nrf_ppi_setup(PPI_TIMER_IFS_RADIO_START,
                BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_IFS_START),
                BLE_RADIO_ADDR, NRF_RADIO_START);

  nrf_ppi_setup(PPI_END_PIPELINE_RESET,
                BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_PIPELINE_RESET),
                NRF_PPI_ADDR, NRF_PPI_CHG_DIS(PPI_GROUP_PIPELINE_RESET));

  nrf_reg_set(NRF_PPI_ADDR, NRF_PPI_CHG(PPI_GROUP_PIPELINE_RESET), 0
              | (1 << PPI_END_TIMER_START)
              | (1 << PPI_TIMER_IFS_RADIO_START));
}

void nrf5x_ble_ppi_cleanup(struct nrf5x_ble_private_s *pv)
{
  nrf_ppi_disable_mask(0
                       | (1 << PPI_RTC_MATCH_START)
                       | (1 << PPI_RTC_ENABLE_RXEN)
                       | (1 << PPI_RTC_ENABLE_TXEN)
                       | (1 << PPI_RTC_REQUEST_END_DISABLE)
                       );

  nrf_evt_disable_mask(BLE_RTC_ADDR, 0
                       | (1 << NRF_RTC_COMPARE(RTC_ENABLE))
                       | (1 << NRF_RTC_COMPARE(RTC_START))
                       );

  nrf_it_disable(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
}

void nrf5x_ble_radio_disable(struct nrf5x_ble_private_s *pv)
{
  pv->pipelining = 0;
  pv->current_params.channel = -1;

  nrf_it_disable_mask(BLE_RADIO_ADDR, -1);
  nrf_short_set(BLE_RADIO_ADDR, 0);
  pv->wait_end = 0;

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_READY);

  nrf_ppi_disable_mask(0
                       | (1 << PPI_RTC_MATCH_START)
                       | (1 << PPI_RTC_ENABLE_RXEN)
                       | (1 << PPI_RTC_ENABLE_TXEN)
                       | (1 << PPI_TIMER_IFS_RADIO_START)
                       | (1 << PPI_RTC_REQUEST_END_DISABLE)
                       | (1 << PPI_END_TIMER_START)
                       | (1 << PPI_ADDRESS_TIMER_STOP)
                       );

  nrf_evt_disable_mask(BLE_RTC_ADDR, 0
                       | (1 << NRF_RTC_COMPARE(RTC_ENABLE))
                       | (1 << NRF_RTC_COMPARE(RTC_START))
                       );

  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_STOP);
  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_RSSISTOP);
  nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_DISABLE);
  while (nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE)
         != NRF_RADIO_STATE_DISABLED);
  gpio(I_ENABLE | I_TRANSFER | I_TX | I_PIPELINE, I_ENABLE | I_TRANSFER | I_TX | I_PIPELINE);
  gpio(I_ENABLE | I_TRANSFER | I_TX | I_PIPELINE, 0);

  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_END);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BCC, 16);
  nrf5x_ble_trx_gpio_off();
}

error_t nrf5x_ble_data_setup(struct nrf5x_ble_private_s *pv)
{
  assert(pv->current);
  assert(!pv->transmitting);

  pv->transmitting = pv->current->handler->payload_get(pv->current, pv->current_params.mode);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PACKETPTR, (uintptr_t)pv->transmitting);

  nrf5x_ble_backlog(pv->current, "data setup: %p", (uint32_t)pv->transmitting);

  assert(!(nrf_reg_get(NRF_PPI_ADDR, NRF_PPI_CHEN) & (1 << PPI_TIMER_IFS_RADIO_START)));
  assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) != NRF_RADIO_STATE_RX
         && nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) != NRF_RADIO_STATE_TX);

  if (!pv->transmitting)
    return -ENOMEM;

  nrf_it_disable(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);
  nrf_short_enable(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS_BCSTART);

  if (pv->current_params.rx_rssi) {
    nrf_short_enable_mask(BLE_RADIO_ADDR, 0
                          | bit(NRF_RADIO_ADDRESS_RSSISTART)
                          | bit(NRF_RADIO_DISABLED_RSSISTOP));
  }

  nrf_it_enable_mask(BLE_RADIO_ADDR, 0
                     | (1 << NRF_RADIO_ADDRESS)
                     | (1 << NRF_RADIO_END)
                     );
  nrf_ppi_enable(PPI_END_PIPELINE_RESET);

  return 0;
}

static
void nrf5x_ble_radio_channel_set(
    const struct nrf5x_ble_params_s *params)
{
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_FREQUENCY,
              ble_channel_freq_mhz(params->channel) - 2400);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_DATAWHITEIV, params->channel | 0x40);
}

void nrf5x_ble_config_init(const struct nrf5x_ble_params_s *params)
{
  // Packet available space, except header and decryption prefix
  size_t packet_max_size = params->mode == MODE_RX ? CONFIG_BLE_PACKET_SIZE - 3 : 255;
  uint32_t pcnf0 = 0
    | (8 << NRF_RADIO_PCNF0_LFLEN_OFFSET)
    | (1 << NRF_RADIO_PCNF0_S0LEN_OFFSET)
    ;
  uint8_t radio_mode = NRF_RADIO_MODE_BLE_1MBIT;

  switch (params->phy) {
#if defined(NRF5X_BLE_PHY_2M)
  case BLE_PHY_2M:
    radio_mode = NRF_RADIO_MODE_BLE_2MBIT;
    pcnf0 |= NRF_RADIO_PCNF0_PLEN_16;
    break;
#endif
#if defined(NRF5X_BLE_PHY_LR)
  case BLE_PHY_CODED8:
    radio_mode = NRF_RADIO_MODE_BLE_LR125K;
    pcnf0 |= NRF_RADIO_PCNF0_PLEN_LR;
    pcnf0 |= NRF_RADIO_PCNF0_CILEN(2);
    pcnf0 |= NRF_RADIO_PCNF0_TERMLEN(3);
    break;
  case BLE_PHY_CODED2:
    radio_mode = NRF_RADIO_MODE_BLE_LR500K;
    pcnf0 |= NRF_RADIO_PCNF0_PLEN_LR;
    pcnf0 |= NRF_RADIO_PCNF0_CILEN(2);
    pcnf0 |= NRF_RADIO_PCNF0_TERMLEN(3);
    break;
#endif
  default:
  case BLE_PHY_1M:
    radio_mode = NRF_RADIO_MODE_BLE_1MBIT;
    pcnf0 |= NRF_RADIO_PCNF0_PLEN_8;
    break;
  }

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TXPOWER, params->tx_power / 8);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PCNF0, pcnf0);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PCNF1, 0
              | (packet_max_size << NRF_RADIO_PCNF1_MAXLEN_OFFSET)
              | (3 << NRF_RADIO_PCNF1_BALEN_OFFSET)
              | NRF_RADIO_PCNF1_ENDIAN_LITTLE
              | (params->whitening ? NRF_RADIO_PCNF1_WHITEEN_ENABLED : 0)
              );

  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_BASE0, params->access << 8);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_PREFIX0, params->access >> 24);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_TXADDRESS, 0);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_RXADDRESSES, 1 << 0);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_CRCINIT, params->crc_init);
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODE, radio_mode);

  nrf5x_ble_radio_channel_set(params);
}

extern inline
bool_t nrf5x_ble_phy_is_supported(enum ble_phy_mode_e mode);

bool_t nrf5x_ble_pipelined_setup(struct nrf5x_ble_private_s *pv)
{
  const struct nrf5x_ble_params_s *params = &pv->next_params;

  nrf5x_ble_radio_channel_set(params);

  pv->pipelining = 1;

  gpio(I_PIPELINE, I_PIPELINE);

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODECNF0, 0
    | NRF_RADIO_MODECNF0_RU_NORMAL
    | NRF_RADIO_MODECNF0_DTX_B1);
#endif

  nrf_short_disable(BLE_RADIO_ADDR, NRF_RADIO_READY_START);
  nrf_ppi_disable(PPI_TIMER_IFS_RADIO_START);

  nrf_ppi_enable_mask(0
                      | (1 << PPI_END_TIMER_START)
                      | (1 << PPI_ADDRESS_TIMER_STOP)
                      );
  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_STOP);
  nrf_task_trigger(BLE_TIMER_ADDR, NRF_TIMER_CLEAR);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_READY);

  nrf5x_ble_backlog(pv->current, "pipeline setup %d", params->mode);

  switch (params->mode) {
  case MODE_TX:
    nrf_short_set(BLE_RADIO_ADDR, 0
                  | (1 << NRF_RADIO_END_DISABLE)
                  | (1 << NRF_RADIO_DISABLED_TXEN));
    nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_IFS_START),
                BLE_T_IFS - RADIO_TX_CHAIN_DELAY_US - 2 * PPI_LATENCY_US - 1);
    break;

  default:
  case MODE_RX:
    nrf_short_set(BLE_RADIO_ADDR, 0
                  | (1 << NRF_RADIO_END_DISABLE)
                  | (1 << NRF_RADIO_DISABLED_RXEN));

    nrf_it_enable(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
    nrf_event_clear(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));
    nrf_reg_set(BLE_TIMER_ADDR, NRF_TIMER_CC(TIMER_IFS_START),
                BLE_T_IFS - RADIO_RX_CHAIN_DELAY_US - 2 * PPI_LATENCY_US - 3);
    break;
  }

  // If we ended already, we may not have pipelined in time
  return nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END);
}

void nrf5x_ble_pipelined_commit(struct nrf5x_ble_private_s *pv)
{
  assert(!(nrf_reg_get(NRF_PPI_ADDR, NRF_PPI_CHEN) & (1 << PPI_TIMER_IFS_RADIO_START)));
  nrf_ppi_enable(PPI_TIMER_IFS_RADIO_START);
  gpio(I_PIPELINE, 0);
  gpio(I_PIPELINE, I_PIPELINE);
  gpio(I_PIPELINE, 0);

  nrf5x_ble_backlog(pv->current, "pipeline commit %d", pv->pipelining_race);

  pv->pipelining = 0;
  pv->pipelining_race = 0;
}

void nrf5x_ble_pipelined_reset(struct nrf5x_ble_private_s *pv)
{
  gpio(I_PIPELINE, 0);

  nrf5x_ble_backlog(pv->current, "pipeline reset %d", pv->current_params.mode);

  pv->pipelining = 0;

  nrf5x_ble_radio_disable(pv);
  assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_STATE) == NRF_RADIO_STATE_DISABLED);

  pv->current_params = pv->next_params;
  nrf5x_ble_radio_channel_set(&pv->current_params);

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
  nrf_reg_set(BLE_RADIO_ADDR, NRF_RADIO_MODECNF0, 0
    | NRF_RADIO_MODECNF0_RU_FAST
    | NRF_RADIO_MODECNF0_DTX_B1);
#endif

  nrf_short_enable(BLE_RADIO_ADDR, NRF_RADIO_READY_START);
  nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_READY);
  nrf_ppi_disable(PPI_TIMER_IFS_RADIO_START);

  switch (pv->current_params.mode) {
  case MODE_TX:
    nrf5x_ble_trx_gpio_tx();
    nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_TXEN);
    break;
  case MODE_RX:
    nrf5x_ble_trx_gpio_rx();
    nrf_task_trigger(BLE_RADIO_ADDR, NRF_RADIO_RXEN);
    break;
  }
  gpio(I_ENABLE, I_ENABLE);

  pv->pipelining_race = 0;
}

#if defined(CONFIG_DEVICE_CLOCK)

bool_t nrf5x_ble_clock_request(struct nrf5x_ble_private_s *pv)
{
  gpio(I_CLOCK_REQ, I_CLOCK_REQ);
  dev_clock_sink_throttle(&pv->clock_sink[NRF5X_BLE_CLK_RADIO], NRF5X_BLE_MODE_RADIO);

  return pv->hfclk_is_precise;
}

void nrf5x_ble_clock_release(struct nrf5x_ble_private_s *pv)
{
  gpio(I_CLOCK_REQ, 0);
  gpio(I_CLOCK_RUN, 0);
  dev_clock_sink_throttle(&pv->clock_sink[NRF5X_BLE_CLK_RADIO], NRF5X_BLE_MODE_IDLE);
}

#endif

DEV_IRQ_SRC_PROCESS(nrf5x_ble_radio_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_ble_private_s *pv = dev->drv_pv;

  gpio(I_IRQ, I_IRQ_RADIO);

  if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS)) {
    nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS);

    nrf5x_ble_event_address_matched(pv);
  }

  if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH)
      && nrf_it_is_enabled(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH)) {
    nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_BCMATCH);

    nrf5x_ble_event_bcc_end(pv);
  }

  if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_END)
      && nrf_it_is_enabled(BLE_RADIO_ADDR, NRF_RADIO_END)) {
    //    assert(nrf_reg_get(BLE_RADIO_ADDR, NRF_RADIO_BCC) != 16);
    nrf5x_ble_event_packet_ended(pv);
  }

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
  for (uint8_t i = 0; i < 32; ++i) {
    if (!nrf_it_is_enabled(BLE_RADIO_ADDR, i) && nrf_event_check(BLE_RADIO_ADDR, i))
      nrf_event_clear(BLE_RADIO_ADDR, i);
  }
#endif

  gpio(I_IRQ, 0);
}

DEV_IRQ_SRC_PROCESS(nrf5x_ble_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_ble_private_s *pv = dev->drv_pv;

  gpio(I_IRQ, I_IRQ_TIMER);

  if (nrf_event_check(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT))) {
    nrf_event_clear(BLE_TIMER_ADDR, NRF_TIMER_COMPARE(TIMER_IFS_TIMEOUT));

    if (nrf_event_check(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS)) {
      nrf_event_clear(BLE_RADIO_ADDR, NRF_RADIO_ADDRESS);

      nrf5x_ble_event_address_matched(pv);
    } else {
      nrf5x_ble_event_ifs_timeout(pv);
    }
  }

  gpio(I_IRQ, 0);
}

DEV_IRQ_SRC_PROCESS(nrf5x_ble_rtc_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf5x_ble_private_s *pv = dev->drv_pv;

  gpio(I_IRQ, I_IRQ_RTC);

  if (nrf_event_check(BLE_RTC_ADDR, NRF_RTC_OVERFLW)) {
    nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_OVERFLW);

    pv->base += 1<<24;
  }

  /* if (nrf_event_check(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START)) */
  /*     && nrf_it_is_enabled(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START))) { */
  /*   nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START)); */
  /*   nrf_it_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_START)); */

  /*   ble_radio_first_schedule(pv); */
  /* } */

  if (nrf_event_check(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY))) {
    nrf_event_clear(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));
    nrf_it_disable(BLE_RTC_ADDR, NRF_RTC_COMPARE(RTC_REQUEST_BOUNDARY));

    if (!pv->current)
      kroutine_exec(&pv->rescheduler);
    else if (pv->event_end)
      nrf5x_ble_event_timeout(pv);
  }

  gpio(I_IRQ, 0);
}

void nrf5x_ble_addr_get(struct ble_addr_s *addr)
{
  addr->type = cpu_mem_read_32(NRF_FICR_DEVICEADDRTYPE) & 1 ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
  memcpy(addr->addr, (void*)NRF_FICR_DEVICEADDR(0), 6);
  if (addr->type == BLE_ADDR_RANDOM)
    ble_addr_random_type_set(addr, BLE_ADDR_RANDOM_STATIC);
}

uint8_t nrf5x_ble_tx_power_normalize(int16_t power)
{
#if CONFIG_NRF5X_MODEL == 52833 || CONFIG_NRF5X_MODEL == 52840
  if (power >= 8 * 8)
    return 8;
  if (power >= 7 * 8)
    return 7;
  if (power >= 6 * 8)
    return 6;
  if (power >= 5 * 8)
    return 5;
#endif
  if (power >= 4 * 8)
    return 4;
#if CONFIG_NRF5X_MODEL >= 52000
  if (power >= 3 * 8)
    return 3;
#endif
#if CONFIG_NRF5X_MODEL == 52833 || CONFIG_NRF5X_MODEL == 52840
  if (power >= 2 * 8)
    return 2;
#endif
  if (power >= 0 * 8)
    return 0;
  if (power >= -4 * 8)
    return 0xfc;
  if (power >= -8 * 8)
    return 0xf8;
  if (power >= -12 * 8)
    return 0xf4;
  if (power >= -16 * 8)
    return 0xf0;
  if (power >= -20 * 8)
    return 0xec;
  return 0xd8;
}

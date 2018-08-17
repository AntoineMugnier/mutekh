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

    Copyright (c) 2016, Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/pcm.h>
#include <device/class/iomux.h>
#ifdef CONFIG_DRIVER_NRF52_PDM_POWERGATE
#include <device/clock.h>
#endif

#include <arch/nrf5x/pdm.h>

#define PDM_ADDR NRF_PERIPHERAL_ADDR(NRF5X_PDM)

struct nrf52_pdm_pv_s
{
  struct dev_pcm_rq_s *rq;

  struct dev_irq_src_s irq_ep;

#ifdef CONFIG_DRIVER_NRF52_PDM_POWERGATE
  struct dev_clock_sink_ep_s power_source;
#endif

  struct kroutine_s deleter;
};

DRIVER_PV(struct nrf52_pdm_pv_s);

static KROUTINE_EXEC(nrf52_pdm_ended)
{
  struct nrf52_pdm_pv_s *pv = KROUTINE_CONTAINER(kr, *pv, deleter);
  struct dev_pcm_rq_s *rq = pv->rq;

  if (!rq)
    return;

#ifdef CONFIG_DRIVER_NRF52_PDM_POWERGATE
  dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_NONE);
#endif

  logk_trace("%s\n", __FUNCTION__);

  mem_free(rq->stream[0].buffer[0]);
  pv->rq = NULL;

  rq->error = 0;

  kroutine_exec_flags(&rq->base.kr, DEV_PCM_END_FLAG);
}

static DEV_IRQ_SRC_PROCESS(nrf52_pdm_irq)
{
  struct device_s *dev = ep->base.dev;
  struct nrf52_pdm_pv_s *pv = dev->drv_pv;
  struct dev_pcm_rq_s *rq = pv->rq;

  logk_trace("%s\n", __FUNCTION__);

  if (!rq) {
    nrf_task_trigger(PDM_ADDR, NRF_PDM_STOP);
    nrf_it_disable_mask(PDM_ADDR, -1);
    return;
  }

  if (nrf_event_check(PDM_ADDR, NRF_PDM_STARTED)) {
    nrf_event_clear(PDM_ADDR, NRF_PDM_STARTED);
    nrf_it_disable(PDM_ADDR, NRF_PDM_STARTED);
    nrf_reg_set(PDM_ADDR, NRF_PDM_SAMPLE_PTR, (uint32_t)rq->stream[0].buffer[1]);
  }

  if (nrf_event_check(PDM_ADDR, NRF_PDM_END)) {
    nrf_event_clear(PDM_ADDR, NRF_PDM_END);

    rq->offline_buffer_index = !rq->offline_buffer_index;
    nrf_reg_set(PDM_ADDR, NRF_PDM_SAMPLE_PTR, (uint32_t)rq->stream[0].buffer[rq->offline_buffer_index]);

    if (atomic_dec(&rq->frames_left) == 0) {
      nrf_it_enable(PDM_ADDR, NRF_PDM_STOPPED);
      nrf_task_trigger(PDM_ADDR, NRF_PDM_STOP);
    }

    dev_pcm_rq_done(rq);
  }

  if (nrf_event_check(PDM_ADDR, NRF_PDM_STOPPED)) {
    nrf_event_clear(PDM_ADDR, NRF_PDM_STOPPED);
    nrf_it_disable_mask(PDM_ADDR, -1);

    kroutine_exec(&pv->deleter);
  }
}

static DEV_PCM_REQUEST(nrf52_pdm_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf52_pdm_pv_s *pv = dev->drv_pv;
  size_t stream;
  uint16_t *sample;
  uint32_t mode;

  logk_debug("%s\n", __FUNCTION__);

  if (pv->rq)
    return -EBUSY;

  if (atomic_get(&rq->frames_left) == 0)
    return -EINVAL;

  if (rq->stream_count == 0)
    return -EINVAL;

  if (rq->stream_count > 2)
    return -ENOTSUP;

  for (stream = 0; stream < rq->stream_count; ++stream) {
    if (rq->stream[stream].sample_type != DEV_PCM_DT_INT16LE)
      return -ENOTSUP;

    if (rq->stream[stream].direction != DEV_PCM_DIR_INPUT)
      return -ENOTSUP;

    if (rq->stream[stream].channel_id != stream)
      return -EINVAL;

    rq->stream[stream].stride = rq->stream_count * 2;
  }

  sample = mem_alloc(rq->sample_count * sizeof(int16_t) * 2 * rq->stream_count, mem_scope_sys);

  pv->rq = rq;

  rq->error = 0;

  rq->stream[0].buffer[0] = sample;
  rq->stream[0].buffer[1] = sample + rq->sample_count * rq->stream_count;
  mode = NRF_PDM_MODE_OPERATION_MONO;

  if (rq->stream_count > 1) {
    rq->stream[1].buffer[0] = sample + 1;
    rq->stream[1].buffer[1] = sample + rq->sample_count * rq->stream_count + 1;
    mode = NRF_PDM_MODE_OPERATION_STEREO;
  }

  //mode |= NRF_PDM_MODE_EDGE_LEFTFALLING;

  rq->offline_buffer_index = 1;

  uint32_t reg = NRF_PDM_CLKCTRL_FREQ(rq->sample_rate);
  rq->effective_sample_rate = NRF_PDM_CLKCTRL_RATE(reg);

  logk_debug("Starting pdm %dHz, clock reg=%08x, mode reg=%08x, esr: %d\n",
         rq->sample_rate, reg, mode, rq->effective_sample_rate);

  nrf_reg_set(PDM_ADDR, NRF_PDM_ENABLE, NRF_PDM_ENABLE_ENABLED);
  nrf_reg_set(PDM_ADDR, NRF_PDM_MODE, mode);
  nrf_reg_set(PDM_ADDR, NRF_PDM_GAINL, NRF_PDM_GAIN(0));
  nrf_reg_set(PDM_ADDR, NRF_PDM_GAINR, NRF_PDM_GAIN(0));
  nrf_reg_set(PDM_ADDR, NRF_PDM_CLKCTRL, reg);
  nrf_reg_set(PDM_ADDR, NRF_PDM_SAMPLE_PTR, (uint32_t)sample);
  nrf_reg_set(PDM_ADDR, NRF_PDM_SAMPLE_MAXCNT, rq->sample_count * rq->stream_count);

#ifdef CONFIG_DRIVER_NRF52_PDM_POWERGATE
  dev_clock_sink_gate(&pv->power_source, DEV_CLOCK_EP_POWER);
#endif

  nrf_it_enable(PDM_ADDR, NRF_PDM_STARTED);
  nrf_it_enable(PDM_ADDR, NRF_PDM_END);
  nrf_task_trigger(PDM_ADDR, NRF_PDM_START);

  return 0;
}

#define nrf52_pdm_use dev_use_generic

static DEV_INIT(nrf52_pdm_init)
{
  struct nrf52_pdm_pv_s *pv;
  iomux_io_id_t id[2];

  __unused__ uintptr_t addr = 0;
  assert(device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL) == 0 &&
         PDM_ADDR == addr);

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));

  if (device_iomux_setup(dev, ">clk <din", NULL, id, NULL))
    goto free_pv;

#ifdef CONFIG_DRIVER_NRF52_PDM_POWERGATE
  if (dev_drv_clock_init(dev, &pv->power_source, 0, 0, NULL))
    goto free_pv;
#endif

  kroutine_init_idle(&pv->deleter, nrf52_pdm_ended);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_reg_set(PDM_ADDR, NRF_PDM_ENABLE, 0);

  nrf_reg_set(PDM_ADDR, NRF_PDM_PSEL_CLK, id[0]);
  nrf_reg_set(PDM_ADDR, NRF_PDM_PSEL_DIN, id[1]);

  nrf_it_disable_mask(PDM_ADDR, -1);

  nrf_event_clear(PDM_ADDR, NRF_PDM_STARTED);
  nrf_event_clear(PDM_ADDR, NRF_PDM_STOPPED);
  nrf_event_clear(PDM_ADDR, NRF_PDM_END);

  CPU_INTERRUPT_RESTORESTATE;

  device_irq_source_init(dev, &pv->irq_ep, 1, &nrf52_pdm_irq);
  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto free_pv;

  return 0;

 free_pv:
  nrf_reg_set(PDM_ADDR, NRF_PDM_PSEL_CLK, (uint32_t)-1);
  nrf_reg_set(PDM_ADDR, NRF_PDM_PSEL_DIN, (uint32_t)-1);

  mem_free(pv);

  return -1;
}

static DEV_CLEANUP(nrf52_pdm_cleanup)
{
  struct nrf52_pdm_pv_s *pv = dev->drv_pv;

  if (pv->rq)
    return -EBUSY;

  nrf_it_disable_mask(PDM_ADDR, -1);

#ifdef CONFIG_DRIVER_NRF52_PDM_POWERGATE
  dev_drv_clock_cleanup(dev, &pv->power_source);
#endif

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  nrf_task_trigger(PDM_ADDR, NRF_PDM_STOPPED);

  nrf_reg_set(PDM_ADDR, NRF_PDM_ENABLE, 0);

  nrf_reg_set(PDM_ADDR, NRF_PDM_PSEL_CLK, (uint32_t)-1);
  nrf_reg_set(PDM_ADDR, NRF_PDM_PSEL_DIN, (uint32_t)-1);

  device_iomux_cleanup(dev);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf52_pdm_drv, 0, "nRF52 PDM", nrf52_pdm,
               DRIVER_PCM_METHODS(nrf52_pdm));

DRIVER_REGISTER(nrf52_pdm_drv);

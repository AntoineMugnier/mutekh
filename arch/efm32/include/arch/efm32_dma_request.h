#ifndef __DEVICE_DMA_REQ_H__
#define __DEVICE_DMA_REQ_H__

#include <device/class/dma.h>

struct efm32_dev_dma_rq_s
{
  /* legacy request */
  struct dev_dma_rq_s      rq;
  /* triggering source */
  uint32_t                 trigsrc;
  /* dma mode */
  uint8_t                  mode:3;
  /* arbiter mode */
  uint8_t                  arbiter:4;
  /* Request linked to this one */
  struct efm32_dev_dma_rq_s * lrq;
};

#endif

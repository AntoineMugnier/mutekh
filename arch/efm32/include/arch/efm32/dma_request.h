#ifndef __DEVICE_DMA_REQ_H__
#define __DEVICE_DMA_REQ_H__

#include <device/class/dma.h>


struct efm32_dev_dma_cfg_s
{
  /* triggering source */
  uint32_t                 trigsrc;
  /* arbiter mode */
  uint8_t                  arbiter:4;
};

struct efm32_dev_dma_rq_s
{
  /* legacy request */
  struct dev_dma_rq_s        rq;
  /* Up to 2 configuration for interleaving */
  struct efm32_dev_dma_cfg_s cfg[2];
};

#endif

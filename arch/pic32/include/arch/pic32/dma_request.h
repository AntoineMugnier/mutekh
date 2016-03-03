#ifndef __DEVICE_DMA_REQ_H__
#define __DEVICE_DMA_REQ_H__

#include <device/class/dma.h>


struct pic32_dev_dma_cfg_s
{
  uint32_t                 trigsrc;
  /* arbiter mode */
  uint16_t                  cell_size;
};

struct pic32_dev_dma_rq_s
{
  /* legacy request */
  struct dev_dma_rq_s        rq;
  /* Up to 2 configuration for interleaving */
  struct pic32_dev_dma_cfg_s cfg[2];
};

#endif

/* -*- c -*- */
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

    Copyright (c) 2014 Sebastien Cerdan <sebcerdan@gmail.com>

*/

/* This driver implements a low energy management (270 nA) of the ADXL362. In this mode,
   only activity and inactivity detection are supported. Event duration is taken into 
   account only for inactivity detection. Thus, activity detection is triggered as soon as 
   a overthreshold condition is detected. Activity and inactivity detection can be used 
   simultaneously */

#include "adxl362.h"

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/bytecode.h>

#include <string.h>

static inline void adxl362_end_rq(struct device_s *dev)
{
  struct adxl362_private_s *pv = dev->drv_pv;
  struct dev_request_s *grq = dev_request_queue_head(&pv->queue);
  struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(grq);
  
  struct valio_motion_axis_data_s *v = NULL;

  switch (rq->type)
    {
      case DEVICE_VALIO_READ:
        v = &((struct valio_motion_data_s*)rq->data)->accel;
        break;

      case DEVICE_VALIO_WAIT_UPDATE:
        v = &((struct valio_motion_evt_s*)rq->data)->data.accel;
        break;

      default:
        break;
    }

  if (v != NULL)
    {
      v->axis = VALIO_MOTION_ACC_XYZ; 
      v->x = (int16_t)pv->x << 4; 
      v->y = (int16_t)pv->y << 4;
      v->z = (int16_t)pv->z << 4;
    }

  dev_request_queue_pop(&pv->queue);
  lock_release_irq2(&dev->lock, &pv->irq_save);
  kroutine_exec(&grq->kr, 0);
}

static bool_t adxl362_process(struct device_s *dev)
{
  struct adxl362_private_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_rq_s *srq = &pv->spi_rq;

  struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(dev_request_queue_head(&pv->queue));
  
  if (pv->flags & ADXL362_FLAGS_BC_RUN)
    return 0;

  while(1)
  {
#ifdef ADXL362_DEBUG            
  printk("State: %d\n", pv->state);
#endif 
    switch (pv->state)
      {
        case ADXL362_STATE_DOWN:
          bc_set_pc(&srq->vm, &adxl362_spi_entry_reset);
          pv->state = ADXL362_STATE_READY;
          return 1;
  
        case ADXL362_STATE_READY:
          if (rq)
            {
              pv->flags = 0;

              switch (rq->type)
                {
                   case DEVICE_VALIO_READ:
                     bc_set_pc(&srq->vm, &adxl362_spi_entry_read_value);
                     pv->state = ADXL362_STATE_READ;
                     break;
          
                   case DEVICE_VALIO_WRITE:{
             
                     struct valio_motion_accel_s *macc = (struct valio_motion_accel_s*)rq->data;
             
                     pv->time = macc->inact.duration/ADXL362_TIME_GRANULARITY;
             
                     bc_set_reg(&srq->vm, R_ARG0, macc->mask);
                     bc_set_reg(&srq->vm, R_ARG1, macc->act.x);
                     bc_set_reg(&srq->vm, R_ARG2, macc->inact.x);
                     bc_set_pc(&srq->vm, &adxl362_spi_entry_cfg);
             
                     pv->state = ADXL362_STATE_WRITE;
             
                     break;}
           
                   case DEVICE_VALIO_WAIT_UPDATE:

                     bc_set_pc(&srq->vm, &adxl362_spi_entry_wait);
                     bc_set_reg(&srq->vm, R_ARG0, ((struct valio_motion_evt_s *)rq->data)->evts);

                     pv->flags |= ADXL362_FLAGS_WAIT_RQ;
             
                     pv->state = ADXL362_STATE_WAIT;
             
                     break;
                }
              return 1;
            }
            
        case ADXL362_STATE_WRITE:
        case ADXL362_STATE_READ:
          adxl362_end_rq(dev);
          pv->state = ADXL362_STATE_READY;
          break;
 
        case ADXL362_STATE_WAIT:{
            
          struct valio_motion_evt_s * data = rq->data; 
          uint32_t st = bc_get_reg(&srq->vm, R_STATUS);
          uint16_t evt = 0;
          bool_t end = 0;
 
          if ((st & 0x10) && (data->evts & VALIO_MOTION_ACC_ACT))
            {
              evt |= VALIO_MOTION_ACC_ACT;
              end = 1;
            }

          if ((st & 0x20) && (data->evts & VALIO_MOTION_ACC_INACT))
            {
              evt |= VALIO_MOTION_ACC_INACT;
              end = 1;
            }

          if (end)
            {
              data->revts = evt;
              pv->flags = 0;
              adxl362_end_rq(dev);
              pv->state = ADXL362_STATE_READY;
              break;
            }

          return 0;}

        default:
          return 0;
      }
  }
}

void adxl362_run(struct device_s *dev)
{
  struct adxl362_private_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_rq_s *srq = &pv->spi_rq;

  while (1)
    {
      if (pv->flags & ADXL362_FLAGS_IRQ)
        bc_set_pc(&srq->vm, &adxl362_spi_entry_irq);
      else if (!adxl362_process(dev))
        {
          lock_release_irq2(&dev->lock, &pv->irq_save);
          return;
        }

      pv->flags |= ADXL362_FLAGS_BC_RUN;
      lock_release_irq2(&dev->lock, &pv->irq_save);
      dev_spi_rq_start(srq);

      if (!kroutine_trigger(&srq->base.kr, 0, KROUTINE_IMMEDIATE))
        return;

      lock_spin_irq2(&dev->lock, &pv->irq_save);
    }
}

static DEV_VALIO_REQUEST(adxl362_request)
{
    struct device_s          *dev = accessor->dev;
    struct adxl362_private_s *pv  = dev->drv_pv;

    lock_spin_irq2(&dev->lock, &pv->irq_save);

    req->error = 0;
    bool_t start = dev_request_queue_isempty(&pv->queue) && (pv->state <= ADXL362_STATE_READY);
    bool_t end = 0;

    switch (req->type)
    {
      case DEVICE_VALIO_READ:
          switch (req->attribute)
            {
              case VALIO_MOTION_CAPS:{
                  struct valio_motion_caps_s *caps = (struct valio_motion_caps_s*)req->data;
                  caps->mask = VALIO_MOTION_CAP_ACC_INACT | VALIO_MOTION_CAP_ACC_ACT;
                  end = 1;
                  break;}
              case VALIO_MOTION_DATA:
                  dev_request_queue_pushback(&pv->queue, &req->base);
                  break;
              default:
                  req->error = -ENOTSUP;
                  break;
            }
          break;
    
      case DEVICE_VALIO_WRITE:
          switch (req->attribute)
            {
              case VALIO_MOTION_ACCEL:{
                struct valio_motion_accel_s *macc = (struct valio_motion_accel_s*)req->data;
                if (macc->mask & ~(VALIO_MOTION_ACC_OPT_ACT | VALIO_MOTION_ACC_OPT_INACT))
                  req->error = -ENOTSUP;
                else if (macc->mask & VALIO_MOTION_ACC_OPT_ACT && macc->act.duration)
                  req->error = -ENOTSUP;
                else
                  dev_request_queue_pushback(&pv->queue, &req->base);
                break;}
              default:
                  req->error = -ENOTSUP;
                  break;
            }
          break;
    
      case DEVICE_VALIO_WAIT_UPDATE:
          switch (req->attribute)
            {
              case VALIO_MOTION_EVENT:
                  dev_request_queue_pushback(&pv->queue, &req->base);
                  break;
              default:
                  req->error = -ENOTSUP;
                  break;
            }
          break;

      default:
          break;
    }

   start = start && !(pv->flags & ADXL362_FLAGS_BC_RUN) && !req->error && !end;

   if (start)
     adxl362_run(dev);
   else
     lock_release_irq2(&dev->lock, &pv->irq_save);

   if (req->error || end)
     kroutine_exec(&req->base.kr, cpu_is_interruptible());
}

static DEV_IRQ_SRC_PROCESS(adxl362_irq_source_process)
{
  struct device_s *dev = ep->base.dev;
  struct adxl362_private_s *pv = dev->drv_pv;

#ifdef ADXL362_DEBUG            
  printk("irq\n");
#endif 

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  if (!(pv->flags & ADXL362_FLAGS_WAIT_RQ))
    {
      lock_release_irq2(&dev->lock, &pv->irq_save);
      return;
    }
    
  pv->flags |= ADXL362_FLAGS_IRQ;

  if (pv->flags & ADXL362_FLAGS_BC_RUN)
    lock_release_irq2(&dev->lock, &pv->irq_save);
  else
    adxl362_run(dev);
}

static KROUTINE_EXEC(spi_rq_done)
{
  struct dev_request_s *grq = KROUTINE_CONTAINER(kr, *grq, kr);
  struct dev_spi_ctrl_rq_s *srq = dev_spi_ctrl_rq_s_cast(grq);
  struct device_s *dev = grq->pvdata;
  struct adxl362_private_s *pv = dev->drv_pv;

  lock_spin_irq2(&dev->lock, &pv->irq_save);

  pv->flags ^= ADXL362_FLAGS_BC_RUN;

  if (srq->err)
    abort();

  if (kroutine_triggered_1st(kr))
    adxl362_run(dev);
  else
    lock_release_irq2(&dev->lock, &pv->irq_save);
}

static DEV_INIT(adxl362_init);
static DEV_CLEANUP(adxl362_cleanup);

#define adxl362_use dev_use_generic

DRIVER_DECLARE(adxl362_drv, 0, "ADXL362 motion", adxl362,
               DRIVER_VALIO_METHODS(adxl362));

DRIVER_REGISTER(adxl362_drv);

static DEV_INIT(adxl362_init)
{
  struct adxl362_private_s *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;
  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  struct dev_spi_ctrl_rq_s *srq = &pv->spi_rq;

  if (dev_spi_request_init(dev, srq))
    goto err_mem;

  if (!device_check_accessor(&srq->queue->timer))
    goto err_srq;

  srq->config.bit_rate = 1000000;
  srq->config.word_width = 8;
  srq->config.bit_order = DEV_SPI_MSB_FIRST;
  srq->cs_polarity = DEV_SPI_CS_ACTIVE_LOW;
  srq->base.pvdata = dev;

  /* init GPIO stuff */

  static const gpio_width_t pin_wmap[1] = {1};

  if (device_res_gpio_map(dev, "nsel:1", pv->pin_map, NULL))
    goto err_srq;          

  if (device_get_param_dev_accessor(dev, "gpio", &srq->gpio, DRIVER_CLASS_GPIO))
    goto err_srq;

  srq->gpio_map = pv->pin_map;
  srq->gpio_wmap = pin_wmap;

  if (device_gpio_map_set_mode(&srq->gpio, pv->pin_map, pin_wmap, 1, DEV_PIN_PUSHPULL))
    goto err_srq;

  if (pv->pin_map[0] != GPIO_INVALID_ID)
    srq->cs_gpio = 1;

  dev_request_queue_init(&pv->queue);

  pv->state = ADXL362_STATE_DOWN;

  kroutine_init(&srq->base.kr, &spi_rq_done, KROUTINE_TRIGGER);
  bc_init(&srq->vm, &adxl362_bytecode, 1, /* R_CTX_PV */ pv);

  /* Disable bytecode trace */
  bc_set_trace(&srq->vm, 0, 0);

  /* irq io pin */
  device_irq_source_init(dev, &pv->src_ep, 1,
                         &adxl362_irq_source_process /*, DEV_IRQ_SENSE_RISING_EDGE*/);

  if (device_irq_source_link(dev, &pv->src_ep, 1, -1))
    goto err_srq;

  /* done */
  dev->drv = &adxl362_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

#if 0
 err_unlink:
  device_irq_source_unlink(dev, &pv->src_ep, 1);
#endif
 err_srq:
  dev_spi_request_cleanup(srq);
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(adxl362_cleanup)
{
  struct adxl362_private_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->src_ep, 1);
  dev_spi_request_cleanup(&pv->spi_rq);

  mem_free(pv);
}

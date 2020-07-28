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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014

*/


#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>

#include <device/class/timer.h>
#include <device/class/iomux.h>
#include <device/class/valio.h>
#include <device/class/gpio.h>
#include <device/valio/button.h>

DRIVER_PV(struct push_button_context_s
{
  bool_t state;
  /* Value when released */
  bool_t release_state;
  /* Interrupt endpoint */
  struct dev_irq_src_s irq_ep;
  /* request queue */
  dev_request_queue_root_t queue;
  /* gpio dev and map */
  struct device_gpio_s gpio;
  gpio_id_t pin_map[1];
#ifdef CONFIG_DRIVER_PUSH_BUTTON_TIMER
  /* Timer accessor */
  struct device_timer_s timer;
  /* fast conversion */
  int8_t shifta, shiftb;
  /* Last read value on timer */
  dev_timer_value_t last_value;
#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
  /* lock during debouncing */
  bool_t lock;
  /* debouncing timeout */
  uint32_t timeout;
  /* timer request */
  struct dev_timer_rq_s trq;
#endif
#endif
});

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING

static KROUTINE_EXEC(push_button_lock_timeout)
{
   struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
   struct device_s *dev = rq->pvdata;
   struct push_button_context_s *pv  = dev->drv_pv;

   LOCK_SPIN_IRQ(&dev->lock);
   pv->lock = 0;
   LOCK_RELEASE_IRQ(&dev->lock);
}

static bool_t push_button_timer_rq(struct device_s *dev)
{
   struct push_button_context_s *pv  = dev->drv_pv;

   while (1)
     {
       pv->trq.delay = pv->timeout;
       switch (DEVICE_OP(&pv->timer, request, &pv->trq))
         {
         case -EAGAIN:
           if (!dev_timer_init_sec(&pv->timer, &pv->timeout, &pv->trq.rev,
                  CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING_TIMING, 1000))
             continue;
         default:
           printk("Push button driver: Timer error\n");
           return 1;
         case -ETIMEDOUT:
           return 1;
         case 0:
           return 0;
         }
     }
}

#endif

/***************************************** interrupt */

static DEV_IRQ_SRC_PROCESS(push_button_irq)
{
  struct device_s *dev = ep->base.dev;
  struct push_button_context_s *pv  = dev->drv_pv;

  struct dev_valio_rq_s *rq = dev_valio_rq_head(&pv->queue);

  lock_spin(&dev->lock);

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
  if (pv->lock)
    {
      lock_release(&dev->lock);
      return;
    }
#endif

  bool_t done = 0;

  pv->state ^= 1;

  uint32_t timestamp = 0;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_TIMER

  dev_timer_value_t value;

  if (device_check_accessor(&pv->timer.base))
    {
      /* Get timer value */
      if (!DEVICE_OP(&pv->timer, get_value, &value, 0))
        timestamp = dev_timer_delay_shift_t2s(pv->shiftb, value - pv->last_value);

      pv->last_value = value;

      /* Saturate timestamp */
      if (timestamp >> 16)
        timestamp = (1 << 16) - 1;
    }

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
  push_button_timer_rq(dev);
  pv->lock = 1;
#endif

#endif

  if (rq)
    {
      struct valio_button_update_s * data = (struct valio_button_update_s *)rq->data;
      data->timestamp = timestamp;

      switch ((enum valio_button_att)rq->attribute)
        {
          case VALIO_BUTTON_TOGGLE:
            done = 1;
            break;
          case VALIO_BUTTON_PUSH:
            done = pv->release_state ^ pv->state;
            break;
          case VALIO_BUTTON_RELEASE:
            done = !(pv->release_state ^ pv->state);
            break;
        }
    }

  if (done)
    {
      dev_valio_rq_pop(&pv->queue);
      dev_valio_rq_done(rq);
    }

  lock_release(&dev->lock);
}

/***************************************** request */

static DEV_VALIO_REQUEST(push_button_request)
{
  struct device_s *dev = accessor->dev;
  struct push_button_context_s *pv  = dev->drv_pv;

  bool_t done = 1;

  LOCK_SPIN_IRQ(&dev->lock);

  rq->error = 0;

  struct valio_button_read_s * data = (struct valio_button_read_s*)rq->data;

  switch (rq->type) {
    case DEVICE_VALIO_WRITE:
      rq->error = -ENOTSUP;
      break;

    case DEVICE_VALIO_READ:
      data->state = pv->state;
      break;

    case DEVICE_VALIO_WAIT_EVENT:
      done = 0;
      dev_valio_rq_pushback(&pv->queue, rq);
      break;

    default:
      rq->error = -EINVAL;
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (done)
    dev_valio_rq_done(rq);
}


#define push_button_use dev_use_generic
#define push_button_cancel (dev_valio_cancel_t*)&dev_driver_notsup_fcn

static DEV_INIT(push_button_init)
{
  struct push_button_context_s *pv;


  /* allocate driver private context. */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  /* Retrieve button release state from ressource */
  struct dev_resource_s *r = device_res_get_from_name(dev, DEV_RES_UINT_PARAM, 0, "release-state");

  if (r)
    pv->release_state = r->u.uint_param.value;
  else
    goto err_mem;

  pv->state = pv->release_state;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_TIMER
  /* Get accessor on timer */
  if (!device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER))
    {
      /* Start timer */
      device_start(&pv->timer.base);
      dev_timer_shift_sec(&pv->timer, &pv->shifta, &pv->shiftb, 0, 1, 1000);
#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
      dev_timer_init_sec(&pv->timer, &pv->timeout, 0,
        CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING_TIMING, 1000);
      pv->trq.pvdata = dev;
      pv->trq.delay = pv->timeout;
      dev_timer_rq_init(&pv->trq, push_button_lock_timeout);
#endif
    }
  else
    device_init_accessor(&pv->timer.base);
#endif

  dev_rq_queue_init(&pv->queue);

  /* Init GPIO and Irq */
  if (device_gpio_get_setup(&pv->gpio, dev, "=pbirq:1", pv->pin_map, NULL))
    goto err_mem;

  device_irq_source_init(dev, &pv->irq_ep, 1, &push_button_irq);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, -1))
    goto err_mem;


  return 0;

 err_mem:
   mem_free(pv);
   return -EINVAL;
}

static DEV_CLEANUP(push_button_cleanup)
{
  struct push_button_context_s *pv = dev->drv_pv;

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  /* Destroy request queue */
  dev_rq_queue_destroy(&pv->queue);

  /* deallocate private driver context. */
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(push_button_drv, 0, "Push-button", push_button,
               DRIVER_VALIO_METHODS(push_button));

DRIVER_REGISTER(push_button_drv);


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

#include <stdbool.h>

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
  bool busy; // Indicates that another wait event request is in action
  uint8_t release_state; // Value when button is released
  uint8_t current_state; // Current value
  dev_request_queue_root_t queue; // Request queue
  struct device_gpio_s gpio; // Gpio device
  gpio_id_t pin_map[1]; // Gpio map
  struct dev_gpio_rq_s gpio_rq; // Gpio request
#ifdef CONFIG_DRIVER_PUSH_BUTTON_TIMER
  struct device_timer_s timer; //Timer device
  int8_t shifta, shiftb;
  uint32_t base_time; // Time base for the driver
  dev_timer_value_t last_value; // Last read value on timer
#ifdef CONFIG_DRIVER_PUSH_BUTTON_SUSTAINED
  bool sustain_trq_active; // Indicates if sustain tiemr request is active
  bool was_sustained; // Indicates if button was pressed during the whole time
  struct dev_timer_rq_s sustain_trq; // Sustain push timer request
#endif
#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
  bool debounce_lock; // Soft debouncing lock
  bool debounce_was_locked; // Indicates that a soft debouncing lock was active
  uint32_t debounce_timeout; // Debouncing timeout
  struct dev_timer_rq_s debounce_trq; // Debouncing timer request
#endif
#endif
});

#ifdef CONFIG_DRIVER_PUSH_BUTTON_TIMER
// Return true if request started
static bool push_button_start_timer_rq(struct device_timer_s *timer, struct dev_timer_rq_s *trq)
{
  error_t err = DEVICE_OP(timer, request, trq);
  switch (err)
  {
    default:
      printk("Push button driver: Start timer req error %d\n", err);
      return false;
    case -ETIMEDOUT:
      return false;
    case 0:
      return true;
  }
}
// Return true if request canceled
static bool push_button_cancel_timer_rq(struct device_timer_s *timer, struct dev_timer_rq_s *trq)
{
  error_t err = DEVICE_OP(timer, cancel, trq);
  switch (err)
  {
    default:
      printk("Push button driver: Cancel timer req error %d\n", err);
      return false;
    case -ETIMEDOUT:
    case -EBUSY:
      return false;
    case 0:
      return true;
  }
}
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING

static KROUTINE_EXEC(push_button_lock_timeout)
{
   struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
   struct device_s *dev = rq->pvdata;
   struct push_button_context_s *pv  = dev->drv_pv;

   LOCK_SPIN_IRQ(&dev->lock);
   pv->debounce_lock = false;
   LOCK_RELEASE_IRQ(&dev->lock);
   if (pv->debounce_was_locked)
   {
    pv->debounce_was_locked = false;
    DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
   }
}
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SUSTAINED

static KROUTINE_EXEC(push_button_sustain_timeout)
{
  struct dev_timer_rq_s *trq = dev_timer_rq_from_kr(kr);
  struct device_s *dev = trq->pvdata;
  struct push_button_context_s *pv  = dev->drv_pv;
  struct dev_valio_rq_s *rq = dev_valio_rq_head(&pv->queue);

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq == NULL)
    return;
  /* Cancel request if button was released but trq couldn't cancel */
  if (!pv->was_sustained)
  {
    /* Clear request flag */
    pv->sustain_trq_active = false;
    pv->busy = false;
    dev_valio_rq_pop(&pv->queue);
    dev_valio_rq_done(rq);
    return;
  }
  struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;
  /* Activate callback */
  data->pb_event(rq);
  /* Restart timer rq */
  push_button_start_timer_rq(&pv->timer, &pv->sustain_trq);

  LOCK_RELEASE_IRQ(&dev->lock);
}

#endif

/***************************************** event */

static KROUTINE_EXEC(push_button_event)
{
  struct dev_gpio_rq_s *grq = dev_gpio_rq_from_kr(kr);
  struct device_s *dev = grq->pvdata;
  struct push_button_context_s *pv  = dev->drv_pv;
  struct dev_valio_rq_s *rq = dev_valio_rq_head(&pv->queue);

if (grq->error != 0)
{
  printk("Push button driver: Gpio error %d\n", grq->error);
  return;
}

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
  if (pv->debounce_lock)
  {
    pv->debounce_was_locked = true;
    return;
  }
#endif

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
  push_button_start_timer_rq(&pv->timer, &pv->debounce_trq);
  pv->debounce_lock = true;
#endif

#endif
  /* Update gpio value */
  pv->current_state = !pv->current_state;

  /* Process request */
  if (rq != NULL)
    {
      bool rq_done = false;
      struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;

      switch ((enum valio_button_att)rq->attribute)
        {
          case VALIO_BUTTON_TOGGLE:
            rq_done = true;
            break;

          case VALIO_BUTTON_PUSH:
            rq_done = (pv->current_state != pv->release_state);
            break;

          case VALIO_BUTTON_RELEASE:
            rq_done = (pv->current_state == pv->release_state);
            break;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SUSTAINED
         case VALIO_BUTTON_SUSTAINED_PUSH:
          /* Button released */
          if (pv->current_state == pv->release_state)
          {
            pv->was_sustained = false;
            /* Allow new request only if cancel successful */
            if (push_button_cancel_timer_rq(&pv->timer, &pv->sustain_trq))
            {
              pv->sustain_trq_active = false;
              rq_done = true;
            }
          }
          /* Button pushed and no timer request */
          else if (!pv->sustain_trq_active)
          {
            /* Activate callback */
            data->pb_event(rq);
            /* Start timer requests */
            pv->was_sustained = true;
            pv->sustain_trq_active = true;
            push_button_start_timer_rq(&pv->timer, &pv->sustain_trq);
          }
          break;
#endif
          default:
            rq->error = -ENOTSUP;
            rq_done = true;
            break;
        }
    /* Complete request */
    if (rq_done)
      {
        /* Note timestamp */
        data->timestamp = timestamp;
        /* End request */
        pv->busy = false;
        dev_valio_rq_pop(&pv->queue);
        dev_valio_rq_done(rq);
      }
    /* Wait until gpio change */
    else
      {
        DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
      }
    }
}

/***************************************** request */

static DEV_VALIO_REQUEST(push_button_request)
{

  struct device_s *dev = accessor->dev;
  struct push_button_context_s *pv  = dev->drv_pv;

  bool rq_done = true;

  LOCK_SPIN_IRQ(&dev->lock);

  struct valio_button_read_s * data = (struct valio_button_read_s*)rq->data;

  rq->error = 0;

  switch (rq->type) {
    case DEVICE_VALIO_WRITE:
      rq->error = -ENOTSUP;
      break;

    case DEVICE_VALIO_READ:
      /* Read gpio value */
      pv->gpio_rq.type = DEV_GPIO_GET_INPUT;
      pv->gpio_rq.input.data = (uint8_t *)&data->state;
      DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
      break;

    case DEVICE_VALIO_WAIT_EVENT:
      if (pv->busy) {
        rq->error = -EBUSY;
        break;
      }
#ifdef CONFIG_DRIVER_PUSH_BUTTON_SUSTAINED
      /* Initialize sustain timer request delay */
      if (rq->attribute == VALIO_BUTTON_SUSTAINED_PUSH)
        pv->sustain_trq.delay = pv->base_time * ((struct valio_button_update_s *)rq->data)->delay;
#else
      if (rq->attribute == VALIO_BUTTON_SUSTAINED_PUSH)
        rq->error = -ENOTSUP;
#endif
      rq_done = false;
      pv->busy = true;
      dev_valio_rq_pushback(&pv->queue, rq);
      pv->gpio_rq.type = DEV_GPIO_UNTIL;
      pv->gpio_rq.until.mask = dev_gpio_mask1;
      pv->gpio_rq.until.data = &pv->current_state;
      DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
      break;

    default:
      rq->error = -EINVAL;
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  if (rq_done)
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

  if (r == NULL)
    goto err_mem;

  pv->release_state = r->u.uint_param.value & 0x1;
  pv->current_state = pv->release_state;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_TIMER
  /* Get accessor on timer */
  if (!device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER))
    {
      /* Start timer */
      device_start(&pv->timer.base);
      dev_timer_shift_sec(&pv->timer, &pv->shifta, &pv->shiftb, 0, 1, 1000);
      dev_timer_init_sec(&pv->timer, &pv->base_time, 0, 1, 1000);

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SUSTAINED
      pv->sustain_trq.pvdata = dev;
      dev_timer_rq_init(&pv->sustain_trq, push_button_sustain_timeout);
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
      pv->debounce_timeout = pv->base_time * CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING_TIMING;
      pv->debounce_trq.pvdata = dev;
      pv->debounce_trq.delay = pv->debounce_timeout;
      dev_timer_rq_init(&pv->debounce_trq, push_button_lock_timeout);
#endif
    }
  else
    device_init_accessor(&pv->timer.base);
#endif

  dev_rq_queue_init(&pv->queue);

  /* Init GPIO and Irq */
  if (device_gpio_get_setup(&pv->gpio, dev, "=pbirq:1", pv->pin_map, NULL))
    goto err_mem;

  pv->gpio_rq.io_first = pv->pin_map[0];
  pv->gpio_rq.io_last = pv->pin_map[0];
  pv->gpio_rq.pvdata = dev;
  dev_gpio_rq_init(&pv->gpio_rq, push_button_event);

  return 0;

 err_mem:
   mem_free(pv);
   return -EINVAL;
}

static DEV_CLEANUP(push_button_cleanup)
{
  struct push_button_context_s *pv = dev->drv_pv;

  /* Destroy request queue */
  dev_rq_queue_destroy(&pv->queue);

  /* deallocate private driver context. */
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(push_button_drv, 0, "Push-button", push_button,
               DRIVER_VALIO_METHODS(push_button));

DRIVER_REGISTER(push_button_drv);


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
#define LOGK_MODULE_ID "pbtn"

#include <stdbool.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/class/timer.h>
#include <device/class/gpio.h>
#include <device/valio/button.h>

enum pb_gpio_req_state {
  PB_GPIO_REQ_STATE_IDLE,
  PB_GPIO_REQ_STATE_ACTIVE,
};

enum pb_repeat_state_e {
  PB_REPEAT_STATE_IDLE,
  PB_REPEAT_STATE_ACTIVE,
  PB_REPEAT_STATE_CANCELED,
};

enum pb_delay_state_e {
  PB_DELAY_STATE_IDLE,
  PB_DELAY_STATE_ACTIVE,
  PB_DELAY_STATE_CANCELED,
};

DRIVER_PV(struct push_button_context_s
{
  enum pb_gpio_req_state grq_state; // State of the gpio request
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
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_DELAYED
  uint8_t active_delay_rq_count; // Counts the number of active delayed requests
  dev_request_queue_root_t delay_queue; // Delayed request queue
  struct dev_timer_rq_s delay_trq; // Delayed push timer request
  enum pb_delay_state_e delay_state; // State of the delay process
  uint32_t pushed_time; // Current button push duration
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_REPEAT
  enum pb_repeat_state_e repeat_state; // State of the repeat process
  struct dev_timer_rq_s repeat_trq; // Sustain push timer request
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
  uint32_t debounce_timeout; // Debouncing timeout
  struct dev_timer_rq_s debounce_trq; // Debouncing timer request
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
      logk_trace("start timer req error %d", err);
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
      logk_trace("cancel timer req error %d", err);
      return false;
    case -ETIMEDOUT:
    case -EBUSY:
      return false;
    case 0:
      return true;
  }
}

static uint16_t push_button_timestamp(struct push_button_context_s *pv)
{
  uint32_t timestamp = 0;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_TIMER
    dev_timer_value_t value;

    /* Get timer value */
    if (!DEVICE_OP(&pv->timer, get_value, &value, 0))
      timestamp = dev_timer_delay_shift_t2s(pv->shiftb, value - pv->last_value);

    pv->last_value = value;
#endif

  return timestamp;
}
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_DELAYED
static void push_button_calc_delay(struct push_button_context_s *pv)
{
  /* Parse requests to find min delay value */
  uint32_t delay_min = ~0; // Init to max value

  GCT_FOREACH(dev_request_queue, &pv->delay_queue, r, {
    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(r);
    struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;

    /* Get request delay value */
    if (data->isActive)
      delay_min = __MIN(delay_min, data->delay);
  });
  /* Throw error if 0 */
  if (delay_min == 0)
    UNREACHABLE();
  /* Set delay trq to min delay value */
  assert(delay_min > pv->pushed_time);
  pv->delay_trq.delay = (delay_min - pv->pushed_time) * pv->base_time;
}

static void push_button_end_delayed(struct push_button_context_s *pv)
{
  /* Parse requests */
  GCT_FOREACH(dev_request_queue, &pv->delay_queue, r, {
    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(r);
    struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;

    /* Check if delay reached */
    if (data->isActive && (pv->pushed_time >= data->delay))
    {
      /* Note timestamp */
      data->timestamp = push_button_timestamp(pv);
      /* End request */
      data->isActive = false;
      dev_valio_rq_remove(&pv->queue, rq);
      dev_valio_rq_done(rq);
      /* Decrement active requests count */
      assert(pv->active_delay_rq_count > 0);
      pv->active_delay_rq_count--;
    }
  });
}

static void push_button_activate_delay_rq(struct push_button_context_s *pv)
{
  GCT_FOREACH(dev_request_queue, &pv->delay_queue, r, {
    struct dev_valio_rq_s *rq = dev_valio_rq_s_cast(r);
    struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;
    /* Activate request */
    data->isActive = true;
    /* Increment active requests count */
    pv->active_delay_rq_count++;
  });
}

static void push_button_continue_delayed_rq(struct push_button_context_s *pv)
{
  /* Check if there is a request to continue */
  if (pv->active_delay_rq_count == 0)
  {
    /* Go back to idle */
    pv->delay_state = PB_DELAY_STATE_IDLE;
  }
  else
  {
    /* Set delay trq value */
    push_button_calc_delay(pv);
    /* Start delay trq */
    push_button_start_timer_rq(&pv->timer, &pv->delay_trq);
  }
}

static bool push_button_process_delayed_rq(struct push_button_context_s *pv)
{
  struct dev_valio_rq_s *rq = dev_valio_rq_head(&pv->delay_queue);
  /* Check if there is a request */
  if (rq == NULL)
    return true;

  bool isPushed = (pv->current_state != pv->release_state);

  switch (pv->delay_state)
  {
    case PB_DELAY_STATE_IDLE:
      /* Activate trq if button pushed */
      if (isPushed)
      {
        pv->delay_state = PB_DELAY_STATE_ACTIVE;
         /* Activate requests */
        push_button_activate_delay_rq(pv);
        /* Set delay trq value */
        pv->pushed_time = 0;
        push_button_calc_delay(pv);
        /* Start delay trq */
        push_button_start_timer_rq(&pv->timer, &pv->delay_trq);
      }
    break;

    case PB_DELAY_STATE_ACTIVE:
      /* Cancel trq if button released */
      if (!isPushed)
      {
        /* Try to cancel request */
        if (push_button_cancel_timer_rq(&pv->timer, &pv->delay_trq))
          pv->delay_state = PB_DELAY_STATE_IDLE;
        /* If cancel fail, wait for timer request to end */
        else
          pv->delay_state = PB_DELAY_STATE_CANCELED;
      }
    break;

    case PB_DELAY_STATE_CANCELED:
      /* Wait for trq to end */
    break;

    default:
      UNREACHABLE();
  }

  return false;
}

static KROUTINE_EXEC(push_button_delay_timeout)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct device_s *dev = rq->pvdata;
  struct push_button_context_s *pv  = dev->drv_pv;

  assert(rq);

  LOCK_SPIN_IRQ(&dev->lock);

  switch(pv->delay_state)
  {
    case PB_DELAY_STATE_ACTIVE:
      /* Process requests */
      pv->pushed_time += rq->delay / pv->base_time;
      push_button_end_delayed(pv);
      push_button_continue_delayed_rq(pv);
    break;

    case PB_DELAY_STATE_CANCELED:
      /* Return to idle */
      pv->delay_state = PB_DELAY_STATE_IDLE;
    break;

    case PB_DELAY_STATE_IDLE:
    default:
      UNREACHABLE();
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING

static KROUTINE_EXEC(push_button_lock_timeout)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct device_s *dev = rq->pvdata;
  struct push_button_context_s *pv  = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);
  /* Resume gpio request */
  DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_REPEAT

static KROUTINE_EXEC(push_button_repeat_timeout)
{
  struct dev_timer_rq_s *trq = dev_timer_rq_from_kr(kr);
  struct device_s *dev = trq->pvdata;
  struct push_button_context_s *pv  = dev->drv_pv;
  struct dev_valio_rq_s *rq = dev_valio_rq_head(&pv->queue);

  if (rq == NULL)
    return;

  LOCK_SPIN_IRQ(&dev->lock);

  struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;

  switch(pv->repeat_state)
  {
    case PB_REPEAT_STATE_ACTIVE:
      /* Activate callback */
      data->pb_event(rq);
      /* Restart timer rq */
      push_button_start_timer_rq(&pv->timer, &pv->repeat_trq);
    break;

    case PB_REPEAT_STATE_CANCELED:
      /* Return to idle */
      pv->repeat_state = PB_REPEAT_STATE_IDLE;
      /* End pb request */
      dev_valio_rq_pop(&pv->queue);
      dev_valio_rq_done(rq);
    break;

    case PB_REPEAT_STATE_IDLE:
    default:
      UNREACHABLE();
  }

  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

/***************************************** event */
static bool push_button_process_rq(struct push_button_context_s *pv)
{
  bool rq_done = false;
  struct dev_valio_rq_s *rq = dev_valio_rq_head(&pv->queue);

  if (rq == NULL)
    return true;

  struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;
  bool isPushed = (pv->current_state != pv->release_state);

  switch ((enum valio_button_att)rq->attribute)
    {
      case VALIO_BUTTON_TOGGLE:
        rq_done = true;
        break;

      case VALIO_BUTTON_PUSH:
        rq_done = isPushed;
        break;

      case VALIO_BUTTON_RELEASE:
        rq_done = !isPushed;
        break;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_REPEAT
      case VALIO_BUTTON_REPEAT_PUSH:
        switch (pv->repeat_state)
        {
          case PB_REPEAT_STATE_IDLE:
            /* Activate trq if button pushed */
            if (isPushed)
            {
              pv->repeat_state = PB_REPEAT_STATE_ACTIVE;
              /* Activate callback */
              data->pb_event(rq);
              /* Start timer requests */
              push_button_start_timer_rq(&pv->timer, &pv->repeat_trq);
            }
          break;

          case PB_REPEAT_STATE_ACTIVE:
            /* Cancel trq if button released */
            if (!isPushed)
            {
              /* Try to cancel timer req */
              if (push_button_cancel_timer_rq(&pv->timer, &pv->repeat_trq))
              {
                /* Cancel success */
                pv->repeat_state = PB_REPEAT_STATE_IDLE;
                rq_done = true;
              }
              /* Cancel fail, wait for timer request to end */
              else
                pv->repeat_state = PB_REPEAT_STATE_CANCELED;
            }
          break;

          case PB_REPEAT_STATE_CANCELED:
            /* Wait for trq to end */
          break;

          default:
            UNREACHABLE();
        }
      break;
#endif
      default:
        rq->error = -ENOTSUP;
        rq_done = true;
        break;
    }
  /* Process request end */
  if (rq_done)
  {
    /* Note timestamp */
    data->timestamp = push_button_timestamp(pv);
    /* End pb request */
    dev_valio_rq_pop(&pv->queue);
    dev_valio_rq_done(rq);
  }
  return rq_done;
}

static KROUTINE_EXEC(push_button_event)
{
  struct dev_gpio_rq_s *grq = dev_gpio_rq_from_kr(kr);
  struct device_s *dev = grq->pvdata;
  struct push_button_context_s *pv  = dev->drv_pv;
  bool restart_grq = false;

/* Check for gpio error */
if (grq->error != 0)
{
  logk_trace("gpio error %d\n", grq->error);
  return;
}

  /* Update gpio value */
  pv->current_state = !pv->current_state;

  /* Process request */
  if (!push_button_process_rq(pv))
    restart_grq = true;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_DELAYED
  /* Process delayed requests */
  if (!push_button_process_delayed_rq(pv))
    restart_grq = true;
#endif

  /* Restart gpio rq check */
  if (restart_grq)
  {
#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
    /* Wait debouncing trq before restarting request */
    push_button_start_timer_rq(&pv->timer, &pv->debounce_trq);
#else
    /* Restart gpio rq */
    DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
#endif
  }
  /* End of gpio rq */
  else
    pv->grq_state = PB_GPIO_REQ_STATE_IDLE;
}

/***************************************** request */

static bool push_button_accept_delayed_rq(struct push_button_context_s *pv, struct dev_valio_rq_s *rq)
{
  struct valio_button_update_s *data = (struct valio_button_update_s *)rq->data;

#ifdef CONFIG_DRIVER_PUSH_BUTTON_DELAYED
  /* Accept the request */
  data->isActive = false;
  dev_valio_rq_pushback(&pv->delay_queue, rq);
  return true;
#else
  /* Refuse the request */
  rq->error = -ENOTSUP;
  dev_valio_rq_done(rq);
  return false;
#endif
}

static bool push_button_accept_rq(struct push_button_context_s *pv, struct dev_valio_rq_s *rq)
{
  bool rq_done = true;
  struct valio_button_read_s * data = (struct valio_button_read_s*)rq->data;
  struct dev_valio_rq_s *prev_rq = dev_valio_rq_head(&pv->queue);
  rq->error = 0;

  switch (rq->type) {
    case DEVICE_VALIO_WRITE:
      rq->error = -ENOTSUP;
      break;

    case DEVICE_VALIO_READ:
      /* Read gpio value */
      DEVICE_OP(&pv->gpio, get_input, pv->pin_map[0], pv->pin_map[0], &data->state);
      break;

    case DEVICE_VALIO_WAIT_EVENT:
      /* Only one request allowed at a time */
      if (prev_rq != NULL)
      {
        rq->error = -EBUSY;
        break;
      }
#ifdef CONFIG_DRIVER_PUSH_BUTTON_REPEAT
      /* Initialize repeat timer request delay */
      if (rq->attribute == VALIO_BUTTON_REPEAT_PUSH)
        pv->repeat_trq.delay = pv->base_time * ((struct valio_button_update_s *)rq->data)->delay;
#else
      if (rq->attribute == VALIO_BUTTON_REPEAT_PUSH)
      {
        rq->error = -ENOTSUP;
        break;
      }
#endif
      rq_done = false;
      dev_valio_rq_pushback(&pv->queue, rq);
      break;

    default:
      rq->error = -EINVAL;
      break;
    }

  if (rq_done)
    dev_valio_rq_done(rq);

  return !rq_done;
}

static DEV_VALIO_REQUEST(push_button_request)
{
  struct device_s *dev = accessor->dev;
  struct push_button_context_s *pv  = dev->drv_pv;
  bool start_grq = false;

  LOCK_SPIN_IRQ(&dev->lock);

  /* Process requests */
  if (rq->attribute == VALIO_BUTTON_DELAYED_PUSH)
    start_grq |= push_button_accept_delayed_rq(pv, rq);
  else
    start_grq |= push_button_accept_rq(pv, rq);

  /* Start gpio rq check */
  switch (pv->grq_state)
  {
    case PB_GPIO_REQ_STATE_IDLE:
      /* Start gpio req if needed */
      if (start_grq)
      {
        pv->grq_state = PB_GPIO_REQ_STATE_ACTIVE;
        /* Start gpio request */
        pv->gpio_rq.type = DEV_GPIO_UNTIL;
        pv->gpio_rq.until.mask = dev_gpio_mask1;
        pv->gpio_rq.until.data = &pv->current_state;
        DEVICE_OP(&pv->gpio, request, &pv->gpio_rq);
      }
    break;

    case PB_GPIO_REQ_STATE_ACTIVE:
    /* Nothing to do */
    break;

    default:
      UNREACHABLE();
  }

  LOCK_RELEASE_IRQ(&dev->lock);

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
  if (device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER) != 0)
    goto err_mem;
  /* Start timer */
  device_start(&pv->timer.base);
  dev_timer_shift_sec(&pv->timer, &pv->shifta, &pv->shiftb, 0, 1, 1000);
  dev_timer_init_sec(&pv->timer, &pv->base_time, 0, 1, 1000);
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_DELAYED
  /* Init delay timer request */
  pv->delay_trq.pvdata = dev;
  dev_timer_rq_init(&pv->delay_trq, push_button_delay_timeout);
  /* Init delay request queue */
  dev_rq_queue_init(&pv->delay_queue);
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_REPEAT
  pv->repeat_trq.pvdata = dev;
  dev_timer_rq_init(&pv->repeat_trq, push_button_repeat_timeout);
#endif

#ifdef CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING
  pv->debounce_timeout = pv->base_time * CONFIG_DRIVER_PUSH_BUTTON_SOFT_DEBOUNCING_TIMING;
  pv->debounce_trq.pvdata = dev;
  pv->debounce_trq.delay = pv->debounce_timeout;
  dev_timer_rq_init(&pv->debounce_trq, push_button_lock_timeout);
#endif
  /* Init request queue */
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
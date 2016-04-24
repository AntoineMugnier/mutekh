
#include <device/class/timer.h>
#include <device/class/char.h>
#include <device/device.h>
#include <device/driver.h>
#include <mutek/mem_alloc.h>

struct devinit_test_pv_s
{
  struct device_timer_s timer;
  struct dev_timer_rq_s rq;
};

# ifdef CONFIG_DEVICE_INIT_ASYNC
static KROUTINE_EXEC(devinit_test_timeout)
{
  struct dev_timer_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, rq.kr);
  struct device_s *dev = rq->rq.pvdata;
  struct devinit_test_pv_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

#ifdef CONFIG_DEVICE_CLEANUP
  if (dev->status == DEVICE_INIT_DECLINE)
    {
      device_put_accessor(&pv->timer.base);
      mem_free(pv);
      device_async_cleanup_done(dev);
    }
  else
#endif
    {
      device_async_init_done(dev, 0);
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

static DEV_INIT(devinit_test_init)
{
  struct devinit_test_pv_s *pv;
  error_t err;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

# ifdef CONFIG_DEVICE_INIT_ASYNC
  err = -ENOENT;
  if (device_get_param_dev_accessor(dev, "timer", &pv->timer.base, DRIVER_CLASS_TIMER))
    goto err;

  err = -EINVAL;
  if (dev_timer_init_sec(&pv->timer, &pv->rq.delay, NULL, 5, 1))
    goto err_acc;

  pv->rq.rev = 0;
  pv->rq.rq.pvdata = dev;
  kroutine_init_deferred(&pv->rq.rq.kr, devinit_test_timeout);

  ensure(DEVICE_OP(&pv->timer, request, &pv->rq) == 0);
#endif

# ifdef CONFIG_DEVICE_INIT_PARTIAL
  /* makes timer API usable immediately */
  device_init_set_class(dev, 1);
# endif

# ifdef CONFIG_DEVICE_INIT_ASYNC
  return -EAGAIN;
# else
  return 0;
# endif

 err_acc:
  device_put_accessor(&pv->timer.base);
 err:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(devinit_test_cleanup)
{
  struct devinit_test_pv_s *pv = dev->drv_pv;

# ifdef CONFIG_DEVICE_INIT_ASYNC
  if (dev->status != DEVICE_INIT_DECLINE)
    DEVICE_OP(&pv->timer, request, &pv->rq);

  return -EAGAIN;
#else

  return 0;
#endif
}

#define devinit_test_use dev_use_generic

static DEV_CHAR_CANCEL(char_async_cancel)
{
  return -ENOTSUP;
}

static DEV_CHAR_REQUEST(char_async_request)
{
}

static DEV_TIMER_CANCEL(timer_async_cancel)
{
  return -ENOTSUP;
}

static DEV_TIMER_REQUEST(timer_async_request)
{
  return -ENOTSUP;
}

static DEV_TIMER_CONFIG(timer_async_config)
{
  return -ENOTSUP;
}

static DEV_TIMER_GET_VALUE(timer_async_get_value)
{
  return -ENOTSUP;
}

DRIVER_DECLARE(devinit_test_drv, 0,
               "MutekH device init test driver", devinit_test,
               DRIVER_CHAR_METHODS(char_async),
               DRIVER_TIMER_METHODS(timer_async));


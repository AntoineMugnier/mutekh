#include <assert.h>

#include <device/request.h>
#include <device/class/valio.h>
#include <device/valio/button.h>

#include <mutek/printk.h>
#include <mutek/kroutine.h>

static struct device_valio_s        pbdev;
static struct dev_valio_rq_s        pbrq;
static struct valio_button_update_s btn_upd;

static
KROUTINE_EXEC(push_button_pressed)
{
  struct dev_request_s *base = KROUTINE_CONTAINER(kr, struct dev_request_s, kr);
  struct dev_valio_rq_s *rq  = dev_valio_rq_s_cast(base);

  DEVICE_OP(&pbdev, request, rq);

  printk("pressed!\n");
}

void main()
{
  ensure(device_get_accessor_by_path(&pbdev.base, NULL, "btn btn*", DRIVER_CLASS_VALIO) == 0);

  pbrq.attribute = VALIO_BUTTON_PUSH;
  pbrq.type      = DEVICE_VALIO_WAIT_EVENT;
  pbrq.data      = &btn_upd;

  dev_valio_rq_init(&pbrq, &push_button_pressed);

  DEVICE_OP(&pbdev, request, &pbrq);

  while (1)
    ;
}


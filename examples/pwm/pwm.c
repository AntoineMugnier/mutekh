
#include <mutek/printk.h>

#include <device/class/pwm.h>

void main()
{
  struct device_pwm_s pwm;
  if (device_get_accessor_by_path(&pwm, 0, "/pwm2", DRIVER_CLASS_PWM))
    {
      printk("pwm: error, cannot get accessor.\n");
      goto err_accessor;
    }

#if 1
  struct dev_pwm_fract_s freq;
  freq.num   = 480000;
  freq.denom = 1;

  if (DEVICE_OP(&pwm, freq, &freq, NULL))
    {
      printk("pwm: error, cannot set PWM frequency.\n");
      goto err_accessor;
    }

  struct dev_pwm_fract_s duty;
  duty.num   = 1;
  duty.denom = 4;

  if (DEVICE_OP(&pwm, duty, 0, &duty, NULL))
    {
      printk("pwm: error, cannot set PWM duty cycle on channel 0.\n");
      goto err_accessor;
    }

  enum dev_pwm_polarity_e pol = DEV_PWM_POL_HIGH;
  if (DEVICE_OP(&pwm, polarity, 0, &pol, NULL))
    {
      printk("pwm: error, cannot set PWM polarity on channel 0.\n");
      goto err_accessor;
    }
#endif

  if (DEVICE_OP(&pwm, start_stop, 0 /* channel */, 1 /* start */, NULL))
    printk("pwm: error, cannot start PWM.\n");

err_accessor:
  while (1)
    ;
}


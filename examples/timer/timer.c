#include <mutek/printk.h>
#include <mutek/kroutine.h>

#include <hexo/types.h>

#include <device/class/timer.h>

/*
  TIMER:      Timer to test.
  REF_TIMER:  reference timer used to validate TIMER

  DELAY_UNIT: Unit used to computed requests delay (1=>s, 1000=>ms, 1000000=>us)
  MAX_DELAY:  Maximum delay used in requests (in DELAY_UNIT unit).

  SKEW_MAX:   maximum skew authorized between planned deadline and real
              effective deadline (calculed with REF_TIMER).
*/

#if defined(CONFIG_ARCH_CC26XX)
# define REF_TIMER   "timer0"
# define TIMER       "rtc"
# define SKEW_MAX    0xffff
# define DELAY_UNIT  10000
# define MAX_DELAY   100000
#elif defined(CONFIG_ARCH_NRF5X)
# define REF_TIMER   "timer1"
# define TIMER       "rtc1"
# define SKEW_MAX    0xffff
# define DELAY_UNIT  10000
# define MAX_DELAY   100000
#elif defined(CONFIG_ARCH_SOCLIB)
# define REF_TIMER    "/fdt/cpu@0"
# define TIMER        "/fdt/vci_rttimer*"
# define SKEW_MAX     0xffffff
# define DELAY_UNIT   10000
# define MAX_DELAY    10000
#else
# error Unsupported arch
#endif

#define DEBUG 0
#define RQ_NB 64
#define CANCEL

enum  rq_state_e
{
  TEST_TIMER_STATE_NEW_REQUEST,
  TEST_TIMER_STATE_WAIT_FOR_DEADLINE,
  TEST_TIMER_STATE_DEADLINE_REACHED
};

struct pvdata_s
{
  dev_timer_value_t ref_start;
  dev_timer_value_t calc_deadline;
  dev_timer_value_t ref_deadline;
  uint32_t          delay;
  uint32_t          id;
  enum rq_state_e   state;
};


struct pvdata_s       pvdata_g[RQ_NB];
struct dev_timer_rq_s request_g[RQ_NB];
struct device_timer_s ref_dev_g;
struct device_timer_s timer_dev_g;
struct kroutine_s     kcontrol_g;
#ifdef CANCEL
struct kroutine_s     kcancel_g;
#endif
uint32_t              ref_min_delay;
uint32_t              timer_min_delay;
dev_timer_value_t     larger_skew_g;
dev_timer_value_t     next_print;
dev_timer_delay_t     ref_sec;
uint32_t              rq_cnt_g;



static KROUTINE_EXEC(request_handler);



/******************************************************************************/


static inline void display_deadline(struct pvdata_s *pvdata,
                                    dev_timer_value_t *skew)
{
#if DEBUG == 0
  printk("[%u] delay:%9u skew:0x%08llx [0x%llx]\n",
          rq_cnt_g,
          pvdata->delay,
          *skew,
          larger_skew_g);
#elif DEBUG == 1
  printk("\e[33m%016llx [%4d] DEADLINE            skew: 0x%016llx\e[39m\n",
          pvdata->ref_deadline,
          pvdata->id,
          *skew);
#endif
}




static inline void display_request(struct pvdata_s *pvdata)
{
#if DEBUG == 1
  printk("%016llx [%4d] REQUEST    calc_deadline: 0x%016llx  delay: %9u\n",
          pvdata->ref_start,
          pvdata->id,
          pvdata->calc_deadline,
          pvdata->delay);
#endif
}




static inline void display_error(struct pvdata_s *pvdata,
                                 dev_timer_value_t *ref,
                                 dev_timer_value_t *skew)
{
  cpu_interrupt_disable();
  printk("-------------------- ERROR -------------------\n");
  printk("ref       =                0x%016llx\n", *ref);
  printk("\n");
  printk("SKEW_MAX  =                0x%016x\n", SKEW_MAX);
  printk("skew      =                0x%016llx\n", *skew);
  printk("\n");
  printk("  request [%d]\n", pvdata->id);
  printk("\n");
  printk("    pvdata:\n");
  printk("      ref_start:           0x%016llx\n", pvdata->ref_start);
  printk("      delay:               %u\n", pvdata->delay);
  printk("      calc_deadline:       0x%016llx\n", pvdata->calc_deadline);
  printk("      ref_deadline:        0x%016llx\n", pvdata->ref_deadline);
  printk("-----------------------------------------------\n");
}




static inline bool_t check_skew(struct pvdata_s *pvdata, dev_timer_value_t *ref,
                                dev_timer_value_t *skew)
{
  *skew = 0;
  if (pvdata->calc_deadline < *ref)
    *skew = *ref - pvdata->calc_deadline;

  if (*skew > larger_skew_g)
    larger_skew_g = *skew;

  if (*skew > SKEW_MAX)
    {
      display_error(pvdata, ref, skew);
      return 0;
    }
  return 1;
}




/******************************************************************************/


static inline uint32_t get_delay(void)
{
  static uint32_t modulo = MAX_DELAY;
  static uint32_t base_counter = 2;
  static uint32_t counter = 2;

  if (!counter)
    {
      if (modulo <= 10)
        {
          modulo = MAX_DELAY;
          base_counter = 2;
        }
      else
        {
          modulo /= 10;
          base_counter <<= 1;
        }
      counter = base_counter;
    }

  uint32_t delay = (rand() << 15) | rand();
  counter--;

  return delay % modulo;
}




static inline void new_request_handler(struct dev_timer_rq_s *timer_rq,
                                       struct pvdata_s *pvdata)
{
  /* get random delay */
  uint32_t delay = get_delay();
  if (delay == 0)
    delay++;

  /* get the current timer value */
  dev_timer_value_t timer_current;
  DEVICE_OP(&timer_dev_g, get_value, &timer_current, 0);

  /* get the current ref value */
  dev_timer_value_t ref_current;
  DEVICE_OP(&ref_dev_g, get_value, &ref_current, 0);

  /* set the deadline for the timer request */
  timer_rq->deadline = timer_current + (delay * timer_min_delay);

  /* save data for the reference timer */
  pvdata->ref_start = ref_current;
  pvdata->delay = delay;
  pvdata->calc_deadline = ref_current + (ref_min_delay * delay);
  pvdata->ref_deadline = 0;

  display_request(pvdata);

  pvdata->state = TEST_TIMER_STATE_WAIT_FOR_DEADLINE;
  dev_timer_rq_init_immediate(timer_rq, request_handler);
  error_t err = DEVICE_OP(&timer_dev_g, request, timer_rq);

  if (err)
    {
      if (err == -ETIMEDOUT)
        {
          printk("\e[34mInfo: rq %d ETIMEDOUT (delay = %u)\e[39m\n",
                  pvdata->id, delay);
        }
      pvdata->state = TEST_TIMER_STATE_NEW_REQUEST;
      dev_timer_rq_init_sched_switch(timer_rq, request_handler);
      kroutine_exec(&timer_rq->base.kr);
    }
  else
    {
      rq_cnt_g++;
#ifdef CANCEL
      if ((rq_cnt_g % RQ_NB) == 0)
        kroutine_exec(&kcancel_g);
#endif
    }
}




static inline void irq_handler(struct dev_timer_rq_s *timer_rq,
                               struct pvdata_s *pvdata)
{
  DEVICE_OP(&ref_dev_g, get_value, &pvdata->ref_deadline, 0);
  pvdata->state = TEST_TIMER_STATE_DEADLINE_REACHED;
  dev_timer_rq_init_sched_switch(timer_rq, request_handler);
  kroutine_exec(&timer_rq->base.kr);
}




static inline void deadline_handler(struct dev_timer_rq_s *timer_rq,
                                    struct pvdata_s *pvdata)
{
  dev_timer_value_t skew;

  if(!check_skew(pvdata, &pvdata->ref_deadline, &skew))
    abort();
  display_deadline(pvdata, &skew);

  /* ready to post new request */
  pvdata->state = TEST_TIMER_STATE_NEW_REQUEST;
  dev_timer_rq_init_sched_switch(timer_rq, request_handler);
  kroutine_exec(&timer_rq->base.kr);
}




static KROUTINE_EXEC(request_handler)
{
  struct dev_request_s  *rq = KROUTINE_CONTAINER(kr, *rq, kr);
  struct pvdata_s       *pvdata = (struct pvdata_s *)rq->pvdata;

  switch (pvdata->state)
    {
      case TEST_TIMER_STATE_NEW_REQUEST:
        new_request_handler((struct dev_timer_rq_s *)rq, pvdata);
        break;
      case TEST_TIMER_STATE_WAIT_FOR_DEADLINE:
        irq_handler((struct dev_timer_rq_s *)rq, pvdata);
        break;
      case TEST_TIMER_STATE_DEADLINE_REACHED:
        deadline_handler((struct dev_timer_rq_s *)rq, pvdata);
        break;
      default:
        printk("Error: request handler invalid state\n");
        abort();
    }
}


/******************************************************************************/


#ifdef CANCEL
static KROUTINE_EXEC(kcancel_handler)
{
  uint32_t id = rand() % RQ_NB;

  error_t err = DEVICE_OP(&timer_dev_g, cancel, &request_g[id]);

  switch (pvdata_g[id].state)
    {
      /* Check that unqueue rq return an error */
      case TEST_TIMER_STATE_NEW_REQUEST:
      case TEST_TIMER_STATE_DEADLINE_REACHED:
        if (!err)
          {
            printk("Error: Cancel unqueued request didn't return an error\n");
            abort();
          }
        break;
      case TEST_TIMER_STATE_WAIT_FOR_DEADLINE:
        if (!err)
          printk("\e[31mInfo: [%d] cancelled\e[39m\n", id);
        else
          {
            printk("Error: Cannot cancel [%d]\n", id);
            abort();
          }
        break;
    }
    pvdata_g[id].state = TEST_TIMER_STATE_NEW_REQUEST;
    kroutine_exec(&request_g[id].base.kr);
}
#endif



static KROUTINE_EXEC(kcontrol_handler)
{
  dev_timer_value_t skew;
  dev_timer_value_t ref_current;

  /* Control routine */
  uint32_t  id;
  for (id = 0; id < RQ_NB; id++)
    {
      DEVICE_OP(&ref_dev_g, get_value, &ref_current, 0);
      if (pvdata_g[id].state == TEST_TIMER_STATE_WAIT_FOR_DEADLINE)
        if(!check_skew(&pvdata_g[id], &ref_current, &skew))
          abort();
    }

  if (next_print < ref_current) {
    printk("Larger skew: %llx\n", larger_skew_g);
    next_print += ref_sec;
  }

  /* Re-run */
  kroutine_exec(&kcontrol_g);
}


/******************************************************************************/


static inline void run_requests(void)
{
  /* initialize the requests */
  memset(&pvdata_g, 0, sizeof(pvdata_g));
  memset(&request_g, 0, sizeof(request_g));

  uint32_t  id;

  for (id = 0; id < RQ_NB; id++)
    {
      pvdata_g[id].state = TEST_TIMER_STATE_NEW_REQUEST;
      pvdata_g[id].id = id;
      request_g[id].base.pvdata = &pvdata_g[id];
      dev_timer_rq_init_sched_switch(&request_g[id], request_handler);
    }

  for (id = 0; id < RQ_NB; id++)
    kroutine_exec(&request_g[id].base.kr);
}




void main(void)
{
  rq_cnt_g = 0;
  larger_skew_g = 0;
  memset(&timer_dev_g, 0, sizeof(timer_dev_g));
  memset(&ref_dev_g, 0, sizeof(ref_dev_g));

  if (DELAY_UNIT > 1000000000 || MAX_DELAY > 1000000000)
    {
      printk("Error: DELAY_UNIT and MAX_DELAY should be < 1000000000\n");
      abort();
    }

  /* get accessor for the timer under test and the for the reference timer */
  if (device_get_accessor_by_path(&timer_dev_g.base, NULL, TIMER, DRIVER_CLASS_TIMER))
    {
      printk("Error: cannot get accessor\n");
      abort();
    }
  if (device_get_accessor_by_path(&ref_dev_g.base, NULL, REF_TIMER, DRIVER_CLASS_TIMER))
    {
      printk("Error: cannot get accessor\n");
      abort();
    }

  /* for each timer, save the value for the unit min */
  if ((dev_timer_init_sec(&timer_dev_g, &timer_min_delay, 0, 1, DELAY_UNIT)))
    {
      printk("Error: cannot get timer unit min\n");
      abort();
    }

  if ((dev_timer_init_sec(&ref_dev_g, &ref_min_delay, 0, 1, DELAY_UNIT)))
    {
      printk("Error: cannot get ref unit min\n");
      abort();
    }

  if ((dev_timer_init_sec(&ref_dev_g, &ref_sec, 0, 1, 1)))
    {
      printk("Error: cannot get ref second\n");
      abort();
    }

  /* start the reference timer */
  if (device_start(&ref_dev_g.base))
    {
      printk("Error: cannot start ref timer\n");
      abort();
    }

  next_print = 0;
  run_requests();

  /* run control routine */
  kroutine_init_sched_switch(&kcontrol_g, kcontrol_handler);
  kroutine_exec(&kcontrol_g);

#ifdef CANCEL
  /* init cancel kroutine */
  kroutine_init_sched_switch(&kcancel_g, kcancel_handler);
#endif
}

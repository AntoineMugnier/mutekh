#include <mutek/printk.h>
#include <mutek/kroutine.h>

#include <hexo/types.h>

#include <device/class/timer.h>


#define DEBUG 0


#define CC26XX

#ifdef CC26XX

# define TIMER     "timer0"
# define REF_TIMER "cpu"
# define SKEW_MAX 0xffff

#else

/* SOCLIB */
# define TIMER     "/fdt/vci_rttimer*"
# define REF_TIMER "/fdt/cpu@0"
# define SKEW_MAX 0xffffffff

#endif



#define RQ_NB 16
#define MAX_DELAY_US 1000000000



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
  uint32_t          delay_us;
  uint32_t          id;
  enum rq_state_e   state;
};


struct pvdata_s       pvdata_g[RQ_NB];
struct dev_timer_rq_s request_g[RQ_NB];
struct device_timer_s ref_dev_g;
struct device_timer_s timer_dev_g;
struct kroutine_s     kcontrol_g;
uint32_t              ref_us_g;
uint32_t              timer_us_g;
uint32_t              min_delay_us_g;
dev_timer_value_t     larger_skew_g;
uint32_t              rq_cnt_g;
bool_t                cancel_flag_g;



static KROUTINE_EXEC(request_handler);



/******************************************************************************/


static inline void display_deadline(struct pvdata_s *pvdata, dev_timer_value_t *skew)
{
#if DEBUG == 0
  printk("rq %4d skew:0x%08llx delay:%10uus\n", pvdata->id, *skew, pvdata->delay_us);
#elif DEBUG == 1
  printk("\e[33m%016llx deadline %4d:  skew    : 0x%016llx\e[39m\n", pvdata->ref_deadline, pvdata->id, *skew);
#else
  printk("larger skew:0x%016llx  min delay:%10uus\n", larger_skew_g, min_delay_us_g);
#endif
}




static inline void display_request(struct pvdata_s *pvdata)
{
#if DEBUG == 1
  printk("%016llx request  %4d:  deadline: 0x%016llx  delay: %13uus\n",
          pvdata->ref_deadline,
          pvdata->id,
          pvdata->ref_deadline,
          pvdata->delay_us);
#endif
}




static inline void display_error(struct pvdata_s *pvdata, dev_timer_value_t *ref, dev_timer_value_t *skew)
{
  cpu_interrupt_disable();
  printk("-------------------- ERROR -------------------\n");
  printk("ref       =                0x%016llx\n", *ref);
  printk("\n");
  printk("SKEW_MAX  =                0x%016x\n", SKEW_MAX);
  printk("skew      =                0x%016x\n", *skew);
  printk("min delay =                %uus\n", min_delay_us_g);
  printk("\n");
  printk("  request [%d]\n", pvdata->id);
  printk("\n");
  printk("    pvdata:\n");
  printk("      ref_start:           0x%016llx\n", pvdata->ref_start);
  printk("      delay_us:            %u us\n", pvdata->delay_us);
  printk("      calc_deadline:       0x%016llx\n", pvdata->calc_deadline);
  printk("      ref_deadline:        0x%016llx\n", pvdata->ref_deadline);
  printk("-----------------------------------------------\n");
}




static inline bool_t check_skew(struct pvdata_s *pvdata, dev_timer_value_t *ref, dev_timer_value_t *skew)
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


#define BASE_DELAY_COUNTER (1 << 10)

/* get 1024 (BASE_DELAY_COUNTER) random delay modulo 10 */
/* 512 random delay modulo 100 */
/* etc... until modulo == MAX_DELAY_US */

static inline uint32_t get_delay(void)
{
  static uint32_t modulo = 10;
  static uint32_t base_counter = BASE_DELAY_COUNTER;
  static uint32_t counter = BASE_DELAY_COUNTER;

  uint32_t delay_us = (rand() << 15) | rand();

  if (!counter)
    {
      if (modulo == MAX_DELAY_US)
        {
          modulo = 10;
          base_counter = BASE_DELAY_COUNTER;
        }
      else
        {
          modulo *= 10;
          base_counter >>= 1;
        }
      counter = base_counter;
    }
  counter--;
  return delay_us % modulo;
}




static inline void new_request_handler(struct dev_timer_rq_s *timer_rq, struct pvdata_s *pvdata)
{
  /* get random delay (in us) */
  uint32_t delay_us = get_delay();
  if (delay_us < min_delay_us_g)
    delay_us += min_delay_us_g;

  /* get the current timer value */
  dev_timer_value_t timer_current;
  DEVICE_OP(&timer_dev_g, get_value, &timer_current, 0);

  /* get the current ref value */
  dev_timer_value_t ref_current;
  DEVICE_OP(&ref_dev_g, get_value, &ref_current, 0);

  /* set the deadline for the timer request */
  timer_rq->deadline = timer_current + (delay_us * timer_us_g);

  /* save data for the reference timer */
  pvdata->ref_start = ref_current;
  pvdata->delay_us = delay_us;
  pvdata->calc_deadline = ref_current + (ref_us_g * delay_us);
  pvdata->ref_deadline = 0;

  display_request(pvdata);

  pvdata->state = TEST_TIMER_STATE_WAIT_FOR_DEADLINE;
  kroutine_init(&timer_rq->rq.kr, request_handler, KROUTINE_IMMEDIATE);
  error_t err = DEVICE_OP(&timer_dev_g, request, timer_rq);

  if (err)
    {
      if (err == -ETIMEDOUT)
        {
          min_delay_us_g = ++delay_us;
          printk("\e[34mInfo: rq %d ETIMEDOUT (new min delay = %uus)\e[39m\n", pvdata->id, min_delay_us_g);
        }
      pvdata->state = TEST_TIMER_STATE_NEW_REQUEST;
      kroutine_init(&timer_rq->rq.kr, request_handler, KROUTINE_SCHED_SWITCH);
      kroutine_exec(&timer_rq->rq.kr, 0);
    }
  else
    {
      if (cancel_flag_g == 0 && (++rq_cnt_g % RQ_NB) == 0)
        cancel_flag_g = 1;
    }
}




static inline void irq_handler(struct dev_timer_rq_s *timer_rq, struct pvdata_s *pvdata)
{
  DEVICE_OP(&ref_dev_g, get_value, &pvdata->ref_deadline, 0);
  pvdata->state = TEST_TIMER_STATE_DEADLINE_REACHED;
  kroutine_init(&timer_rq->rq.kr, request_handler, KROUTINE_SCHED_SWITCH);
  kroutine_exec(&timer_rq->rq.kr, 0);
}




static inline void deadline_handler(struct dev_timer_rq_s *timer_rq, struct pvdata_s *pvdata)
{
  dev_timer_value_t skew;

  if(!check_skew(pvdata, &pvdata->ref_deadline, &skew))
    abort();
  display_deadline(pvdata, &skew);

  /* ready to post new request */
  pvdata->state = TEST_TIMER_STATE_NEW_REQUEST;
  kroutine_init(&timer_rq->rq.kr, request_handler, KROUTINE_SCHED_SWITCH);
  kroutine_exec(&timer_rq->rq.kr, 0);
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


static inline void cancel_request(uint32_t id)
{
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
          printk("\e[31mInfo: rq %d canceled\e[39m\n", id);
        else
          {
            printk("Error: Cannot cancel rq %d\n", id);
            abort();
          }
        break;
    }
    pvdata_g[id].state = TEST_TIMER_STATE_NEW_REQUEST;
    kroutine_exec(&request_g[id].rq.kr, 0);
}




static KROUTINE_EXEC(kcontrol_handler)
{
  dev_timer_value_t skew;

  dev_timer_value_t ref_current;
  DEVICE_OP(&ref_dev_g, get_value, &ref_current, 0);

  /* Control routine */
  uint32_t  id;
  for (id = 0; id < RQ_NB; id++)
    {

      if (pvdata_g[id].state == TEST_TIMER_STATE_WAIT_FOR_DEADLINE)
        if(!check_skew(&pvdata_g[id], &ref_current, &skew))
          abort();

    }

  if (cancel_flag_g)
    {
      cancel_flag_g = 0;
      cancel_request(rand() % RQ_NB);
    }
  /* Re-run */
  kroutine_exec(&kcontrol_g, 0);
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
      request_g[id].rq.pvdata = &pvdata_g[id];
      kroutine_init(&request_g[id].rq.kr, request_handler, KROUTINE_SCHED_SWITCH);
    }

  for (id = 0; id < RQ_NB; id++)
    kroutine_exec(&request_g[id].rq.kr, 0);
}




void main(void)
{
  min_delay_us_g = 1;
  rq_cnt_g = 0;
  cancel_flag_g = 0;
  larger_skew_g = 0;

  /* get accessor for the timer under test and the for the reference timer */
  if (device_get_accessor_by_path(&timer_dev_g, NULL, TIMER, DRIVER_CLASS_TIMER))
    {
      printk("Error: cannot get accessor\n");
      abort();
    }
  if (device_get_accessor_by_path(&ref_dev_g, NULL, REF_TIMER, DRIVER_CLASS_TIMER))
    {
      printk("Error: cannot get accessor\n");
      abort();
    }

  /* for each timer, save the value for 1us */
  dev_timer_init_sec(&timer_dev_g, &timer_us_g, 0, 1, 1000000);

#ifdef CC26XX
  ref_us_g = timer_us_g;
#else
  dev_timer_init_sec(&timer_dev_g, &ref_us_g, 0, 1, 1000000);
#endif

  /* start the reference timer */
  if (device_start(&ref_dev_g))
    {
      printk("Error: cannot start ref timer\n");
      abort();
    }

  run_requests();

  /* run control routine */
  kroutine_init(&kcontrol_g, kcontrol_handler, KROUTINE_SCHED_SWITCH);
  kroutine_exec(&kcontrol_g, 0);
}








#include <device/input.h>
#include <device/gpio.h>
#include <device/driver.h>
#include <device/device.h>

# include <mutek/scheduler.h>
# include <hexo/lock.h>

#include <stdio.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

extern struct device_s dev_mt5f;
extern struct device_s dev_gpio_piob;

lock_t lock;
struct sched_context_s * volatile ctx;

CONTAINER_TYPE(j_queue, CLIST,
struct joy_event_s
{
	devinput_ctrlid_t id;
	bool_t new_val;

	j_queue_entry_t queue_entry; /* used by driver to enqueue requests */
}, queue_entry);

CONTAINER_FUNC(j_queue, CLIST, static inline, j_queue);

static j_queue_root_t j_list;

static DEVINPUT_CALLBACK(joystick_moved)
{
	lock_spin(&lock);

	struct joy_event_s *n = malloc(sizeof(*n));
	n->id = id;
	n->new_val = value;
	j_queue_pushback(&j_list, n);

	if (ctx != NULL)
		sched_context_start(ctx);
	ctx = NULL;
	lock_release(&lock);
}

static DEVGPIO_IRQ(button_pressed)
{
	lock_spin(&lock);

	struct joy_event_s *n = malloc(sizeof(*n));
	n->id = (uintptr_t)private;

	switch ( event ) {
	default:
		n->new_val = 0;
		break;
	case GPIO_EDGE_RAISING:
	case GPIO_VALUE_UP:
		n->new_val = 1;
		break;
	}

	j_queue_pushback(&j_list, n);

	if (ctx != NULL)
		sched_context_start(ctx);
	ctx = NULL;
	lock_release(&lock);
}

const char *name[] = {
	"left",
	"down",
	"up",
	"right",
	"joy_button",
	"sw1",
	"sw2",
};

void *joystick_main(void *unused)
{
	j_queue_init(&j_list);

	dev_gpio_set_value(&dev_gpio_piob, 24, 1);
	dev_gpio_set_way(&dev_gpio_piob, 24, GPIO_WAY_INPUT);
	dev_gpio_assign_to_peripheral(&dev_gpio_piob, 24, 0);
	dev_gpio_register_irq(&dev_gpio_piob, 24, GPIO_EVENT_ALL, button_pressed, (void*)5);

	dev_gpio_set_value(&dev_gpio_piob, 25, 1);
	dev_gpio_set_way(&dev_gpio_piob, 25, GPIO_WAY_INPUT);
	dev_gpio_assign_to_peripheral(&dev_gpio_piob, 25, 0);
	dev_gpio_register_irq(&dev_gpio_piob, 25, GPIO_EVENT_ALL, button_pressed, (void*)6);

	dev_input_setcallback(&dev_mt5f, DEVINPUT_CTRLID_ALL, 0, joystick_moved, NULL);
	dev_input_setcallback(&dev_mt5f, DEVINPUT_CTRLID_ALL, 0, joystick_moved, NULL);

	while (1) {
		struct joy_event_s *n;

		CPU_INTERRUPT_SAVESTATE_DISABLE;
		lock_spin(&lock);
		n = j_queue_head(&j_list);

		if ( !n ) {
			ctx = sched_get_current();
			sched_context_stop_unlock(&lock);
			asm volatile("":::"memory");
			lock_spin(&lock);
		}
		
		n = j_queue_head(&j_list);

		if ( n )
			j_queue_remove(&j_list, n);

		lock_release(&lock);
		CPU_INTERRUPT_RESTORESTATE;

		if ( !n )
			continue;

		printf("Joystick event: %s %s\r\n", name[n->id], n->new_val?"pressed":"released");
		free(n);
	}
}

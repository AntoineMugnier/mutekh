
#include <pthread.h>
#include <mutek/printk.h>

#include <GL/gl.h>
#include <GL/vgafb.h>
#include "ui.h"

/* for struct device_s definition */
#include <drivers/fb/vga/fb-vga.h>
#include <device/device.h>
#include <device/driver.h>

extern struct device_s fb_dev;

void tkSwapBuffers(void)
{
	vgafb_swap_buffer();
}

int ui_loop(int argc, char **argv, const char *name)
{
	struct vgafb_context *ctx = NULL;

	if (vgafb_create_context(&ctx) != 0)
        goto err;

	/* can we assume fb_vga_init has already been called?
	 * 	(for now, we assume it is in hw_init.c) */

	/* setmode and ensure we are on the proper page */
	fb_vga_setmode(&fb_dev, 320, 200, 8, FB_PACK_INDEX);
    fb_vga_flippage(&fb_dev, 0);

	if (vgafb_make_current(ctx, &fb_dev) != 0)
        goto err;

	init();
	reshape(320,200);

	while(1)
	{
		/* we cannot handle key pressing */
		idle();
	}

err:
    if (ctx != NULL)
        vgafb_destroy_context(ctx);
	return 0;
}

int main(int argc, char **argv);

void* demo_vgafb(void *param)
{
	main(0, NULL);
}

void app_start()
{
    static pthread_t a;
    pthread_create(&a, NULL, demo_vgafb, NULL);
}


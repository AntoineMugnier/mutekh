#include <stdio.h>
#include <mutek/printk.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <vfs/vfs.h>

#include <drivers/fs/ramfs/ramfs.h>

#define NTHREAD 10

void random_vfs_actions();

struct vfs_fs_s * vfs_init()
{
	struct vfs_fs_s *root_fs;

    printk("init vfs... ");
	ramfs_open(&root_fs);
    printk("ok\n");

	return root_fs;
}

pthread_t thread[NTHREAD];
pthread_t main_thread;

void *vfs_stress(void *root_ptr)
{
	vfs_set_root(root_ptr);
	vfs_set_cwd(root_ptr);

	random_vfs_actions((uint32_t)pthread_self());

	vfs_set_root(NULL);
	vfs_set_cwd(NULL);
	return NULL;
}

void *_main(void *root_ptr)
{
	while (1) {
		size_t i;
		for ( i=0; i<NTHREAD; ++i )
			pthread_create(&thread[i], NULL, vfs_stress, root_ptr);
		for ( i=0; i<NTHREAD; ++i )
			pthread_join(thread[i], NULL);

		printk("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		printk("System still alive after all this...\n");
		printk("Tree:\n");
		vfs_dump(root_ptr);
		for ( i=0; i<100000; ++i )
			asm volatile("nop");
		printk("Going on...\n");
	}
	return NULL;
}

void app_start()
{
	struct vfs_fs_s *root = vfs_init();

	pthread_create(&main_thread, NULL, _main, root->root);
}

#include <stdio.h>
#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/device/enum/fdt/enum-fdt.h>

#include <vfs/vfs.h>
#include <vfs/vfs-private.h>
#include "cwd.h"

#define NTHREAD 3

CONTEXT_LOCAL struct vfs_node_s *cwd;

void random_vfs_actions();

static
struct vfs_node_s * _vfs_init(struct device_s *dev)
{
	struct vfs_node_s *root_node;

    printk("init vfs... ");
    error_t err = vfs_init(dev, VFS_VFAT_TYPE, 20, 20, &root_node);
    printk("%d\n", err);

	return root_node;
}

pthread_t thread[NTHREAD];
pthread_t main_thread;

void *vfs_stress(void *root_ptr)
{
    struct vfs_node_s *root = root_ptr;
    CONTEXT_LOCAL_SET(cwd, root);
    vfs_node_up(root);
	random_vfs_actions();
    vfs_node_down(CONTEXT_LOCAL_GET(cwd));
	return NULL;
}

void action_rmrf_inner(struct vfs_node_s *_cwd, const char *name);
error_t memory_allocator_stats(struct memory_allocator_region_s *region,
                               size_t *alloc_blocks,
                               size_t *free_size,
                               size_t *free_blocks);

void print_malloc_stats()
{
    size_t alloc_blocks;
    size_t free_size;
    size_t free_blocks;

    memory_allocator_stats(default_region,
                           &alloc_blocks, &free_size, &free_blocks);

    printk("Malloc stats: block: alloc: %d free: %d; %d bytes left\n",
           alloc_blocks, free_blocks, free_size);
}

void *_main(void *root_ptr)
{
	struct vfs_node_s *root = root_ptr;

    print_malloc_stats();

    size_t alloc;
    memory_allocator_stats(default_region,
                           &alloc, 0, 0);

	while (1) {
		size_t i;
		for ( i=0; i<NTHREAD; ++i )
			pthread_create(&thread[i], NULL, vfs_stress, root);
		for ( i=0; i<NTHREAD; ++i )
			pthread_join(thread[i], NULL);

        print_malloc_stats();

		printk("Cleaning up /...\n");
        action_rmrf_inner(root, ".");

		printk("System still alive after all this...\n");
        print_malloc_stats();

		for ( i=0; i<100000; ++i )
			asm volatile("nop");
//        memory_allocator_dump_used(default_region, alloc);
		printk("Going on...\n");
	}
	return NULL;
}

void app_start()
{
    extern struct device_s fdt_enum_dev;
    struct vfs_node_s *root = _vfs_init(enum_fdt_lookup(&fdt_enum_dev, "/block@1"));

	pthread_create(&main_thread, NULL, _main, root);
}

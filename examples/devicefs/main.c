#include <hexo/interrupt.h>
#include <drivers/device/icu/8259/icu-8259.h>
#include <drivers/device/block/ata/block-ata.h>
#include <drivers/device/block/partition/block-partition.h>
#include <hexo/device.h>
#include <device/driver.h>
#include <stdio.h>
#include <vfs/vfs.h>
#include <vfs/buffer_cache.h>
#include <pthread.h>

#ifdef CONFIG_DRIVER_FS_DEV
#include <drivers/fs/devfs/devfs.h>
#include <drivers/device/char/null/null.h>
#include <drivers/device/char/zero/zero.h>
#include <drivers/device/char/random/random.h>
#endif /* End DevFS */

#define NR_THREADS  1

extern struct device_s icu_dev;

struct vfs_node_s *root;
struct vfs_node_s *ms_n_cwd;

static pthread_t task[NR_THREADS];
static struct device_s ata;

void* thread_func(void *arg)
{
  ssize_t err = 0;
  struct device_s *drv0 = NULL;
  struct device_s *part1 = NULL;

  if ((drv0 = device_get_child(&ata, 0)) == NULL)
    {
      printf("Couldn't find first disk on system\n");
      while (1)
	continue;
    }

  if (block_partition_create(drv0, 0) == 0)
    {
      printf("Couldn't find partition\n");
      while (1)
	continue;
    }

  if ((part1 = device_get_child(drv0, 0)) == NULL)
    {
      printf("Couldn't find first partition on disk\n");
      while (1)
	continue;
    }

#ifdef CONFIG_VFS

  // Initialize VFS
  if ((err = vfs_init(part1, 1, 10, 10, &root))){
    printf("error while initializing VFSLib: %d\n",err);
    while (1)
      continue;
  }
  else
    printf("OK initializing VFSLib\n");

#ifdef CONFIG_DRIVER_FS_DEV

  // Initialize DevFS
  if ((err = devfs_init("/dev")))
    while (1)
      continue;

  struct device_s null_dev, zero_dev, random_dev;

  // Initialize devices
  device_init(&null_dev);
  device_init(&zero_dev);
  device_init(&random_dev);

  dev_null_init(&null_dev, NULL, NULL);
  dev_zero_init(&zero_dev, NULL, NULL);
  dev_random_init(&random_dev, NULL, NULL);

  // Create nodes
  devfs_register("tty", NULL, DEVFS_CHAR);
  devfs_register("null", &null_dev, DEVFS_CHAR);
  devfs_register("zero", &zero_dev, DEVFS_CHAR);
  devfs_register("random", &random_dev, DEVFS_CHAR);
  devfs_register("urandom", &random_dev, DEVFS_CHAR);
  devfs_register("hda", drv0, DEVFS_BLOCK);
  devfs_register("hda1", part1, DEVFS_BLOCK);

  // Remove nodes
  devfs_unregister("tty");
  devfs_unregister("urandom");
  devfs_unregister("null");
  devfs_unregister("random");

  // Clear DevFS
  devfs_destroy("/dev");

#endif /* End DevFS */
#endif /* End VFS */

  bc_dump(&bc);
  bc_sync(&bc,&freelist);
  printf("Finished\n");
  while(1) pthread_yield();
  return NULL;
}



int main()
{
  cpu_interrupt_enable();
  assert(cpu_interrupt_getstate());

  device_init(&ata);

  ata.addr[0] = 0x1f0;
  ata.addr[1] = 0x3f0;
  ata.irq = 14;

  controller_ata_init(&ata, &icu_dev, NULL);
  DEV_ICU_BIND(&icu_dev, &ata);

  int i;
  for(i=0;i<NR_THREADS;i++){
    printf("main: creating thread %d\n",i);

    if(pthread_create(&task[i],NULL,thread_func,(void *) i)){
      printf("error creating thread number: %d\n",i);
      while(1);
    }
  }

  printf("LEAVE\n");
  return 0;
}


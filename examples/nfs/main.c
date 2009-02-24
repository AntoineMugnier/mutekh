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

#ifdef CONFIG_DRIVER_FS_NFS
#include <drivers/fs/nfs/nfs.h>
#endif /* End NFS */

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
      printk("Couldn't find first disk on system\n");
      while (1)
	continue;
    }

  if (block_partition_create(drv0, 0) == 0)
    {
      printk("Couldn't find partition\n");
      while (1)
	continue;
    }

  if ((part1 = device_get_child(drv0, 0)) == NULL)
    {
      printk("Couldn't find first partition on disk\n");
      while (1)
	continue;
    }

#ifdef CONFIG_VFS

  // Initialize VFS
  if ((err = vfs_init(part1, 1, 10, 10, &root))){
    printk("error while initializing VFSLib: %d\n",err);
    while (1)
      continue;
  }
  else
    printk("OK initializing VFSLib\n");

#ifdef CONFIG_DRIVER_FS_NFS

  if_up("eth0");

  if (dhcp_client("eth0"))
    while (1)
      continue;

  if (rarp_client("eth0"))
    while (1)
      continue;

  // Initialize NFS
  // to 192.168.1.237 -> 0xC0A801ED
  // to 10.0.2.2      -> 0x0A000202
  if ((err = nfs_mount("/nfs", 0xC0A801ED)))
    printk("error while initializing NFS: %d\n",err);

#endif /* End NFS */
#endif /* End VFS */

  bc_dump(&bc);
  bc_sync(&bc,&freelist);
  printk("Finished\n");
  while(1) pthread_yield();
  return NULL;
}



int main()
{
  cpu_interrupt_enable();
  assert(cpu_is_interruptible());

  device_init(&ata);

  ata.addr[0] = 0x1f0;
  ata.addr[1] = 0x3f0;
  ata.irq = 14;

  controller_ata_init(&ata, &icu_dev, NULL);
  DEV_ICU_BIND(&icu_dev, &ata);

  int i;
  for(i=0;i<NR_THREADS;i++){
    printk("main: creating thread %d\n",i);

    if(pthread_create(&task[i],NULL,thread_func,(void *) i)){
      printk("error creating thread number: %d\n",i);
      while(1);
    }
  }

  printk("LEAVE\n");
  return 0;
}


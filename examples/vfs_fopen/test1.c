
#include <stdio.h>
#include <pthread.h>

#include <device/device.h>
#include <device/driver.h>

#include <drivers/device/block/file-emu/block-file-emu.h>

#include <vfs/vfs.h>

struct device_s bd_dev;

extern struct vfs_node_s *vfs_root;
extern struct stream_ops_s vfs_ops;

pthread_t a;

void* cat(void* arg)
{
#if defined(CONFIG_VFS_LIBC_STREAM)
    printf("init vfs... ");
    if (vfs_init(&bd_dev, VFS_VFAT_TYPE, 20, 20, NULL) != 0)
    {
        printf("not ok\n");
        abort();
    }

    printf("ok\n");
#endif

    FILE* f;
    uint8_t buffer[100];

    if ((f = fopen(arg, "r")) == NULL)
    {
        printf("Can't open file %s\n", arg);
        abort();
    }
#if 0
    while (fgets(buffer, sizeof(buffer), f))
      fputs(buffer, stdout);
    fflush(stdout);
#else
    while (!feof(f))
    {
      fread(buffer, sizeof(uint8_t), 10, f);
      size_t i;
      for (i = 0; i < 10; i++)
	printf("%c", buffer[i]);
    }
#endif

    fclose(f);

    fprintf(stderr, "bye...\nbye");
}

void app_start()
{
    device_init(&bd_dev);
    block_file_emu_init(&bd_dev, "img.bin");

#if 1
    pthread_create(&a, NULL, cat, "DESC.C");
#else
    pthread_create(&a, NULL, cat, "A.X");
#endif
}


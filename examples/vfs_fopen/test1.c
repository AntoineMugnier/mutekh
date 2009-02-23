
#include <stdio.h>
#include <pthread.h>

#include <hexo/device.h>
#include <device/driver.h>

#include <drivers/device/block/file-emu/block-file-emu.h>

#include <vfs/vfs.h>

struct device_s bd_dev;

extern struct vfs_node_s *vfs_root;
extern struct stream_ops_s vfs_ops;

pthread_t a;

void* cat(void* arg)
{
#if defined(CONFIG_LIBC_STREAM_VFS)
    printf("init vfs... ");
    if (vfs_init(&bd_dev, VFS_VFAT_TYPE, 20, 20, &vfs_root) != 0)
    {
        printf("not ok\n");
        abort();
    }
    stream_fops = &vfs_ops;
    printf("ok\n");
#endif

    FILE* f;
    uint8_t buffer[10];

    if ((f = fopen(arg, "r")) == NULL)
    {
        printf("Can't open file %s\n", arg);
        abort();
    }
#if 1
    while (fgets(buffer, sizeof(buffer), f))
        printf("%s", buffer);
#else
    while (fread(buffer, sizeof(uint8_t), 10, f))
    {
        size_t i;
        for (i = 0; i < 10; i++)
            printf("%x", buffer[i]);
    }
#endif

    fclose(f);

    printf("bye...\n");
}

int main()
{
    device_init(&bd_dev);
    block_file_emu_init(&bd_dev, NULL, "img.bin");

#if 1
    pthread_create(&a, NULL, cat, "DESC.C");
#else
    pthread_create(&a, NULL, cat, "A.X");
#endif
}


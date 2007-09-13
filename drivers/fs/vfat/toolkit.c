#include <stdlib.h>
#include <string.h>
#include <hexo/alloc.h>
#include "vfat.h"

void vfat_tk_dump_bpb(struct vfat_bpb_s *bpb)
{
  printf("bpb->BS_jmpBoot\t\t%P\n", bpb->BS_jmpBoot, 3);
  printf("bpb->BS_OEMName\t\t%P\n", bpb->BS_OEMName, 11);
  printf("bpb->BPB_BytsPerSec\t%d\n", bpb->BPB_BytsPerSec);
  printf("bpb->BPB_SecPerClus\t%d\n", bpb->BPB_SecPerClus);
  printf("bpb->BPB_RsvdSecCnt\t%d\n", bpb->BPB_RsvdSecCnt);
  printf("bpb->BPB_NumFATs\t%d\n", bpb->BPB_NumFATs);
  printf("bpb->BPB_RootEntCnt\t%d\n", bpb->BPB_RootEntCnt);
  printf("bpb->BPB_TotSec16\t%d\n", bpb->BPB_TotSec16);
  printf("bpb->BPB_Media\t\t%d\n", bpb->BPB_Media);
  printf("bpb->BPB_FATSz16\t%d\n", bpb->BPB_FATSz16);
  printf("bpb->BPB_SecPerTrk\t%d\n", bpb->BPB_SecPerTrk);
  printf("bpb->BPB_NumHeads\t%d\n", bpb->BPB_NumHeads);
  printf("bpb->BPB_HiddSec\t%d\n", bpb->BPB_HiddSec);
  printf("bpb->BPB_TotSec32\t%d\n", bpb->BPB_TotSec32);
  printf("bpb->BPB_FATSz32\t%d\n", bpb->BPB_FATSz32);
  printf("bpb->BPB_ExtFlags\t%d\n", bpb->BPB_ExtFlags);
  printf("bpb->BPB_VFVer\t\t%d\n", bpb->BPB_VFVer);
  printf("bpb->BPB_RootClus\t%d\n", bpb->BPB_RootClus);
  printf("bpb->BPB_FSInfo\t\t%d\n", bpb->BPB_FSInfo);
  printf("bpb->BPB_BkBootSec\t%d\n", bpb->BPB_BkBootSec);
  printf("bpb->BPB_Reserved\t%P\n", bpb->BPB_Reserved, 12);
  printf("bpb->BS_DrvNum\t\t%d\n", bpb->BS_DrvNum);
  printf("bpb->BS_Reserved1\t%d\n", bpb->BS_Reserved1);
  printf("bpb->BS_BootSig\t\t%d\n", bpb->BS_BootSig);
  printf("bpb->BS_VolID\t\t%d\n", bpb->BS_VolID);
  printf("bpb->BS_VolLab\t\t%P\n", bpb->BS_VolLab);
  printf("bpb->BS_FilSysType\t%P\n", bpb->BS_FilSysType, 8);
}

void vfat_tk_dump_context(struct vfat_disk_context_s *dsk_dev)
{
  printf("dsk->fat_begin_lba\t\t%d\n", dsk_dev->fat_begin_lba);
  printf("dsk->bytes_per_sector\t%d\n", dsk_dev->bytes_per_sector);
  printf("dsk->cluster_begin_lba\t%d\n", dsk_dev->cluster_begin_lba);
  printf("dsk->sectors_per_cluster\t%d\n", dsk_dev->sectors_per_cluster);
  printf("dsk->rootdir_first_cluster\t%d\n", dsk_dev->rootdir_first_cluster);
}

extern char vfat_image[];
asm ("vfat_image:");
asm (".incbin \"/dsk/l1/misc/FAT.img\"");

static inline error_t pv_vfat_init_ramdisk_image(struct device_s *device,
					  uint16_t blk_sz,
					  uint32_t blk_cnt)
{
  uint32_t i;
  error_t errno;

  struct dev_block_rq_s rq;

  uint8_t *data[1];
  rq.lba = 0;
  rq.count = 1;
  rq.data = data;

  if (dev_block_getparams(device)->blk_size != blk_sz ||
      dev_block_getparams(device)->blk_count < blk_cnt)
    {
      printf("- %d : %d\n", dev_block_getparams(device)->blk_size, blk_sz);
      printf("- %d : %d\n", dev_block_getparams(device)->blk_count, blk_cnt);
      printf("- warning: device block size/count mismatch.\n");

      goto err;
    }

  for (i = 0; i < blk_cnt; i++)
    {
      rq.lba = i;
      rq.count = 1;

      data[0] = &vfat_image[blk_sz * i];
      if ((errno = dev_block_wait_write(device, &rq)) != 0)
	goto err;
    }

  printf("- vfat image written on disk.\n");
  return 0;

 err:
  printf("- error\n");
  return -1;
}

error_t vfat_tk_init_ramdisk(struct device_s *device)
{
  return (pv_vfat_init_ramdisk_image(device, 512, 65536));
}


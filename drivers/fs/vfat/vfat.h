#ifndef __VFAT_H__
#define __VFAT_H__

#include <drivers/device/block-ramdisk/block-ramdisk.h>
#include <hexo/device/block.h>
#include <hexo/device.h>
#include <hexo/driver.h>
#include <hexo/fs.h>

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_clist.h>

typedef uint32_t vfat_cluster_t;

struct vfat_bpb_s
{
  uint8_t	BS_jmpBoot[3];
  uint8_t	BS_OEMName[8];
  uint16_t	BPB_BytsPerSec;
  uint8_t	BPB_SecPerClus;
  uint16_t	BPB_RsvdSecCnt;
  uint8_t	BPB_NumFATs;
  uint16_t	BPB_RootEntCnt;
  uint16_t	BPB_TotSec16;
  uint8_t	BPB_Media;
  uint16_t	BPB_FATSz16;
  uint16_t	BPB_SecPerTrk;
  uint16_t	BPB_NumHeads;
  uint32_t	BPB_HiddSec;
  uint32_t	BPB_TotSec32;
  uint32_t	BPB_FATSz32;
  uint16_t	BPB_ExtFlags;
  uint16_t	BPB_VFVer;
  uint32_t	BPB_RootClus;
  uint16_t	BPB_FSInfo;
  uint16_t	BPB_BkBootSec;
  uint8_t	BPB_Reserved[12];
  uint8_t	BS_DrvNum;
  uint8_t	BS_Reserved1;
  uint8_t	BS_BootSig;
  uint32_t	BS_VolID;
  uint8_t	BS_VolLab[11];
  uint8_t	BS_FilSysType[8];
} __attribute__ ((packed));

struct vfat_DirEntry_s
{
  uint8_t	DIR_Name[11];
  uint8_t	DIR_Attr;
  uint8_t	DIR_NTRes;
  uint8_t	DIR_CrtTimeTenth;
  uint16_t	DIR_CrtTime;
  uint16_t	DIR_CrtDate;
  uint16_t	DIR_LstAccTime;
  uint16_t	DIR_FstClusHI;
  uint16_t	DIR_WrtTime;
  uint16_t	DIR_WrtDate;
  uint16_t	DIR_FstClusLO;
  uint32_t	DIR_FileSize;
} __attribute__ ((packed));

struct vfat_LongDirEntry_s
{
  uint8_t	LDIR_Ord;
  uint8_t	LDIR_Name1[10];
  uint8_t	LDIR_Attr;
  uint8_t	LDIR_Type;
  uint8_t	LDIR_Chksum;
  uint8_t	LDIR_Name2[12];
  uint16_t	LDIR_FstClusLO;
  uint32_t	LDIR_Name3;
} __attribute__ ((packed));

#define VFAT_ATTR_READ_ONLY	0x01
#define VFAT_ATTR_HIDDEN	0x02
#define VFAT_ATTR_SYSTEM	0x04
#define VFAT_ATTR_VOLUME_ID	0x08
#define VFAT_ATTR_DIRECTORY	0x10
#define VFAT_ATTR_ARCHIVE	0x20
#define VFAT_ATTR_LONG_NAME	0x0F




/* 
   vfat_disk_context_s extends the basic fs_disk_context_s structure
   and holds vfat specific datas retreived from the partition boot block.
*/
struct vfat_disk_context_s
{
  struct fs_disk_context_s	fs;
  dev_block_lba_t		fat_begin_lba;
  uint16_t			bytes_per_sector; /* should be 512 */
  uint32_t			bytes_per_cluster;
  dev_block_lba_t		cluster_begin_lba;
  uint32_t			sectors_per_cluster; /* [1,2,4 ... 4096]*/
  vfat_cluster_t		rootdir_first_cluster;
};

/*
  vfat_ent_dir_s are used to store directory informations in
  a fs_entity_s structure,
  They can be accessed throu the fs_entity_s::pv pointer,
  providing fs_entity_s::flags has FS_ENT_DIRECTORY set.
*/

struct vfat_ent_dir_s
{
  vfat_cluster_t		clus_idx;
  vfat_cluster_t		next_cluster;
  size_t			clus_offset;
};

/*
  vfat_ent_dir_s are used to store directory informations in
  a fs_entity_s structure,
  They can be accessed throu the fs_entity_s::pv pointer,
  providing fs_entity_s::flags has FS_ENT_FILE set.
*/
struct vfat_ent_file_s
{
  vfat_cluster_t		clus_idx;
};

#define VFAT_CLUS_TO_BLK(dev, clus_offset) ((clus_offset - 2) * dev->sectors_per_cluster)
#define VFAT_CLUSTER_SIZE(dev) ((dev)->sectors_per_cluster * (dev)->bytes_per_sector)

/* context.c */
struct fs_disk_context_s *vfat_context_create(struct device_s *device);
error_t vfat_context_destroy(struct fs_disk_context_s *disk_context);

/* toolkit.c */
error_t vfat_tk_init_ramdisk(struct device_s *device);
void vfat_tk_dump_context(struct vfat_disk_context_s *dsk_ctx);
void vfat_tk_dump_bpb(struct vfat_bpb_s *bpb);

/* access.c */
error_t vfat_get_root_info(struct fs_disk_context_s *disk_context,
			   struct fs_entity_s *file);

error_t vfat_get_entity_info(struct fs_disk_context_s *disk_context,
			     struct fs_entity_s *parent,
			     struct fs_entity_s *file,
			     char *fname);

#ifdef CONFIG_VFS
#include <vfs/vfs.h>
static const struct vfs_drv_s vfs_vfat_drv = 
  {
    .get_entity_info = vfat_get_entity_info,
    .get_root_info = vfat_get_root_info,
    .context_create = vfat_context_create,
    .context_destroy = vfat_context_destroy,
  };
#endif

#endif

#ifndef __VFAT_H__
#define __VFAT_H__

#include <drivers/device/block/ramdisk/block-ramdisk.h>
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
  uint16_t	LDIR_Name1[5];
  uint8_t	LDIR_Attr;
  uint8_t	LDIR_Type;
  uint8_t	LDIR_Chksum;
  uint16_t	LDIR_Name2[6];
  uint16_t	LDIR_FstClusLO;
  uint16_t	LDIR_Name3[2];
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
  uint_fast32_t			fat_blk_count;
  uint8_t			*fat_map;
  uint_fast16_t			bytes_per_sector; /* should be 512 */
  uint_fast32_t			bytes_per_cluster;
  dev_block_lba_t		cluster_begin_lba;
  uint_fast16_t			sectors_per_cluster; /* [1,2,4 ... 4096]*/
  vfat_cluster_t		rootdir_first_cluster;
};

/*
  vfat_handle_s
*/
struct vfat_handle_s
{
  vfat_cluster_t	clus_idx;	/* current cluster index */
  vfat_cluster_t	next_cluster;	/* next cluster in chain (index) */
  size_t		clus_offset;	/* offset in cluster (bytes) */
#ifdef CONFIG_DRIVER_VFAT_BLOCK_CACHE
  uint8_t		*cluster;	/* cluster buffer (optional) */
#endif
};

/*
  vfat_entity_s
*/
struct vfat_entity_s
{
  vfat_cluster_t		clus_idx; // First cluster ID
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

/* entity.c */
error_t vfat_get_root_info(struct fs_disk_context_s *disk_context,
			   struct fs_entity_s *entity);

error_t vfat_get_entity_info(struct fs_disk_context_s *disk_context,
			     struct fs_entity_s *parent,
			     struct fs_entity_s *entity,
			     char *name);

error_t vfat_init_entity(struct fs_entity_s *entity);
error_t vfat_destroy_entity(struct fs_entity_s *entity);

/* handle.c */
error_t vfat_init_handle(struct fs_disk_context_s *disk_context,
			 struct fs_handle_s *handle,
			 struct fs_entity_s *entity);

void vfat_release_handle(struct fs_handle_s *handle);

/* directory.c */
error_t vfat_find_first_file(struct fs_disk_context_s *disk_context,
			     struct fs_handle_s *parent,
			     struct fs_entity_s *entity,
			     char *filename);

error_t vfat_find_next_file(struct fs_disk_context_s *disk_context,
			    struct fs_handle_s *parent,
			    struct fs_entity_s *entity,
			    char *filename);

/* access.c */
error_t vfat_read(struct fs_disk_context_s *disk_context,
		  struct fs_handle_s *handle,
		  uint8_t *buffer,
		  uint32_t size);

/* VFS stuff */
#define VFS_EXPORT

#ifdef CONFIG_VFS
#include <vfs/vfs.h>
static const struct vfs_drv_s vfs_vfat_drv = 
  {
    /* entity */
    .get_root_info	=	vfat_get_root_info,
    .get_entity_info	=	vfat_get_entity_info,
    .init_entity	=	vfat_init_entity,
    .destroy_entity	=	vfat_destroy_entity,

    /* directory */
    .find_first_file	=	vfat_find_first_file,
    .find_next_file	=	vfat_find_next_file,

    /* context */
    .context_create	=	vfat_context_create,
    .context_destroy	=	vfat_context_destroy,

    /* handle */
    .init_handle	=	vfat_init_handle,
    .release_handle	=	vfat_release_handle,

    /* io */
    .read		=	vfat_read,
  };
#endif

#endif

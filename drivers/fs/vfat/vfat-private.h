/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    UPMC / LIP6 / SOC (c) 2008
    Copyright Ghassan Almaless <ghassan.almaless@gmail.com>
*/

#ifndef __VFAT_PRIVATE_H__
#define __VFAT_PRIVATE_H__

#define VFAT_INSTRUMENT 1

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


#define VFAT_ATTR_READ_ONLY	0x01
#define VFAT_ATTR_HIDDEN	0x02
#define VFAT_ATTR_SYSTEM	0x04
#define VFAT_ATTR_VOLUME_ID	0x08
#define VFAT_ATTR_DIRECTORY	0x10
#define VFAT_ATTR_ARCHIVE	0x20
#define VFAT_ATTR_LONG_NAME	0x0F


struct device_s;
struct rwlock_s;
struct bc_buffer_s;
struct bc_request_s;

typedef uint32_t vfat_cluster_t;
typedef uint32_t vfat_sector_t;
typedef uint32_t vfat_offset_t;


struct vfat_context_s
{
  struct device_s *dev;
  struct rwlock_s lock;
  vfat_sector_t fat_begin_lba;
  vfat_sector_t fat_blk_count;
  uint_fast16_t bytes_per_sector;
  uint_fast32_t	bytes_per_cluster;
  vfat_cluster_t cluster_begin_lba;
  uint_fast16_t	sectors_per_cluster;
  vfat_cluster_t rootdir_first_cluster;
  vfat_sector_t last_allocated_sector;
  vfat_sector_t last_allocated_index;
};

struct vfat_node_s
{
  uint_fast8_t flags;
  vfat_cluster_t parent_cluster;
  vfat_cluster_t node_cluster;
  vfat_sector_t entry_sector;
  uint_fast16_t entry_index;
};

struct vfat_file_s
{
  struct vfat_context_s *ctx;
  vfat_cluster_t  node_cluster;
  vfat_cluster_t  current_cluster;
  vfat_offset_t	  current_offset;
};


struct vfat_entry_request_s
{
  struct vfat_context_s *ctx;
  vfat_cluster_t parent_cluster;
  char *entry_name;
  struct vfat_DirEntry_s *entry;
  vfat_sector_t *entry_sector;
  vfat_sector_t *entry_index;
};

#define VFAT_CONVERT_CLUSTER(ctx,cluster) (ctx)->cluster_begin_lba + \
					  ((cluster) - 2) * (ctx)->sectors_per_cluster

error_t vfat_locate_entry(struct vfat_entry_request_s *rq);

struct bc_request_s* vfat_read_sectors(struct vfat_context_s *ctx, 
				       struct bc_request_s *request,
				       vfat_sector_t first_sector,
				       uint_fast8_t count);

error_t vfat_write_sector(struct vfat_context_s *ctx,
			  vfat_sector_t sector,
			  uint8_t *content);

error_t vfat_query_fat(struct vfat_context_s *ctx,
		       vfat_cluster_t cluster_index, 
		       vfat_cluster_t *next_cluster_index);

error_t vfat_alloc_fat_entry(struct vfat_context_s *ctx,
			     vfat_cluster_t *new_cluster);

error_t vfat_extend_cluster(struct vfat_context_s *ctx, 
			    vfat_cluster_t current_cluster,
			    vfat_cluster_t *next_cluster);

error_t vfat_free_fat_entry(struct vfat_context_s *ctx, 
			    vfat_cluster_t start_cluster);

inline void vfat_getshortname(char *from, char *to);


#ifdef CONFIG_DRIVER_FS_VFAT_INSTRUMENT
extern uint_fast32_t blk_rd_count;
extern uint_fast32_t rd_count;
extern uint_fast32_t wr_count;
#endif


#endif

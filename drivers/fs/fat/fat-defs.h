/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; version 2.1 of the
  License.
    
  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
    
  You should have received a copy of the GNU Lesser General Public
  License along with MutekH; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#ifndef _FAT_DEFS_H_
#define _FAT_DEFS_H_

#include <hexo/types.h>
#include "fat-types.h"

struct fat_bpb_s
{
	uint8_t jmpboot[3];
	uint8_t oemname[8];
	uint16_t byte_per_sector;
	uint8_t sector_per_cluster;
	uint16_t reserved_sect_count;
	uint8_t fat_count;
	uint16_t root_dirent_count;
	uint16_t total_sector_count16;
	uint8_t media;
	uint16_t fat_size;
	uint8_t drive_info[8];
	uint32_t total_sector_count32;

	/* Fat 12/16 specific from here */
	uint8_t drive_number;
	uint8_t nt_reserved;
	uint8_t boot_sig;
	uint8_t volume_id[4];
	uint8_t volume_label[11];
	uint8_t volume_type[8];
} __attribute__((packed));

union fat_dirent_u
{
  struct fat_dirent_83_s
  {
    char       name[11];
    uint8_t    attr;
    uint8_t    ntres;
    uint8_t    create_time_tenth;
    uint16_t   create_time;
    uint16_t   create_date;
    uint16_t   access_date;
    uint16_t   clust_hi;
    uint16_t   update_time;
    uint16_t   update_date;
    uint16_t   clust_lo;
    uint32_t   file_size;
  } __attribute__ ((packed)) old;

  struct fat_dirent_lfn_s
  {
    uint8_t    id;
    uint16_t   name0_4[5];
    uint8_t    attr;
    uint8_t    res0;        // zero
    uint8_t    chksum_83;
    uint16_t   name5_10[6];
    uint16_t   res1;        // zero
    uint16_t   name11_12[2];
  } __attribute__ ((packed)) lfn;

  char raw[32];
};

#define ATTR_READ_ONLY 0x1
#define ATTR_HIDDEN    0x2
#define ATTR_SYSTEM    0x4
#define ATTR_VOLUME_ID 0x8
#define ATTR_LFN       0xf
#define ATTR_DIRECTORY 0x10
#define ATTR_ARCHIVE   0x20

#define FAT_DIR_ENTRY_FREE 0xe5
#define FAT_DIR_ENTRY_E5   0x05
#define FAT_DIR_ENTRY_LAST 0x0

#define CLUSTER_MASK   ((cluster_t)0xffff)
#define CLUSTER_END     ((cluster_t)0xffff)

#define FAT_83_NAMELEN 13

#endif

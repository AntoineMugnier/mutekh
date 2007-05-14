#include <hexo/alloc.h>
#include <vfs/vfs.h>

struct vfs_node_s vfs_root_g;

struct fs_file_s *pv_test_get_file_info(struct fs_file_s *parent, char *name)
{
  struct fs_file_s *file = mem_alloc(sizeof(*file), MEM_SCOPE_SYS);

  file->name = strdup(name);
  return file;
};

int_fast32_t pv_test_open()
{
  printf("open\n");
  return 0;
}

void pv_test_close()
{
  printf("close\n");
}

int_fast32_t pv_test_read()
{
  printf("read\n");
  return 0;
}

int_fast32_t pv_test_write()
{
  printf("write\n");
  return 0;
}

error_t vfs_init(void)
{
  struct vfs_fs_inst_s *fs_inst = mem_alloc(sizeof(*fs_inst), MEM_SCOPE_SYS);
  struct vfs_fs_s *fs = mem_alloc(sizeof(*fs), MEM_SCOPE_SYS);

  memset(&vfs_root_g, 0x0, sizeof(vfs_root_g));
  memset(fs_inst, 0x0, sizeof(*fs_inst));
  memset(fs, 0x0, sizeof(*fs));

  fs->get_file_info = pv_test_get_file_info;
  fs->open = pv_test_open;
  fs->close = pv_test_close;
  fs->read = pv_test_read;
  fs->write = pv_test_write;

  fs_inst->mpoint = &vfs_root_g;
  fs_inst->drv = fs;

  vfs_root_g.refcount = 2;
  vfs_root_g.file = fs->get_file_info(NULL, "/");
  vfs_root_g.fs_inst = fs_inst;

  vfs_node_func_init(&vfs_root_g.children);
  //  vfs_dbg_print_node("vfs_root_g", &vfs_root_g);
  return 0;
}

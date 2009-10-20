#ifndef FDT_READ_H_
#define FDT_READ_H_

#include <hexo/types.h>
#include <hexo/error.h>

struct fdt_walker_s;
struct fdt_walker_state_s;

#define FDT_ON_NODE_ENTRY_FUNC(x) bool_t (x)(						   \
		void *private,												   \
		const struct fdt_walker_state_s *state,						   \
		const char *name)

#define FDT_ON_NODE_LEAVE_FUNC(x) void (x)(void *private)

#define FDT_ON_NODE_ATTR_FUNC(x) void (x)(							   \
		void *private,												   \
		const struct fdt_walker_state_s *state,						   \
		const char *name,											   \
		const void *data,											   \
		size_t datalen)

#define FDT_ON_MEM_RESERVE_FUNC(x) void (x)(						   \
		void *private,												   \
		uint64_t addr,												   \
		uint64_t size)

typedef FDT_ON_NODE_ENTRY_FUNC(fdt_on_node_entry_func_t);
typedef FDT_ON_NODE_LEAVE_FUNC(fdt_on_node_leave_func_t);
typedef FDT_ON_NODE_ATTR_FUNC(fdt_on_node_attr_func_t);
typedef FDT_ON_MEM_RESERVE_FUNC(fdt_on_mem_reserve_func_t);

struct fdt_walker_s
{
	void *private;
	fdt_on_node_entry_func_t *on_node_entry;
	fdt_on_node_leave_func_t *on_node_leave;
	fdt_on_node_attr_func_t *on_node_attr;
	fdt_on_mem_reserve_func_t *on_mem_reserve;
};

error_t fdt_walk_blob(const void *blob, struct fdt_walker_s *walker);

#endif

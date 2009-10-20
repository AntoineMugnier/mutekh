
#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/endian.h>

#include <fdt/reader.h>

#include <mutek/printk.h>


#include "fdt_internals.h"


#include <string.h>

struct fdt_walker_state_s
{
	const void *blob;
	const char *string_table;
	const uint32_t *ptr;
};

static bool_t nop()
{
	return 0;
}

static struct fdt_walker_s null_walker =
{
	.private = NULL,
	.on_node_entry = (fdt_on_node_entry_func_t*)nop,
	.on_node_leave = (fdt_on_node_leave_func_t*)nop,
	.on_node_attr = (fdt_on_node_attr_func_t*)nop,
};

void fdt_skip_str(struct fdt_walker_state_s *state)
{
	uint32_t data;
	do {
		data = endian_be32(*state->ptr++);
	} while (((data - 0x01010101) & 0x80808080) == 0);
}

/**
 * Walks a node, start token should already be eaten. Will eat the
 * stop token.
 */
error_t fdt_walk_node(struct fdt_walker_state_s *state, struct fdt_walker_s *walker)
{
	bool_t wanted = walker->on_node_entry(
		walker->private, state, (const char*)state->ptr);

	fdt_skip_str(state);

	if ( !wanted )
		walker = &null_walker;

	for (;;) {
		uint32_t token = endian_be32(*state->ptr++);
		switch ( token ) {
		case FDT_NODE_START: {
			error_t err = fdt_walk_node(state, walker);
			if (err)
				return err;
			break;
		}
		case FDT_ATTR: {
			struct fdt_attr_s *attr = (struct fdt_attr_s*)state->ptr;
			state->ptr += (8 + endian_be32(attr->size) + 3) / 4;
			walker->on_node_attr(
				walker->private,
				state,
				&state->string_table[endian_be32(attr->strid)],
				attr->data,
				endian_be32(attr->size));
			break;
		}
		case FDT_NODE_END:
			walker->on_node_leave(walker->private);
			return 0;
		default:
			printk("Unhandled FDT Token: %x @ %p\n", token, (void*)state->ptr-state->blob);
			return EINVAL;
		}
	}
}

error_t fdt_walk_blob(const void *blob, struct fdt_walker_s *walker)
{
	struct fdt_walker_state_s state = {
		.blob = blob,
		.ptr = blob,
	};

	const struct fdt_header_s *header = blob;

	if ( (uintptr_t)blob & 3 ) {
		printk("Unaligned FDT: %p\n", blob);
		return EINVAL;
	}

	if ( endian_be32(header->magic) != FDT_MAGIC ) {
		printk("FDT bad magic, expected %x, got %x\n",
			   FDT_MAGIC, endian_be32(header->magic));
		return EINVAL;
	}

/* 	printk("FDT magic OK, string table @ %p\n", endian_be32(header->off_dt_strings)); */
/* 	printk("FDT magic OK, struct table @ %p\n", endian_be32(header->off_dt_struct)); */

	state.string_table = (const char*)blob + endian_be32(header->off_dt_strings);
	state.ptr = (void*)((uintptr_t)blob + endian_be32(header->off_dt_struct));

	struct fdt_mem_reserve_map_s *reserve_map =
		(void*)((uintptr_t)blob + endian_be32(header->off_mem_rsvmap));
	while ( reserve_map->addr || reserve_map->size ) {
		walker->on_mem_reserve(walker->private,
							   endian_be64(reserve_map->addr),
							   endian_be64(reserve_map->size));
		reserve_map++;
	}

	if ( endian_be32(*state.ptr) != FDT_NODE_START ) {
		printk("FDT bad token, expected %x, got %x\n",
			   FDT_NODE_START, endian_be32(*state.ptr));
		return EINVAL;
	}

	while ( endian_be32(*state.ptr) == FDT_NODE_START ) {
		state.ptr++;
		fdt_walk_node(&state, walker);
	}

	return 0;
}

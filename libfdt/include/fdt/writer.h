#ifndef FDT_WRITER_H_
#define FDT_WRITER_H_

#include <hexo/types.h>
#include <hexo/error.h>

/**
 * An opaque structure containing the internal state of the
 * writer. This is user-provided.
 */
struct fdt_writer_s
#ifndef FDT_INTERNAL
{
	void *opaque[7];
}
#endif
;

/**
 * Initialize the writer internal state, give a memory region to the
 * writer code. If this memory region is not big enough, caller wont
 * be notified until fdt_writer_finalize() is called.
 */
error_t fdt_writer_init(
	struct fdt_writer_s *writer,
	void *blob,
	size_t available_size);

/**
 * Add a reservation entry in the memory reservation map.
 *
 * All calls to this function should be done before beginning to build
 * the structure (nodes and propibutes)
 */
void fdt_writer_add_mem_reservation(
	struct fdt_writer_s *writer,
	uint64_t addr,
	uint64_t size);

/**
 * Pushes a node in the device tree. The root node should have an
 * empty name ("").
 */
uint32_t fdt_writer_node_entry(
	struct fdt_writer_s *writer,
	const char *name);

/**
 * Pushes an propibute in the current node. The data will be copied in
 * the blob.
 */
void fdt_writer_node_prop(
	struct fdt_writer_s *writer,
	const char *name,
	void *data,
	size_t len);

/**
 * Closes the current node.
 */
void fdt_writer_node_leave(struct fdt_writer_s *writer);

/**
 * Finalizes the blob. This function packs the data and writes all
 * the offsets.
 */
error_t fdt_writer_finalize(struct fdt_writer_s *writer, size_t *real_size);

#endif

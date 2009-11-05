#ifndef FDT_READ_H_
#define FDT_READ_H_

/**
   @file
   @module{FDT access library}
   @short Read-only access to FDT blobs
 */

#include <hexo/types.h>
#include <hexo/error.h>

struct fdt_walker_s;
struct fdt_walker_state_s;

/**
   on_node_entry prototype macro
 */
#define FDT_ON_NODE_ENTRY_FUNC(x) bool_t (x)(						   \
		void *private,												   \
		struct fdt_walker_state_s *state,							   \
		const char *path)

/**
   on_node_leave prototype macro
 */
#define FDT_ON_NODE_LEAVE_FUNC(x) void (x)(void *private)

/**
   on_node_prop prototype macro
 */
#define FDT_ON_NODE_PROP_FUNC(x) void (x)(							   \
		void *private,												   \
		struct fdt_walker_state_s *state,							   \
		const char *name,											   \
		const void *data,											   \
		size_t datalen)

/**
   on_mem_reserve prototype macro
 */
#define FDT_ON_MEM_RESERVE_FUNC(x) void (x)(						   \
		void *private,												   \
		uint64_t addr,												   \
		uint64_t size)

/**
   Type definition for entry in a new node. As nodes may be nested,
   this function may be called many times in a row.

   @param private private data provided in the fdt_walker_s
   @param offset node offset from the beginning of the structure, this
          can be used to resolve references
   @param path full path of the node

   @return whether user is interested in this node, its properties and
           its subnodes.
 */
typedef FDT_ON_NODE_ENTRY_FUNC(fdt_on_node_entry_func_t);

/**
   Type definition for end of a node. As nodes may be nested, this
   function may be called many times in a row.

   @param private private data provided in the fdt_walker_s
 */
typedef FDT_ON_NODE_LEAVE_FUNC(fdt_on_node_leave_func_t);

/**
   Type definition for function called on a node property. As
   properties are inside nodes, on_node_entry has already be called
   once when calling this function.

   @param private private data provided in the fdt_walker_s
   @param offset offset of the parameter in the structure
   @param name name of the parameter
   @param data pointer to raw data. User must take care of the meaning
          by itself. If data contains numeric values, they are
          stored bigendian.
   @param datalen length of the data, in bytes
 */
typedef FDT_ON_NODE_PROP_FUNC(fdt_on_node_prop_func_t);

/**
   Type definition for function called on a memory reservation
   node. Note values are always 64 bits for this type of nodes.

   There is no endian adaptation to perform on the parameters.

   @param private private data provided in the fdt_walker_s
   @param addr base address of reservation
   @param size size of reservation
 */
typedef FDT_ON_MEM_RESERVE_FUNC(fdt_on_mem_reserve_func_t);

/**
   Structure containing pointers to user-provided functions and
   private data. When using @ref fdt_walk_blob with this structure,
   all function pointers must be provided. Leaving a NULL pointer to
   function will lead to unpredictable results.
 */
struct fdt_walker_s
{
	void *private; /** User-owned pointer, ignored by walker */
	fdt_on_node_entry_func_t *on_node_entry; /** Function to call entering a node */
	fdt_on_node_leave_func_t *on_node_leave; /** Function to call leaving a node */
	fdt_on_node_prop_func_t *on_node_prop; /** Function to call for each property */
	fdt_on_mem_reserve_func_t *on_mem_reserve; /** Function to call for each memory reservation map */
};

/**
   @this processes a whole blob calling the provided functions when
   needed. The blob contains its size in its own header.

   @param blob Pointer to the FDT blob header
   @param walker User-provided functions
   @return whether the parsing went well, to the end
 */
error_t fdt_walk_blob(const void *blob, struct fdt_walker_s *walker);

/**
   @this retrieves the structure offset for the current state.

   This is not valid when walking through memory reservation.

   @param state Internal state of the parser
   @return the current token offset
 */
uint32_t fdt_reader_get_struct_offset(struct fdt_walker_state_s *state);

/**
   @this retrieves the value of a property inside a node, if it
   exists.  Calling this function is only valid when inside a
   on_node_entry.

   @param state Internal state of the parser
   @param prop Property name to look for
   @param propval return pointer to the property value, if found
   @param propsize return size of property value, if found
   @return 1 if the property has been found, 0 otherwise
 */
bool_t fdt_reader_has_prop(const struct fdt_walker_state_s *state,
						   const char *propname,
						   const void **propval, size_t *propsize);

/**
   @this gets the complete size of a blob as told in its header.

   @param blob a pointer to the start of a device tree blob
   @return the total size of the blob, 0 if the blob is invalid
 */
size_t fdt_get_size(void *blob);

/**
   @this processes a blob calling the provided functions when needed,
   only from the given offset in the blob. The offset must be a value
   previously returned by @ref fdt_reader_get_struct_offset, and has
   no meaningful value outside this context.  This function will never
   walk the memory reservation nodes. The corresponding pointer in
   walker may be NULL.

   @param blob Pointer to the FDT blob header
   @param walker User-provided functions
   @param offset The subnode offset to walk
   @return whether the parsing went well, to the end
 */
error_t fdt_walk_blob_from(const void *blob,
						   struct fdt_walker_s *walker,
						   uint32_t offset);

/**
   @this retrieves a particular memory reservation entry from the
   table.  This function does not check for bounds.  User can tell it
   reached the last entry when *addr and *size are 0.

   @param blob Pointer to the FDT blob header
   @param resno Index of the memory reservation in the map, 0-indexed
   @param addr Pointer @this must fill with the address of the
   reservation
   @param size Pointer @this must fill with the size of the
   reservation
 */
void fdt_get_rsvmap(const void *blob, uint32_t resno,
					uint64_t *addr, uint64_t *size);

#endif

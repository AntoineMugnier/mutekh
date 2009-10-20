
#define FDT_MAGIC 0xd00dfeed
#define FDT_NODE_START 0x1
#define FDT_NODE_END 0x2
#define FDT_ATTR 0x3
#define FDT_END 0x9

struct fdt_header_s
{
	uint32_t magic;
	uint32_t totalsize;
	uint32_t off_dt_struct;
	uint32_t off_dt_strings;
	uint32_t off_mem_rsvmap;
	uint32_t version;
	uint32_t last_comp_version;
	uint32_t boot_cpuid_phys;
	uint32_t size_dt_strings;
	uint32_t size_dt_struct;
};

struct fdt_mem_reserve_map_s
{
	uint64_t addr;
	uint64_t size;
};

struct fdt_attr_s
{
	const uint32_t size;
	const uint32_t strid;
	const char data[0];
};

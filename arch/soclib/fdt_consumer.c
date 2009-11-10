#include <hexo/init.h>
#include <hexo/segment.h>
#include <hexo/types.h>
#include <hexo/interrupt.h>
#if defined(CONFIG_HEXO_IPI)
#include <hexo/ipi.h>
#endif

#include <drivers/device/enum/fdt/enum-fdt.h>
#include <device/icu.h>
#include <device/device.h>
#include <device/driver.h>

#include <mutek/printk.h>

#include <fdt/reader.h>

#if 0
#define dprintk(...) printk(__VA_ARGS__)
#else
#define dprintk(...) do{}while(0)
#endif

#ifdef CONFIG_MUTEK_CONSOLE
extern struct device_s *console_dev;
#endif

#ifdef CONFIG_VFS
extern struct device_s *root_dev;
#endif

#if defined (CONFIG_MUTEK_TIMERMS)
extern struct device_s *timerms_dev;
#endif

enum state_e
{
	IN_ROOT,
	IN_CHOSEN,
	IN_CPUS,
	IN_CPU,
	IN_TOPOLOGY,
};

struct consumer_state_s
{
	struct device_s *enum_dev;
	enum state_e state;
#if defined(CONFIG_HEXO_IPI)
	struct device_s *ipi_dev;
	uint32_t ipi_no;
#endif
	uint32_t cpuid;
};

static FDT_ON_NODE_ENTRY_FUNC(creator_node_entry)
{
	struct consumer_state_s *priv = private;

	dprintk(">> %s, %d\n", path, priv->state);

	switch (priv->state) {
	case IN_ROOT:
		if ( !strcmp(path, "/chosen") ) {
			priv->state = IN_CHOSEN;
			return 1;
		}
		if ( !strcmp(path, "/cpus") ) {
			priv->state = IN_CPUS;
			return 1;
		}
		if ( !strcmp(path, "/topology") ) {
			priv->state = IN_TOPOLOGY;
			return 1;
		}
		break;
	case IN_CPUS: {
		priv->state = IN_CPU;
#if defined(CONFIG_HEXO_IPI)
		priv->ipi_dev = NULL;
		priv->ipi_no = 0;
#endif

#if defined(CONFIG_SMP)
		bool_t got;
		got = fdt_reader_get_prop_int(
			state, "reg",
			&priv->cpuid, sizeof(priv->cpuid));
		assert(got);

		void *cls = arch_cpudata_alloc();
		dprintk("  created CLS @ %p\n", cls);
		cpu_local_storage[priv->cpuid] = cls;
#endif
		break;
	}
	default:
		break;
	}
			
	return 1;
}

static FDT_ON_NODE_LEAVE_FUNC(creator_node_leave)
{
	struct consumer_state_s *priv = private;

	dprintk("<< %d\n", priv->state);

	switch (priv->state) {
	case IN_ROOT:
		break;
	case IN_CPUS:
	case IN_CHOSEN:
	case IN_TOPOLOGY:
		priv->state = IN_ROOT;
		break;
	case IN_CPU:
		priv->state = IN_CPUS;
#if defined(CONFIG_HEXO_IPI)
		if ( priv->ipi_dev && priv->ipi_dev->drv ) {
			dprintk("Preparing ipi dev\n");
			void *foo = dev_icu_setupipi(priv->ipi_dev, priv->ipi_no);
			dprintk("  CPU %d using %p:%d as ipi device, cls=%p, priv=%p\n",
				   priv->cpuid, priv->ipi_dev, priv->ipi_no,
				   cpu_local_storage[priv->cpuid], foo);
			ipi_hook_cpu(cpu_local_storage[priv->cpuid], priv->ipi_dev, foo);
		} else {
			dprintk("  No IPI dev for CPU %d\n", priv->cpuid);
		}
#endif
		break;
	}
}

static FDT_ON_NODE_PROP_FUNC(creator_node_prop)
{
	struct consumer_state_s *priv = private;

	switch (priv->state) {
	case IN_CHOSEN:
#ifdef CONFIG_MUTEK_CONSOLE
		if ( !strcmp(name, "console") ) {
			struct device_s *cd = enum_fdt_lookup(priv->enum_dev, data);
			if ( cd && cd->drv ) {
				dprintk("Setting console device to node %s\n", data);
				console_dev = cd;
			}
		}
#endif
#ifdef CONFIG_MUTEK_TIMERMS
		if ( !strcmp(name, "timer") ) {
			struct device_s *cd = enum_fdt_lookup(priv->enum_dev, data);
			if ( cd && cd->drv ) {
				dprintk("Setting timer device to node %s\n", data);
				timerms_dev = cd;
			}
		}
#endif
#ifdef CONFIG_MUTEK_CONSOLE
		if ( !strcmp(name, "root") ) {
			struct device_s *cd = enum_fdt_lookup(priv->enum_dev, data);
			if ( cd && cd->drv ) {
				dprintk("Setting root device to node %s\n", data);
				root_dev = cd;
			}
		}
#endif
		break;
	case IN_CPU:
#if defined(CONFIG_HEXO_IPI)
		if ( !strcmp(name, "ipi_dev") ) {
			priv->ipi_dev = enum_fdt_lookup(priv->enum_dev, data);
			dprintk("Getting ipi dev for cpu %d \"%s\": %p\n", priv->cpuid, data, priv->ipi_dev);
		} else if ( !strcmp(name, "ipi_no") ) {
			fdt_parse_sized(1, data, sizeof(priv->ipi_no), &priv->ipi_no);
			dprintk("Getting ipi no for cpu %d: %d\n", priv->cpuid, priv->ipi_no);
		}
#endif
		break;
	default:
		break;
	}
}

static FDT_ON_MEM_RESERVE_FUNC(creator_mem_reserve)
{
}


void soclib_parse_fdt(void *blob, struct device_s *enum_dev)
{
	struct consumer_state_s priv = {
		.enum_dev = enum_dev,
		.state = IN_ROOT,
	};

	struct fdt_walker_s walker = {
		.private = &priv,
		.on_node_entry = creator_node_entry,
		.on_node_leave = creator_node_leave,
		.on_node_prop = creator_node_prop,
		.on_mem_reserve = creator_mem_reserve,
	};

	dprintk("%s walking blob\n", __FUNCTION__);
	fdt_walk_blob(blob, &walker);
}

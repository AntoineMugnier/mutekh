
header_sources = $(wildcard $(LOCAL_SRC_DIR)/include/gfx/*.t)
header_raw = $(wildcard $(LOCAL_SRC_DIR)/include/gfx/*.h)
pre_headers = $(subst $(LOCAL_SRC_DIR),,$(header_sources:.t=.h))
copy = $(subst $(LOCAL_SRC_DIR),,$(header_raw))

objs = gfx.o
objs-$(CONFIG_GFX_BYTECODE) += bytecode.o

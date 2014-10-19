header_sources = $(wildcard $(LOCAL_SRC_DIR)/gct/include/gct/*.t)
header_raw = $(wildcard $(LOCAL_SRC_DIR)/gct/include/gct/*.h)

pre_headers = $(subst $(LOCAL_SRC_DIR)/gct/,,$(header_sources:.t=.h))
copy = $(subst $(LOCAL_SRC_DIR)/gct/,,$(header_raw))

ifeq ($(CONFIG_COMPILE_OPTIMIZE), 0)
objs = libgct-mutek.o
else
objs =
endif

objs += libgct.o


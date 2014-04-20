header_sources = $(wildcard $(LOCAL_SRC_DIR)/gct/include/gct/*.t)
header_raw = $(wildcard $(LOCAL_SRC_DIR)/gct/include/gct/*.h)

pre_headers = $(subst $(LOCAL_SRC_DIR)/gct/,,$(header_sources:.t=.h))
copy = $(subst $(LOCAL_SRC_DIR)/gct/,,$(header_raw))

objs = libgct.o


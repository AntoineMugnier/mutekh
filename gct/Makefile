header_sources = $(wildcard $(LOCAL_SRC_DIR)/gct/include/gct/*.t)
header_raw = $(wildcard $(LOCAL_SRC_DIR)/gct/include/gct/*.h)

pre_headers = $(subst $(LOCAL_SRC_DIR)/gct/,,$(header_sources:.t=.h))
copy = $(subst $(LOCAL_SRC_DIR)/gct/,,$(header_raw))

objs = libgct.o

DIR_CFLAGS = -Wno-maybe-uninitialized

ext_hg = gct 6b4b0bc1e8c6cd3459c2cefe898945641dfbf60c http://hg.savannah.nongnu.org/hgweb/gct

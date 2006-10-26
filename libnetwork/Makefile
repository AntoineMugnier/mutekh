
objs += packet.o
objs += if.o

ifeq ($(CONFIG_NETWORK_ROUTING), defined)
 objs += route.o
endif

ifeq ($(CONFIG_NETWORK_FILTER), defined)
 objs += filter.o
endif

subdirs = protocols

ifeq ($(CONFIG_NETWORK_UDP), defined)
 subdirs += libudp
endif

ifeq ($(CONFIG_NETWORK_TCP), defined)
 subdirs += libtcp
endif

ifeq ($(CONFIG_NETWORK_SOCKET), defined)
 subdirs += libsocket
endif

ifeq ($(CONFIG_NETWORK_NFS), defined)
 subdirs += lwnfs
endif

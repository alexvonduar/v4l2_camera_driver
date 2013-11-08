obj-m += ov5640.o
EXTRA_CFLAGS += -Werror
ifdef DEBUG
		EXTRA_CFLAGS += -Werror -DCONFIG_DBG=1
endif
#ccflags-m += -Werror
#cflags-m += -Werror
#ccflags-y += -Werror
all:
	ifeq($(DEBUG),1)
		EXTRA_CFLAGS += -Werror -DCONFIG_DBG=1
	endif

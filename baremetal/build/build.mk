MYDIR := $(dir $(lastword $(MAKEFILE_LIST)))

ifeq ($(strip ${EBBRT_SRCDIR}),)
$(error EBBRT_SRCDIR not set)
endif

EBBRT_TARGET := matrix
EBBRT_APP_CAPNPS := Messages.capnp
EBBRT_APP_OBJECTS := Matrix.o
EBBRT_APP_VPATH := \
	$(abspath $(MYDIR)..) \
	$(abspath $(MYDIR)../..)
EBBRT_CONFIG = $(abspath $(MYDIR)../ebbrtcfg.h)

include $(abspath $(EBBRT_SRCDIR)/apps/ebbrtbaremetal.mk)

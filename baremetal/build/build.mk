MYDIR := $(dir $(lastword $(MAKEFILE_LIST)))

ifeq ($(strip ${EBBRT_SRCDIR}),)
$(error EBBRT_SRCDIR not set)
endif

ifeq ($(SAGE_STANDALONE),1)
EBBRT_TARGET := standalone_matrix
EBBRT_CONFIG = $(abspath $(MYDIR)../standaloneebbrtcfg.h)
EBBRT_APP_OBJECTS := Matrix.o App.o
else
EBBRT_TARGET := matrix
EBBRT_CONFIG = $(abspath $(MYDIR)../ebbrtcfg.h)
EBBRT_APP_OBJECTS := Matrix.o
endif

EBBRT_APP_CAPNPS := Messages.capnp

EBBRT_APP_VPATH := \
	$(abspath $(MYDIR)..) \
	$(abspath $(MYDIR)../..)

EBBRT_APP_INCLUDES := -iquote $(abspath $(MYDIR)..) -iquote $(abspath $(MYDIR)../..)

include $(abspath $(EBBRT_SRCDIR)/apps/ebbrtbaremetal.mk)

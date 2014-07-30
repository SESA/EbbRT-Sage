EBBRT_BUILDTYPE=Release
EBBRT_OPTFLAGS=-DBOOST_UBLAS_NDEBUG -O4 -DNDEBUG -flto

$(warning standalone = $(SAGE_STANDALONE))

MYDIR := $(dir $(lastword $(MAKEFILE_LIST)))
include $(abspath $(MYDIR)../build.mk)





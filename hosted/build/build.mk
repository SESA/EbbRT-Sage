MYDIR := $(dir $(lastword $(MAKEFILE_LIST)))

ifeq ($(strip ${EBBRT_SRCDIR}),)
  $(error EBBRT_SRCDIR not set)
endif

ifeq ($(strip ${SAGE_ROOT}),)
  $(error SAGE_ROOT not set)
endif

ebbrt_hosted = ${EBBRT_SRCDIR}/hosted	
ebbrt_hostedinc = ${EBBRT_SRCDIR}/hosted/src/include
ebbrt_commoninc = ${EBBRT_SRCDIR}/common/src/include 

ifeq ($(strip ${EBBRT_BUILDTYPE}),Debug)
  OPTFLAGS ?= -O0 -g
else ifeq ($(strip ${EBBRT_BUILDTYPE}),Release)
  OPTFLAGS ?= -O4 -flto
else 
  $(error EBBRT_BUILDTYPE must be set to either Debug or Release)
endif

%.cc: %.pyx
	cython --cplus -v $(CYTHON_CPPFLAGS) $< -o $@

INCLUDES := \
	-iquote $(MYDIR).. \
	-iquote $(CURDIR) \
	-I $(ebbrt_hostedinc) \
	-I $(ebbrt_commoninc) \
	-I $(SAGE_ROOT)/local/include/python2.7

CXXFLAGS := -fpic -std=c++11 -Wall -Werror $(INCLUDES) $(OPTFLAGS)

ebbrt_libdir := $(MYDIR)lib

ebbrt_lib := ${ebbrt_libdir}/libEbbRT.so

CAPNP_SRCS := Messages.capnp
CAPNP_OBJECTS := $(CAPNP_SRCS:.capnp=.capnp.o)
CAPNP_H := $(CAPNP_SRCS:.capnp=.capnp.h)


OBJ := \
	ebb_matrix.o \
	ebb_matrix_helper.o \
	LocalMatrix.o \
	Matrix.o \
	Messages.capnp.o

ebb_matrix.so: $(OBJ) $(ebbrt_lib)
	$(CXX) $(OPTFLAGS) -L $(ebbrt_libdir) -shared -Wl,-soname,ebb_matrix.so \
	-o ebb_matrix.so $(OBJ) -lc -lEbbRT -lboost_coroutine -lboost_context \
	-lboost_filesystem -lboost_system -lcapnp -lkj -lfdt -ltbb -pthread

test: $(OBJ) test.o $(ebbrt_lib)
	$(CXX) $(OPTFLAGS) -L $(ebbrt_libdir) \
	-o test test.o $(OBJ) -lc -lEbbRT -lboost_coroutine -lboost_context \
	-lboost_filesystem -lboost_system -lcapnp -lkj -lfdt -ltbb -pthread

%.o: %.cc $(CAPNP_H)
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.c++ $(CAPNP_H)
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.capnp.h %.capnp.c++: %.capnp
	$(ebbrt_makedir)
	capnp compile -oc++:$(CURDIR) --src-prefix=$(abspath $(MYDIR)../..) $<

${ebbrt_libdir}:
	mkdir ${ebbrt_libdir}

${ebbrt_libdir}/Makefile: ${ebbrt_libdir}
	(cd $(ebbrt_libdir); cmake -DCMAKE_BUILD_TYPE=${EBBRT_BUILDTYPE} \
	-DBUILD_SHARED_LIBS=ON ${ebbrt_hosted})

${ebbrt_lib}: ${ebbrt_libdir}/Makefile
	$(MAKE) -C ${ebbrt_libdir}

.PHONY: distclean clean

.SUFFIXES:

distclean: clean

clean:
	-$(RM) $(wildcard $(OBJ) ebb_matrix.so)
	-$(RM) -rf $(wildcard $(ebbrt_libdir))

VPATH := $(abspath $(MYDIR)..) $(abspath $(MYDIR)../..)

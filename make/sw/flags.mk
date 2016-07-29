# Architecture flags
# ------------------
arch_tgts :=
arch_opts := opt dbg

src_arch := $(shell uname -r)
ifneq ($(findstring el5,$(src_arch)),)
arch_tgts_q := i386-linux x86_64-linux
endif
ifneq ($(findstring el6,$(src_arch)),)
arch_tgts_q := x86_64-rhel6
endif
ifneq ($(findstring el7,$(src_arch)),)
arch_tgts_q := x86_64-rhel7
endif

define arch_opt_template
arch_tgts += $$(addsuffix -$(1),$$(arch_tgts_q))
endef
$(foreach opt,$(arch_opts),$(eval $(call arch_opt_template,$(opt))))

# i386 Linux specific flags
ifneq ($(findstring i386-linux,$(tgt_arch)),)
AS  := as
CPP := gcc -E
CC  := gcc
CXX := g++
LD  := g++
LX  := g++

LIBEXTNS := so
DEPFLAGS := -MM
DEFINES  := -fPIC -D_REENTRANT -D__pentium__ -Wall
CPPFLAGS :=
CFLAGS   := -m32
CXXFLAGS := $(CFLAGS)
CASFLAGS := -x assembler-with-cpp -P $(CFLAGS)
LDFLAGS  := -m32 -shared
LXFLAGS  := -m32

USRLIBDIR := /usr/lib
else
ifneq ($(findstring x86_64,$(tgt_arch)),)
AS  := as
CPP := gcc -E
CC  := gcc
CXX := g++
LD  := g++
LX  := g++

LIBEXTNS := so
DEPFLAGS := -MM
DEFINES  := -fPIC -D_REENTRANT -D__pentium__ -Wall
CPPFLAGS :=
CFLAGS   := -m64
CXXFLAGS := $(CFLAGS)
CASFLAGS := -x assembler-with-cpp -P $(CFLAGS)
LDFLAGS  := -m64 -shared
LXFLAGS  := -m64
USRLIBDIR := /usr/lib64
else
# Sparc Solaris specific flags
ifneq ($(findstring sparc-solaris,$(tgt_arch)),)
AS  := as
CPP := gcc -E
CC  := gcc
CXX := g++
LD  := g++
LX  := g++

LIBEXTNS := so
DEPFLAGS := -MM
DEFINES  := -fPIC -D_REENTRANT -Wall
CPPFLAGS :=
CFLAGS   :=
CXXFLAGS := $(CFLAGS)
CASFLAGS := -x assembler-with-cpp -P $(CFLAGS)
LDFLAGS  := -shared
LXFLAGS  :=
else
# PowerPC RTEMS specific flags
ifneq ($(findstring ppc-rtems,$(tgt_arch)),)
AS  := powerpc-rtems-as
CPP := powerpc-rtems-cpp
CC  := powerpc-rtems-gcc
CXX := powerpc-rtems-g++
LD  := powerpc-rtems-ld
LX  := powerpc-rtems-g++

ifneq ($(findstring ppc-rtems-rce405,$(tgt_arch)),)
RTEMSDIR := $(RELEASE_DIR)/build/rtems/target/powerpc-rtems/rce405/lib
endif

ifneq ($(findstring ppc-rtems-ml405,$(tgt_arch)),)
RTEMSDIR := $(RELEASE_DIR)/build/rtems/target/powerpc-rtems/ml405/lib
endif

LIBEXTNS := a
DEPFLAGS := -B$(RTEMSDIR) -MM
DEFINES  := -Dppc405 -mcpu=403 -Wall
CPPFLAGS :=
CFLAGS   := -B$(RTEMSDIR) -specs bsp_specs -qrtems
CXXFLAGS := $(CFLAGS)
CASFLAGS := -x assembler-with-cpp -P $(CFLAGS)
LDFLAGS  := -r
LXFLAGS  := -B$(RTEMSDIR) -specs bsp_specs -qrtems
MANAGERS := timer sem msg event signal part region dpmem io rtmon ext mp
endif
endif
endif
endif


ifneq ($(findstring -opt,$(tgt_arch)),)
DEFINES += -O4
endif

ifneq ($(findstring -dbg,$(tgt_arch)),)
# RHEL7 g++ sometimes produces incorrect code when not optimized
#   C-style-cast sometimes does not equate to static_cast when it should
#   some instances of Queue<T>::insert(T* e) { insert((Entry*)e); }
DEFINES += -g -O2
endif


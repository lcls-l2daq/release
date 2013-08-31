# List of projects (low level first)

boost_use      := /reg/common/package/boost/1.49.0-python2.7/x86_64-rhel5-gcc41-opt/
ndarray_use    := /reg/common/package/ndarray/1.1.3/x86_64-rhel5-gcc41-opt

boost_use_include    := $(boost_use)/include
boost_use_lib_x86_64 := $(boose_use)/lib

ndarray_use_include := $(ndarray_use)

#
#  List external package base directories for convenience
#
#epics_use      := /reg/common/package/epicsca/3.14.12
epics_use       := /reg/g/pcds/package/external/epicsca-pcds-R1.0-r410
evgr_use       := /reg/g/pcds/package/external/evgr_V00-00-02
qt_use         := /reg/g/pcds/package/external/qt-4.3.4
qwt_use        := /reg/g/pcds/package/external/qwt-5.1.1-wfopt-logfix
#python_use     := /reg/g/pcds/package/python-2.5.2
python_use     := /reg/common/package/python/2.5.5
libraw1394_use := /reg/g/pcds/package/external/libdc1394
libdc1394_use  := /reg/g/pcds/package/external/libdc1394
offlinedb_use  := /reg/g/pcds/package/external/offlinedb-1.5.1
edt_use        := /reg/g/pcds/package/external/edt
leutron_use    := /reg/g/pcds/package/external/leutron_V00-00-00
fli_use        := /reg/g/pcds/package/external/fli-dist-1.71
andor_use      := /reg/g/pcds/package/external/andor-2.93.30007
libusb_use     := /reg/g/pcds/package/external/libusb-1.0.0
usdusb4_use    := /reg/g/pcds/package/external/usdusb4
acqiris_use    := /reg/g/pcds/package/external/acqiris_3.3a
relaxd_use     := /reg/g/pcds/package/external/relaxd-1.8.0
pvcam_use      := /reg/g/pcds/package/external/pvcam2.7.1.7
boost_use      := /reg/common/package/boost/1.49.0-python2.7/x86_64-rhel5-gcc41-opt/
ndarray_use    := /reg/common/package/ndarray/1.1.3/x86_64-rhel5-gcc41-opt
pdsalg_use     := /reg/common/package/pdsalg/0.0.0
pdsdata_use    := /reg/common/package/pdsdata/7.0.0

#
#  *_use_include definitions will create a directory structure under build for
#    external packages which don't already have the needed structure.  The
#    *_use_lib_i386 will create the lib/ structure with soft-links to the
#    variable reference for i386-linux-*.  The *_use_lib_x86_64 will create the
#    analogous soft-links for x86_64-linux-* libraries.  
#  Packages without a *_use_include definition will just have a soft-link under build.
#
boost_use_include    := $(boost_use)/include
boost_use_lib_x86_64 := $(boose_use)/lib

ndarray_use_include := $(ndarray_use)

qwt_use_include    := $(qwt_use)/include
qwt_use_lib_i386   := $(qwt_use)/lib/i386-linux
qwt_use_lib_x86_64 := $(qwt_use)/lib/x86_64-linux

pdsalg_use_include    := $(pdsalg_use)/x86_64-linux-opt
pdsalg_use_lib_i386   := $(pdsalg_use)/i386-linux-opt/lib
pdsalg_use_lib_x86_64 := $(pdsalg_use)/x86_64-linux-opt/lib

pdsdata_use_include    := $(pdsdata_use)/x86_64-linux-opt
pdsdata_use_lib_i386   := $(pdsdata_use)/i386-linux-opt/lib
pdsdata_use_lib_x86_64 := $(pdsdata_use)/x86_64-linux-opt/lib

#
# RTEMS
#
ifneq ($(findstring ppc-rtems-rce,$(tgt_arch)),)
projects := rtems \
            rce \
            rceusr \
            rceapp
#            rcehw

#rtems_use := /afs/slac.stanford.edu/g/npa/package/rtems/4.9.2
#rtems_use := /reg/g/pcds/package/rtems/4.9.2
rtems_use := ~/rtems/4.9.2

rce_use    := release
rceusr_use := release
rceapp_use := release
rcehw_use  := release
endif

#
# 32-bit linux
#
ifneq ($(findstring i386-linux,$(tgt_arch)),)
projects := pdsdata \
      boost \
      ndarray \
      acqiris \
      evgr \
      leutron \
      edt \
      qt \
      qwt \
      epics \
      offlinedb \
      pvcam \
      relaxd \
      fli \
      andor \
      libusb \
      usdusb4 \
      epics \
      pds \
      pdsapp \
      pdsalg \
      ami \
      timetool

pds_use      := release
pdsapp_use   := release
ami_use      := release
rce_use      := release
rceusr_use   := release
rceapp_use   := release
timetool_use := release
endif

#
# 64-bit linux
#
ifneq ($(findstring x86_64,$(tgt_arch)),)
projects := pdsdata \
      boost \
      ndarray \
      qwt \
      epics \
      evgr \
      edt \
      offlinedb \
      leutron \
      python \
      libraw1394 \
      libdc1394 \
      fli \
      andor \
      libusb \
      usdusb4 \
      qwt \
      epics \
      pds \
      pdsapp \
      pdsalg \
      ami \
      timetool

pds_use      := release
pds_use      := release
pdsapp_use   := release
ami_use      := release
timetool_use := release

# RHEL6 has qt in its distribution
ifeq ($(findstring x86_64-rhel6,$(tgt_arch)),)
projects += qt
endif
endif


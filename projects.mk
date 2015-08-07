# List of projects (low level first)

#
#  Determine which packages are in this release
#
rprojects := $(wildcard *)

#
#  List external package base directories for convenience
#
#epics_use      := /reg/common/package/epicsca/3.14.12
#epics_use       := /reg/g/pcds/package/external/epicsca-pcds-R1.0-r410
epics_use      := /reg/g/pcds/package/external/epicsca-R3.14.12-0.4.0
evgr_use       := /reg/g/pcds/package/external/evgr_V00-00-05
qt_use         := /reg/common/package/qt/4.6.2
qwt_use        := /reg/g/pcds/package/external/qwt-5.1.1-wfopt-logfix
#python_use     := /reg/g/pcds/package/python-2.5.2
python_use     := /reg/common/package/python/2.5.5
libraw1394_use := /reg/g/pcds/package/external/libdc1394
libdc1394_use  := /reg/g/pcds/package/external/libdc1394
offlinedb_use  := /reg/g/pcds/package/external/offlinedb-1.5.1
edt_use        := /reg/g/pcds/package/external/edt
leutron_use    := /reg/g/pcds/package/external/leutron_V00-00-00
fli_use        := /reg/g/pcds/package/external/fli-dist-1.71
andor_use      := /reg/g/pcds/package/external/andor-2.98.30010.0
libusb_use     := /reg/g/pcds/package/external/libusb-1.0.0
usdusb4_use    := /reg/g/pcds/package/external/usdusb4
acqiris_use    := /reg/g/pcds/package/external/acqiris_3.3a
relaxd_use     := /reg/g/pcds/package/external/relaxd-1.9.9
pvcam_use      := /reg/g/pcds/package/external/pvcam2.7.1.7
picam_use      := /reg/g/pcds/package/external/picam-2.6.1
gsl_use        := /reg/g/pcds/package/external/gsl-1.13
boost_use      := /reg/common/package/boost/1.49.0-python2.7/x86_64-rhel5-gcc41-opt/
ndarray_use    := /reg/common/package/ndarray/1.1.3/x86_64-rhel5-gcc41-opt
psalg_use      := /reg/common/package/psalg/1.0.9
pdsdata_use    := /reg/common/package/pdsdata/8.3.5
#pdsdata_use    := /reg/common/package/pdsdata/devel

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

psalg_use_include:= $(psalg_use)/x86_64-linux-opt
psalg_use_i386   := $(psalg_use)/i386-linux
psalg_use_x86_64 := $(psalg_use)/x86_64-linux

pdsdata_use_include:= $(pdsdata_use)/i386-linux-opt
pdsdata_use_i386   := $(pdsdata_use)/i386-linux
pdsdata_use_x86_64 := $(pdsdata_use)/x86_64-linux

projects :=

# RHEL6 has qt in its distribution
ifeq ($(findstring x86_64-rhel6,$(tgt_arch)),)
projects += qt
endif

# RHEL6 specific projects
ifneq ($(findstring x86_64-rhel6,$(tgt_arch)),)
 projects += picam
endif

projects += \
      pdsdata \
      boost \
      ndarray \
      qwt \
      psalg \
      python

ifneq ($(filter pds, $(rprojects)),)
  projects += \
      acqiris \
      evgr \
      leutron \
      edt \
      epics \
      offlinedb \
      libdc1394 \
      pvcam \
      relaxd \
      fli \
      andor \
      libusb \
      usdusb4 \
      pds \
      pdsapp

  pds_use        := release
  pdsapp_use     := release
  timetool_use   := release
else
  projects += \
      epics
  timetool_use   := /reg/g/pcds/dist/pds/7.6.7-p8.0.10/build/timetool
  timetool_use_include    := $(timetool_use)/include
  timetool_use_lib_x86_64 := $(timetool_use)/lib/x86_64-linux-opt
endif

projects += timetool

ifneq ($(filter ami, $(rprojects)),)
  projects += gsl ami
  ami_use := release
endif



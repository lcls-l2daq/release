# List of projects (low level first)
#projects := rtems \
            rce \
            rceusr \
            rceapp
#            rcehw

rtems_use := /afs/slac.stanford.edu/g/npa/package/rtems/4.9.2
#rtems_use := /reg/g/pcds/package/rtems/4.9.2
#rtems_use := ~/rtems/4.9.2

rce_use    := release
rceusr_use := release
rceapp_use := release
rcehw_use  := release

projects := pdsdata \
	    acqiris \
	    evgr \
	    leutron \
	    qt \
	    qwt \
	    epics \
	    pds \
	    pdsapp


epics_use   := /reg/g/pcds/package/external/epicsca-pcds-R1.0-r410
acqiris_use := /reg/g/pcds/package/external/acqiris_V00-00-00
evgr_use := /reg/g/pcds/package/external/evgr_V00-00-02
leutron_use := /reg/g/pcds/package/external/leutron_V00-00-00
qt_use := /reg/g/pcds/package/qt-4.3.4
qwt_use := /reg/g/pcds/package/external/qwt-5.1.1

pds_use := release
pdsdata_use := release
pdsapp_use := release

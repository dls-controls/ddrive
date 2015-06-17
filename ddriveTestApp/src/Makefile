TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = ddriveTest
# ddriveTest.dbd will be created and installed
DBD += ddriveTest.dbd

# ddriveTest.dbd will be made up from these files:
ddriveTest_DBD += base.dbd

# Include dbd files from all support applications:
ddriveTest_DBD += asyn.dbd ddrive.dbd motorSupport.dbd drvAsynIPPort.dbd drvAsynSerialPort.dbd
ddriveTest_DBD += asSupport.dbd calcSupport.dbd

# Add all the support libraries needed by this IOC
ddriveTest_LIBS += ddrive asyn motor autosave calc

# ddriveTest_registerRecordDeviceDriver.cpp derives from ddriveTest.dbd
ddriveTest_SRCS += ddriveTest_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
ddriveTest_SRCS_DEFAULT += ddriveTestMain.cpp
ddriveTest_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#ddriveTest_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
ddriveTest_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += ddrive

DBD += ddrive.dbd

# specify all source files to be compiled and added to the library
ddrive_SRCS += ddriveMain.cpp 
ddrive_SRCS += ddriveController.cpp 
ddrive_SRCS += ddriveAxis.cpp 
ddrive_SRCS += asynMotorController.cpp
ddrive_SRCS += asynMotorAxis.cpp

ddrive_LIBS += motor asyn
ddrive_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


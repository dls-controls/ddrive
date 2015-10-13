#!../../bin/linux-x86_64/ddriveTest

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/ddriveTest.dbd",0,0)
ddriveTest_registerRecordDeviceDriver(pdbbase) 

epicsEnvSet("SYS",    "$(SYS=XF:03IDC-ES)")
epicsEnvSet("CTLSYS", "$(CTLSYS=XF:03IDC-CT)")
epicsEnvSet("DEV",    "Ddrive:1")

# Note: if using procServ, setting this manually is unnecessary:
epicsEnvSet("IOCNAME", "ddrive")
epicsEnvSet("IOC_PREFIX", "$(CTLSYS){IOC:$(IOCNAME)}")

epicsEnvSet("EPICS_CA_AUTO_ADDR_LIST", "NO")
epicsEnvSet("EPICS_CA_ADDR_LIST", "10.3.0.255")

epicsEnvSet("DDRIVE_NET_IP", "10.3.2.59")
epicsEnvSet("DDRIVE_NET_PORT", "4012")
epicsEnvSet("ASYN_PORT", "DDRIVE")

drvAsynIPPortConfigure("E0","$(DDRIVE_NET_IP):$(DDRIVE_NET_PORT)",0,0,0)

# drvAsynSerialPortConfigure("$(PORT)", "/dev/ttyS0")
# asynSetOption("E0", -1,"baud",9600)
# asynSetOption("E0", -1,"bits",8)
# asynSetOption("E0", -1,"parity","none")
# asynSetOption("E0", -1,"stop",1)
# asynSetOption("E0", -1, "clocal", "N")
# asynSetOption("E0", -1, "crtscts", "N")

asynOctetSetInputEos("E0", -1, "\n")
asynOctetSetOutputEos("E0", -1, "\n")

# asynSetTraceIOMask("E0",-1,0)
# asynSetTraceMask("E0",-1,0)
# asynSetTraceIOMask("E0",-1,0x2)
# asynSetTraceMask("E0",-1,0x9)

#ddriveCreateController(portName, ddrivePortName, numAxes, movingPollPeriod, idlePollPeriod)
ddriveCreateController("DDRIVE0", "E0", 3, 50, 100)

# asynSetTraceMask("DDRIVE0", 0, 3)
#asynSetTraceMask("E0", -1, 3)
asynSetTraceIOMask("DDRIVE0", -1, 255)
#asynSetTraceIOMask("E0", -1, 255)

dbLoadRecords("$(TOP)/db/motors.db", "SYS=$(SYS),CTLSYS=$(CTLSYS),DEV=$(DEV),PORT=DDRIVE0")

set_savefile_path("${TOP}/as/save","")
set_requestfile_path("$(EPICS_BASE)/as/req")
set_requestfile_path("${TOP}/as/req")

system("install -m 777 -d ${TOP}/as/save")
system("install -m 777 -d ${TOP}/as/req")

cd $(TOP)/as/req
makeAutosaveFiles()

set_pass0_restoreFile("info_positions.sav")
set_pass0_restoreFile("info_settings.sav")

dbLoadRecords("$(EPICS_BASE)/db/save_restoreStatus.db","P=$(IOC_PREFIX)")
dbLoadRecords("$(EPICS_BASE)/db/iocAdminSoft.db","IOC=$(IOC_PREFIX)")
save_restoreSet_status_prefix("$(IOC_PREFIX)")
# asSetFilename("/cf-update/acf/default.acf")

iocInit()

create_monitor_set("info_positions.req", 5, "")
create_monitor_set("info_settings.req", 30, "")

cd ${TOP}
dbl > ./records.dbl
# system "cp ./records.dbl /cf-update/$HOSTNAME.$IOCNAME.dbl"

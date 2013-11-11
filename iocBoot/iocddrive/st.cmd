#!../../bin/linux-x86/ddriveTest

< envPaths

## Register all support components
dbLoadDatabase("../../dbd/ddriveTest.dbd",0,0)
ddriveTest_registerRecordDeviceDriver(pdbbase) 
epicsEnvSet("P", "$(P=MLL:)")
epicsEnvSet("R", "$(R=DDRIVE:)")
epicsEnvSet("DDRIVE_NET_IP", "$(DDRIVE_NET_IP=10.0.0.11)")
epicsEnvSet("DDRIVE_NET_PORT", "$(DDRIVE_NET_PORT=4016)")
epicsEnvSet("AS_PREFIX", "$(P)")
epicsEnvSet("ASYN_PORT", "$(ASYN_PORT=DDRIVE)")

drvAsynIPPortConfigure("$(ASYN_PORT)","$(DDRIVE_NET_IP):$(DDRIVE_NET_PORT)",0,0,0)
#
#drvAsynSerialPortConfigure("$(PORT)", "/dev/ttyS0")
#asynSetOption("$(ASYN_PORT)", -1,"baud",9600)
#asynSetOption("$(ASYN_PORT)", -1,"bits",8)
#asynSetOption("$(ASYN_PORT)", -1,"parity","none")
#asynSetOption("$(ASYN_PORT)", -1,"stop",1)
#asynSetOption("$(ASYN_PORT)", -1, "clocal", "N")
#asynSetOption("$(ASYN_PORT)", -1, "crtscts", "N")
asynOctetSetInputEos("$(ASYN_PORT)", -1, "\n")
asynOctetSetOutputEos("$(ASYN_PORT)", -1, "\n")
asynSetTraceIOMask("$(ASYN_PORT)",-1,0)
asynSetTraceMask("$(ASYN_PORT)",-1,0)
#asynSetTraceIOMask("$(ASYN_PORT)",-1,0x2)
#asynSetTraceMask("$(ASYN_PORT)",-1,0x9)

#ddriveCreateController(portName, ddrivePortName, numAxes, movingPollPeriod, idlePollPeriod)
ddriveCreateController("DDRIVE0", "$(ASYN_PORT)", 3, 50, 100)

#asynSetTraceMask("DDRIVE0", 0, 3)
#asynSetTraceMask("$(ASYN_PORT)", -1, 3)
#asynSetTraceIOMask("DDRIVE0", -1, 255)
#asynSetTraceIOMask("$(ASYN_PORT)", -1, 255)

dbLoadTemplate("ddrive.sub")
dbLoadRecords("$(TOP)/db/ddrive.db", "P=$(P),R=$(R)AX0:,PORT=DDRIVE0,ADDR=0,TIMEOUT=0.5")
#dbLoadRecords("$(TOP)/db/ddrive.db", "P=$(P),R=$(R)AX1:,PORT=DDRIVE0,ADDR=1,TIMEOUT=0.5")
#dbLoadRecords("$(TOP)/db/ddrive.db", "P=$(P),R=$(R)AX2:,PORT=DDRIVE0,ADDR=2,TIMEOUT=0.5")

dbLoadRecords("$(TOP)/db/ddrive_mbb.db", "P=$(P),R=$(R)AX0:,PORT=DDRIVE0,ADDR=0,TIMEOUT=0.5")
#dbLoadRecords("$(TOP)/db/ddrive_mbb.db", "P=$(P),R=$(R)AX1:,PORT=DDRIVE0,ADDR=1,TIMEOUT=0.5")
#dbLoadRecords("$(TOP)/db/ddrive_mbb.db", "P=$(P),R=$(R)AX2:,PORT=DDRIVE0,ADDR=2,TIMEOUT=0.5")
#< save_restore.cmd

iocInit()

#create_monitor_set("auto_positions.req", 5, "P=$(AS_PREFIX)")
#create_monitor_set("auto_settings.req", 30, "P=$(AS_PREFIX)")


ddrive EPICS Driver
=================

Motor record driver ("model-3" type) for the PiezoJena d-drive piezo controller.

Requirements
------------

Though it may work on other versions, the driver was tested on these:

1. EPICS base 3.14.12.3 http://www.aps.anl.gov/epics/
2. asyn 4-18 http://www.aps.anl.gov/epics/modules/soft/asyn/
3. motor record 6-7 http://www.aps.anl.gov/bcda/synApps/motor/

Optional
--------

1. EDM http://ics-web.sns.ornl.gov/edm/log/getLatest.php
   Screens are provided in $TOP/opi for EDM.
2. Autosave http://www.aps.anl.gov/bcda/synApps/autosave/autosave.html

Installation
------------

1. Install EPICS
    1. If using a Debian-based system (e.g., Ubuntu), use the packages here http://epics.nsls2.bnl.gov/debian/
    2. If no packages are available for your distribution, build from source
2. Edit configure/RELEASE
    1. Point the directories listed in there to the appropriate places
    2. If using the Debian packages, everything can be pointed to /usr/lib/epics
3. Edit iocBoot/iocddrive/st.cmd
    1. Change the shebang on the top of the script if your architecture is different than linux-x86:
        #!../../bin/linux-x86/ddrive
        (check if the environment variable EPICS_HOST_ARCH is set, or perhaps `uname -a`, or ask someone if
         you don't know)
    2. The following line sets the prefix to all of your ddrive PVs (with $(P)$(R)):
        ```
        epicsEnvSet("P", "$(P=MLL:)")
        epicsEnvSet("R", "$(R=DDRIVE:)")
        ```
       Set the second quoted strings appropriately.
    3. The following line sets the IP address of the serial device server communicating with the ddrive:
        ```
        epicsEnvSet("DDRIVE_NET_IP", "$(DDRIVE_NET_IP=10.0.0.11)")
        epicsEnvSet("DDRIVE_NET_PORT", "$(DDRIVE_NET_PORT=4016)")
        ```
       Change the 10.0.0.11 to the IP address, and 4016 to the correct port number.
    4. Alternatively, if you have the device directly connected to a serial port on the machine,
       uncomment and modify the drvAsynSerialPortConfigure/asynSetOption lines. Comment out drvAsynIPPortConfigure.
    5. If necessary, you can change the rate at which the controller is polled for positions and such:
        ```
        #ddriveCreateController(portName, ddrivePortName, numAxes, movingPollPeriod, idlePollPeriod)
        ddriveCreateController("DDRIVE0", "$(ASYN_PORT)", 3, 50, 100)
        ```
        The moving and idle poll periods are both in milliseconds. The former rate is used when an axis is in motion, the latter otherwise.
    6. If using autosave, uncomment create_monitor_set lines. Add lines in auto_positions.req and auto_settings.req for each motor.
4.  Edit iocBoot/iocddrive/ddrive.sub
    Modify the lines so that there is one motor.db line per axis. Each will be named $(P)$(M).
    Additional control outside of that available in the EPICS motor record is given in ddrive_mbb.db, so similarly add one line for each of those if necessary.
5. An asyn
6. Go to the top directory and `make`
7. If all goes well:
    ```
    cd iocBoot/iocddrive
    chmod +x st.cmd`
    ./st.cmd
    ```

8. Run EDM:
    ```
    export EDMDATAFILES=$TOP/op/edl:$EDMDATAFILES
    (general motor settings configuration)
    edm -x -m "P=MLL:ddrive:,M=m1" motorx_all &

    (d-drive specific settings)
    edm -x -m "P=MLL:,R=DDRIVE:AX0:" ddrive_all &
    ```

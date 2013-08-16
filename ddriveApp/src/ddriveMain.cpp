#include "ddrive.h"

#ifndef strnchr
char* strnchr(const char* str, size_t len, char c) {
  if (!str)
    return NULL;

  while (len > 0 && *str != '\0') {
    if (*str == c) {
      return (char*)str;
    }
    str++;
    len--;
  }

  if (len > 0 && *str == '\0' && c == '\0')
    return (char*)str;

  return NULL;
}
#endif

static ELLLIST ddriveList;
static int ddriveListInitialized = 0;

bool addToList(const char *portName, ddriveController *drv) {
    if (!ddriveListInitialized) {
        ddriveListInitialized = 1;
        ellInit(&ddriveList);
    } else if (findByPortName(portName) != NULL) {
        fprintf(stderr, "ERROR: Re-using portName=%s\n", portName);
        return false;
    }

    ddriveNode *pNode = (ddriveNode*)calloc(1, sizeof(ddriveNode));
    pNode->portName = epicsStrDup(portName);
    pNode->pController = drv;
    ellAdd(&ddriveList, (ELLNODE*)pNode);
    return true;
}

ddriveController* findByPortName(const char *portName) {
    ddriveNode *pNode;
    static const char *functionName = "findByPortName";

    // Find this 
    if (!ddriveListInitialized) {
        printf("%s:%s: ERROR, ddrive list not initialized\n",
            driverName, functionName);
        return NULL;
    }

    pNode = (ddriveNode*)ellFirst(&ddriveList);
    while(pNode) {
        if (!strcmp(pNode->portName, portName)) {
            return pNode->pController;
        }
        pNode = (ddriveNode*)ellNext((ELLNODE*)pNode);
    }

    printf("%s: ddrive on port %s not found\n",
        driverName, portName);
    return NULL;
}


///// ddriveCreateController
//
/** Creates a new ddriveController object.
  * Configuration command, called directly or from iocsh
  * \param[in] type              The type of the controller [Use GCS for fully GCS-compatible controllers] (GCS, E-755, ...)
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ddrivePortName     The name of the drvAsynIPPPort that was created previously to connect to the ddrive controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int ddriveCreateController(const char *portName, const char *ddrivePortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  new ddriveController(portName, ddrivePortName, numAxes, movingPollPeriod, idlePollPeriod);
  return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ddriveCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg ddriveCreateControllerArg1 = {"ddrive port name", iocshArgString};
static const iocshArg ddriveCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg ddriveCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg ddriveCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const ddriveCreateControllerArgs[] = {&ddriveCreateControllerArg0,
                                                               &ddriveCreateControllerArg1,
                                                               &ddriveCreateControllerArg2,
                                                               &ddriveCreateControllerArg3,
                                                               &ddriveCreateControllerArg4
                                                            };
static const iocshFuncDef ddriveCreateControllerDef = {"ddriveCreateController", 5, ddriveCreateControllerArgs};
static void ddriveCreateControllerCallFunc(const iocshArgBuf *args)
{
  ddriveCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


/***********************************************************************/
static void ddriveMotorRegister(void)
{
  iocshRegister(&ddriveCreateControllerDef, ddriveCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(ddriveMotorRegister);
}

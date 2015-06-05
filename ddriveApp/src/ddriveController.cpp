/*
 * PiezoSystem Jena D-Drive controller driver
 *
 * */
// vim: tabstop=2 shiftwidth=2
#include "ddrive.h"


/** Creates a new ddriveController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] ddrivePortName     The name of the drvAsynIPPPort that was created previously to connect to the ddrive controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
ddriveController::ddriveController(const char *portName, const char *ddrivePortName, int numAxes,
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_DDRIVE_PARAMS,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  //ddriveAxis *pAxis;

  queryRate_ = 1;

  position_move_timeout_ = DDRIVE_MOVE_TIMEOUT;
  dd_param_count_ = NUM_DDRIVE_CONTROLLER_PARAMS; 
  idlePollPeriod_ = idlePollPeriod;
  movingPollPeriod_ = movingPollPeriod;
  timeout_ = DDRIVE_TIMEOUT;

  ddparams_ = new DDParam[NUM_DDRIVE_CONTROLLER_PARAMS];

  // R+W parameters
  //            command to send,      asyn string,          type,               parameter index, poll, global, service mode)
  createDDParam(DD_PARAM_CMD_KTEMP,   DD_PARAM_STR_KTEMP,   asynParamFloat64,  &param_ktemp_);
  createDDParam(DD_PARAM_CMD_ROHM,    DD_PARAM_STR_ROHM,    asynParamInt32,    &param_rohm_);
  createDDParam(DD_PARAM_CMD_RGVER,   DD_PARAM_STR_RGVER,   asynParamInt32,    &param_rgver_);
  createDDParam(DD_PARAM_CMD_FENABLE, DD_PARAM_STR_FENABLE, asynParamInt32,    &param_fenable_);
  createDDParam(DD_PARAM_CMD_SR,      DD_PARAM_STR_SR,      asynParamFloat64,  &param_sr_);
  createDDParam(DD_PARAM_CMD_MODON,   DD_PARAM_STR_MODON,   asynParamInt32,    &param_modon_);
  createDDParam(DD_PARAM_CMD_KP,      DD_PARAM_STR_KP,      asynParamFloat64,  &param_kp_);
  createDDParam(DD_PARAM_CMD_KI,      DD_PARAM_STR_KI,      asynParamFloat64,  &param_ki_);
  createDDParam(DD_PARAM_CMD_KD,      DD_PARAM_STR_KD,      asynParamFloat64,  &param_kd_);
  createDDParam(DD_PARAM_CMD_NOTCHON, DD_PARAM_STR_NOTCHON, asynParamInt32,    &param_notchon_);
  createDDParam(DD_PARAM_CMD_NOTCHF,  DD_PARAM_STR_NOTCHF,  asynParamInt32,    &param_notchf_);
  createDDParam(DD_PARAM_CMD_NOTCHB,  DD_PARAM_STR_NOTCHB,  asynParamInt32,    &param_notchb_);
  createDDParam(DD_PARAM_CMD_LPON,    DD_PARAM_STR_LPON,    asynParamInt32,    &param_lpon_);
  createDDParam(DD_PARAM_CMD_LPF,     DD_PARAM_STR_LPF,     asynParamInt32,    &param_lpf_);
  createDDParam(DD_PARAM_CMD_GASIN,   DD_PARAM_STR_GASIN,   asynParamFloat64,  &param_gasin_);
  createDDParam(DD_PARAM_CMD_GOSIN,   DD_PARAM_STR_GOSIN,   asynParamFloat64,  &param_gosin_);
  createDDParam(DD_PARAM_CMD_GFSIN,   DD_PARAM_STR_GFSIN,   asynParamFloat64,  &param_gfsin_);
  createDDParam(DD_PARAM_CMD_GATRI,   DD_PARAM_STR_GATRI,   asynParamFloat64,  &param_gatri_);
  createDDParam(DD_PARAM_CMD_GOTRI,   DD_PARAM_STR_GOTRI,   asynParamFloat64,  &param_gotri_);
  createDDParam(DD_PARAM_CMD_GFTRI,   DD_PARAM_STR_GFTRI,   asynParamFloat64,  &param_gftri_);
  createDDParam(DD_PARAM_CMD_GSTRI,   DD_PARAM_STR_GSTRI,   asynParamFloat64,  &param_gstri_);
  createDDParam(DD_PARAM_CMD_GAREC,   DD_PARAM_STR_GAREC,   asynParamFloat64,  &param_garec_);
  createDDParam(DD_PARAM_CMD_GOREC,   DD_PARAM_STR_GOREC,   asynParamFloat64,  &param_gorec_);
  createDDParam(DD_PARAM_CMD_GFREC,   DD_PARAM_STR_GFREC,   asynParamFloat64,  &param_gfrec_);
  createDDParam(DD_PARAM_CMD_GSREC,   DD_PARAM_STR_GSREC,   asynParamFloat64,  &param_gsrec_);
  createDDParam(DD_PARAM_CMD_GANOI,   DD_PARAM_STR_GANOI,   asynParamFloat64,  &param_ganoi_);
  createDDParam(DD_PARAM_CMD_GONOI,   DD_PARAM_STR_GONOI,   asynParamFloat64,  &param_gonoi_);
  createDDParam(DD_PARAM_CMD_GASWE,   DD_PARAM_STR_GASWE,   asynParamFloat64,  &param_gaswe_);
  createDDParam(DD_PARAM_CMD_GOSWE,   DD_PARAM_STR_GOSWE,   asynParamFloat64,  &param_goswe_);
  createDDParam(DD_PARAM_CMD_GTSWE,   DD_PARAM_STR_GTSWE,   asynParamFloat64,  &param_gtswe_);
  createDDParam(DD_PARAM_CMD_TRGSS,   DD_PARAM_STR_TRGSS,   asynParamFloat64,  &param_trgss_);
  createDDParam(DD_PARAM_CMD_TRGSE,   DD_PARAM_STR_TRGSE,   asynParamFloat64,  &param_trgse_);
  createDDParam(DD_PARAM_CMD_TRGSI,   DD_PARAM_STR_TRGSI,   asynParamFloat64,  &param_trgsi_);
  createDDParam(DD_PARAM_CMD_TRGLEN,  DD_PARAM_STR_TRGLEN,  asynParamInt32,    &param_trglen_);

  // MBBIO parameters
  createDDParam(DD_PARAM_CMD_MONSRC,  DD_PARAM_STR_MONSRC,  asynParamUInt32Digital, &param_monsrc_);
  createDDParam(DD_PARAM_CMD_GFKT,    DD_PARAM_STR_GFKT,    asynParamUInt32Digital, &param_gfkt_);
  createDDParam(DD_PARAM_CMD_SCT,     DD_PARAM_STR_SCT,     asynParamUInt32Digital, &param_sct_); // sometimes has issues reading back?
  createDDParam(DD_PARAM_CMD_TRGEDGE, DD_PARAM_STR_TRGEDGE, asynParamUInt32Digital, &param_trgedge_);

  // Global parameters                                                                                        
  createDDParam(DD_PARAM_CMD_BRIGHT,  DD_PARAM_STR_BRIGHT,  asynParamInt32,    &param_bright_,   true,  true);
                                                                                                              
  // Write-only                                                                                               
  createDDParam(DD_PARAM_CMD_SSTD,    DD_PARAM_STR_SSTD,      asynParamInt32,  &param_sstd_,    false, false); // set default values
  createDDParam(DD_PARAM_CMD_CL,      DD_PARAM_STR_CL,        asynParamInt32,  &param_cl_,      false, false); // open/closed loop
  createDDParam(DD_PARAM_CMD_FBREAK,  DD_PARAM_STR_FBREAK,    asynParamInt32,  &param_fbreak_,  false, false); // abort soft start
  // start scan sometimes doesn't readback... let's just not query it
  createDDParam(DD_PARAM_CMD_SS,      DD_PARAM_STR_SS,      asynParamInt32,    &param_ss_,      false, false);
  
  // Special handling:
  createDDParam(DD_PARAM_CMD_WRMB,      DD_PARAM_STR_ENC_RATE, asynParamInt32,   &param_enc_rate_, false, false, true, DD_PARAM_SVC_ENC_RATE);
  // Last of queryable-DDrive parameters
  
  // Read-only (updated when status changes)
  createParam(DD_PARAM_STR_PLUGGED,               asynParamInt32,    &param_plugged_);
  createParam(DD_PARAM_STR_CLOSED_SYS,            asynParamInt32,    &param_closed_sys_);
  createParam(DD_PARAM_STR_VOLTAGE_ON,            asynParamInt32,    &param_voltage_on_);
  createParam(DD_PARAM_STR_MEAS_SYS,      asynParamUInt32Digital,    &param_meas_sys_);

  // Read-write
  createParam(DD_PARAM_STR_MOVE_TIMEOUT,  asynParamFloat64,  &param_move_timeout_);

  if (!addToList(portName, this)) {
    printf("%s:%s: Init failed", driverName, portName);
    return;
  }

  setIntegerParam(motorStatusHasEncoder_, 1);

  /* Connect to ddrive controller */
  status = pasynOctetSyncIO->connect(ddrivePortName, 0, &pasynUser_, NULL);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: cannot connect to ddrive controller\n",
      driverName, __func__);
  }

  status = pasynOctetSyncIO->setInputEos(pasynUser_, "\r", 1);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set input EOS on %s: %s\n",
      __func__, ddrivePortName, pasynUser_->errorMessage);
  }

  status = pasynOctetSyncIO->setOutputEos(pasynUser_, "\r", 1);
  if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR|ASYN_TRACE_FLOW,
      "%s: Unable to set output EOS on %s: %s\n",
      __func__, ddrivePortName, pasynUser_->errorMessage);
  }

  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    //pAxis = 
    new ddriveAxis(this, axis);
  }
  startPoller(movingPollPeriod/1000., idlePollPeriod/1000., 2);
}


void ddriveController::enterServiceMode() {
  if (!service_mode_) {
#if DEBUG
    printf("Entering service mode\n");
#endif
    write("*SetA");
    service_mode_ = true;
  }
}

void ddriveController::exitServiceMode() {
  if (service_mode_) {
#if DEBUG
    printf("Exiting service mode\n");
#endif
    write("*SetA,0");
    service_mode_ = false;
  }
}

void ddriveController::createDDParam(const char *cmd, const char *param_str, asynParamType type, 
                                     int *param_idx, bool query, bool global, bool service_mode, int service_param) {
  static int i=0;
  createParam(param_str, type, param_idx);
  ddparams_[i].asyn_param = param_str;
  ddparams_[i].command = cmd;
  ddparams_[i].type = type;
  ddparams_[i].query = query;
  ddparams_[i].global = global;
  ddparams_[i].idx = *param_idx;
  ddparams_[i].service_mode = service_mode;
  ddparams_[i].service_param = service_param;
  printf("Parameter %s is index %d\n", cmd, *param_idx);
  i++;
}

const DDParam *ddriveController::DDParamFromIndex(int param_index) {
  for (int i=0; i < NUM_DDRIVE_CONTROLLER_PARAMS; i++) {
    if (ddparams_[i].idx == param_index)
      return (const DDParam*)&ddparams_[i];
  }
  return NULL;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorSetClosedLoop_ then it turns the drive power on or off.
  * If the function is ddriveReadBinaryIO_ then it reads the binary I/O registers on the controller.
  * For all other functions it calls asynMotorController::writeFloat64.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus ddriveController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ddriveAxis *pAxis = getAxis(pasynUser);
  const char *paramName = "(unset)";

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setDoubleParam(pAxis->axisNo_, function, value);

  const DDParam *ddp = DDParamFromIndex(function);

  if (ddp && ddp->type == asynParamFloat64) {
    if (ddp->global) {
        status = writeParameter(ddp, value);
    } else {
        status = pAxis->writeParameter(ddp, value);
    }
  } else if (function == param_move_timeout_) {
    position_move_timeout_ = value;
    printf("Move timeout set to: %g\n", value);
  } else {
    /* Call base class method */
    status = asynMotorController::writeFloat64(pasynUser, value);
  }
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%s (%d), value=%f\n", 
        driverName, __func__, status, paramName, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%s (%d), value=%f\n", 
        driverName, __func__, paramName, function, value);
  return status;
}

/** Called when asyn clients call pasynUIntDigital->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorSetClosedLoop_ then it turns the drive power on or off.
  * If the function is ddriveReadBinaryIO_ then it reads the binary I/O registers on the controller.
  * For all other functions it calls asynMotorController::writeUInt32Digital.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus ddriveController::writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ddriveAxis *pAxis = getAxis(pasynUser);
  const char *paramName = "(unset)";

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setUIntDigitalParam(pAxis->axisNo_, function, value, mask);

  printf("%s:%s: mask=%x function=%s (%d), value=%d\n", 
        driverName, __func__, mask, paramName, function, value);
  const DDParam *ddp = DDParamFromIndex(function);

  if (ddp && ddp->type == asynParamUInt32Digital) {
    if (ddp->global) {
        status = writeParameter(ddp, (int)value);
    } else {
        status = pAxis->writeParameter(ddp, (int)value);
    }
  } else {
    /* Call base class method */
    status = asynMotorController::writeUInt32Digital(pasynUser, value, mask);
  }
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%s (%d), value=%d\n", 
        driverName, __func__, status, paramName, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%s (%d), value=%d\n", 
        driverName, __func__, paramName, function, value);
  return status;
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorSetClosedLoop_ then it turns the drive power on or off.
  * If the function is ddriveReadBinaryIO_ then it reads the binary I/O registers on the controller.
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus ddriveController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  ddriveAxis *pAxis = getAxis(pasynUser);
  const char *paramName = "(unset)";

  /* Fetch the parameter string name for possible use in debugging */
  getParamName(function, &paramName);

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(pAxis->axisNo_, function, value);

  const DDParam *ddp = DDParamFromIndex(function);

  if (ddp && ddp->type == asynParamInt32) {
    if (ddp->global) {
        status = writeParameter(ddp, value);
    } else {
        status = pAxis->writeParameter(ddp, value);
    }
  } else {
    /* Call base class method */
    status = asynMotorController::writeInt32(pasynUser, value);
  }
  
  /* Do callbacks so higher layers see any changes */
  callParamCallbacks(pAxis->axisNo_);
  if (status) 
    asynPrint(pasynUser, ASYN_TRACE_ERROR, 
        "%s:%s: error, status=%d function=%s (%d), value=%d\n", 
        driverName, __func__, status, paramName, function, value);
  else    
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
        "%s:%s: function=%s (%d), value=%d\n", 
        driverName, __func__, paramName, function, value);
  return status;
}

asynStatus ddriveController::write(const char *fmt, va_list argptr) {
  size_t nwrite;
  char buf[DD_STRING_LEN];

  vsnprintf(buf, DD_STRING_LEN, fmt, argptr);

#if DEBUG
  fprintf(stderr, "%s:%s: %s\n", driverName, __func__, buf);
#endif

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s:%s: %s\n",
    driverName, __func__, buf);

  return pasynOctetSyncIO->write(pasynUser_,
                                 buf, strlen(buf),
                                 timeout_, &nwrite);
}

asynStatus ddriveController::write(const char *fmt, ...) {
  va_list argptr;
  va_start(argptr,fmt);
  asynStatus ret=write(fmt, argptr);
  va_end(argptr);
  return ret;
}

asynStatus ddriveController::writeReadMatch(char *ret_buf, const char* fmt, va_list argptr) {
  char output[DD_STRING_LEN];
  vsnprintf(output, DD_STRING_LEN, fmt, argptr);

  size_t nread;
  char input[DD_STRING_LEN];
  asynStatus status;

  status = writeRead(input, (char*)output, &nread);

  if (status != asynSuccess) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
        "%s: writeRead failed: %s (ret=%d) received: %s (len=%ld)\n",
        __func__, output, status, input, nread);
    fprintf(stderr, "%s: writeRead failed: %s (ret=%d) received: %s (len=%ld)\n",
            __func__, output, status, input, nread);
    if (nread > 0)
      strncpy(ret_buf, input, nread + 1);
    else
      ret_buf[0] = 0;
    return asynError;
  }

  // assume wrote something like:
  // mess,0
  // then the output should match the input up to the input's length:
  // mess,0      <-- in 
  // mess,0,10.0 <-- out
  // match^
  if (strncmp(input, output, strlen(output))) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s: writeRead failed: %s input mismatch: %s\n",
      __func__, output, input);
    fprintf(stderr,
      "%s: writeRead failed: %s input mismatch: %s\n",
      __func__, output, input);

    if (nread > 0)
      strncpy(ret_buf, input, nread + 1);
    else
      ret_buf[0] = 0;
    return asynError;
  }

  char *response=&input[strlen(output)];

  if (response[0] != ',') {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
      "%s: writeRead failed: %s input mismatch: %s -> %s\n",
      __func__, output, input, response);
    return asynError;
  } else {
    response++;
    strncpy(ret_buf, response, DD_STRING_LEN - strlen(output) - 1);
    return asynSuccess;
  }
  return writeReadMatch(ret_buf, output);
}

asynStatus ddriveController::writeReadMatch(char *ret_buf, const char* fmt, ...) {
  va_list argptr;
  va_start(argptr, fmt);
  asynStatus ret=writeReadMatch(ret_buf, fmt, argptr);
  va_end(argptr);
  return ret;
}

asynStatus ddriveController::writeRead(int &value, const char* fmt, va_list argptr) {
  char response[DD_STRING_LEN];
  asynStatus ret = writeReadMatch(response, fmt, argptr);
  if (ret == asynSuccess) {
    value = atoi(response);
    //printf("int value received: (%s) %d\n", response, value);
  }
  return asynSuccess;
}

asynStatus ddriveController::writeRead(int& value, const char* fmt, ...) {
  va_list argptr;
  va_start(argptr,fmt);
  asynStatus ret=writeRead(value, fmt, argptr);
  va_end(argptr);
  return ret;
}

asynStatus ddriveController::writeRead(double &value, const char* fmt, va_list argptr) {
  char response[DD_STRING_LEN];
  asynStatus ret = writeReadMatch(response, fmt, argptr);
  if (ret == asynSuccess) {
    value = atof(response);
    //printf("double value received: (%s) %f\n", response, value);
  }
  return asynSuccess;
}

asynStatus ddriveController::writeRead(double& value, const char* fmt, ...) {
  va_list argptr;
  va_start(argptr,fmt);
  asynStatus ret=writeRead(value, fmt, argptr);
  va_end(argptr);
  return ret;
}

asynStatus ddriveController::writeRead(float& value, const char* fmt, ...) {
  double dval;

  va_list argptr;
  va_start(argptr,fmt);
  asynStatus ret=writeRead(dval, fmt, argptr);
  va_end(argptr);

  value = dval;
  return ret;
}


asynStatus ddriveController::writeRead(char *buf_input, char *output, size_t* nread) {
  size_t nwrite;
  asynStatus status;
  int eomReason;

  if (!buf_input || !output) {
    return asynError;
  }

  lock();

  asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
    "%s:%s: Write: %s\n",
    driverName, __func__, output);

  status = pasynOctetSyncIO->writeRead(pasynUser_,
                                       output, strlen(output),
                                       buf_input, DD_STRING_LEN,
                                       timeout_, &nwrite, nread, &eomReason);

  unlock();

  if (!strnchr(buf_input, DD_STRING_LEN, 0)) {
    asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER|ASYN_TRACE_ERROR,
        "%s: writeRead failed: %s input buffer size (%d, nread=%ld, reason=%d)",
        __func__, output, DD_STRING_LEN, nread, eomReason);
    return asynError;
  } else {
    asynPrint(pasynUser_, ASYN_TRACEIO_DRIVER,
      "%s:%s: Read (%ldb): %s\n",
      driverName, __func__, *nread, buf_input);
  }

  return status;
}

asynStatus ddriveController::writeRead(char *input, size_t* nread, const char *fmt, va_list argptr) {
  char output[DD_STRING_LEN];

  vsnprintf(output, DD_STRING_LEN, fmt, argptr);
  return writeRead(input, nread, output);
}

asynStatus ddriveController::writeRead(char *input, size_t *nread, const char *fmt, ...) {
  va_list argptr;
  va_start(argptr,fmt);
  asynStatus ret=writeRead(input, nread, fmt, argptr);
  va_end(argptr);
  return ret;
}

bool ddriveController::format(int &len, char *buf, const char *fmt, ...) {
  if (!buf) {
	  return false;
  }

  va_list argptr;
  va_start(argptr, fmt);
  format(len, buf, fmt, argptr);
  va_end(argptr);
  return (len >= 0);
}

bool ddriveController::format(int &len, char *buf, const char *fmt, va_list argptr) {
  if (!buf) {
	  return false;
  }

  len = vsnprintf(buf, DD_STRING_LEN, fmt, argptr);
  return (len >= 0);
}

asynStatus ddriveController::writeParameter(int idx, int value) {
  if (idx < 0 || idx > dd_param_count_)
    return asynError;

  const DDParam *param = &ddparams_[idx];
  return writeParameter(param, value);
}



asynStatus ddriveController::writeParameter(const DDParam *param, int value) {
  if (!param || !param->global)
    return asynError;

  return write("%s,%d", param->command, value);
}

asynStatus ddriveController::writeParameter(int idx, double value) {
  if (idx < 0 || idx > dd_param_count_)
    return asynError;

  const DDParam *param = &ddparams_[idx];
  return writeParameter(param, value);
}

asynStatus ddriveController::writeParameter(const DDParam *param, double value) {
  if (!param || !param->global)
    return asynError;

  // %g formatting good enough?
  return write("%s,%g", param->command, value);
}

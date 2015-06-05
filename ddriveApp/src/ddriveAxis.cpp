/*
 * PiezoSystem Jena D-Drive controller driver
 *
 * */
// vim: tabstop=2 shiftwidth=2
#include "ddrive.h"

/** Creates a new ddriveAxis object.
  * \param[in] controller         The ddrive controller
  * \param[in] axis_num           The axis number (1-based)
  */
ddriveAxis::ddriveAxis(ddriveController *controller, int axis_num)
  :  asynMotorAxis((asynMotorController*)controller, axis_num)
{
    pc_ = controller;
    encoder_pos_ = 0.0;
    param_num_ = -1;
    mode_ = DD_MODE_POSITION;
    moving_ = false;
    initial_query_ = true;
    query_status_ = true;
}

asynStatus ddriveAxis::poll(bool *moving) {
  pc_->lock();
  queryPosition();
  
  if (query_status_) {
    queryStatus();
#if !DDRIVE_FAST_STATUS
    query_status_ = false;
#endif
  }

  checkMoving();
  queryParameters();
  pc_->unlock();

  *moving = moving_;
  callParamCallbacks();

  return asynSuccess;
}

asynStatus ddriveAxis::queryPosition() {
  asynStatus ret = pc_->writeRead(encoder_pos_, "mess,%d", axis_num_);
  if (ret == asynSuccess) {
    setDoubleParam(pc_->motorEncoderPosition_, encoder_pos_ / DDRIVE_COUNTS_TO_UM);
    setDoubleParam(pc_->motorPosition_, encoder_pos_ / DDRIVE_COUNTS_TO_UM);
  }
	return ret;
}

template <typename T>
asynStatus ddriveAxis::queryParameter(const DDParam *param, T& value) {
  const char *command = param->command;
  
  if (!command) {
    return asynError;
  }
  
  if (param->service_mode) {
    asynStatus ret = asynError;
  
  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
            "(Axis %d) Querying service mode param %d\n",
            axisNo_, param->service_param);

    pc_->enterServiceMode();
    char read_buf[DD_STRING_LEN];
    // Maybe they'll fix the out-of-order response:
    // rdmb,0,109
    // rdmb109,0,1000
    if (pc_->writeReadMatch(read_buf, "rdmb,%d,%d", axisNo_, param->service_param) == asynError) {
#if DEBUG
      asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
                "(Axis %d) Querying service mode param %d, %s\n", 
                axisNo_, param->service_param, read_buf);
#endif
      // Failed, assume this format:
      // rdmb109,0,1000
      if (read_buf[0] != 0) {
        // read a response
        for (int i=strlen(read_buf); i > 0; i--) {
          if (read_buf[i] == ',') {
            // only support int values, for now (?)
            //value = static_cast<T>(atoi(&read_buf[i+1]));
            value = atoi(&read_buf[i+1]);
            ret = asynSuccess;
            asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
                      "Read out of order value for service param #%d: %d\n",
                      param->service_param, (int)value);
            break;
          }
        }
      } else {
        fprintf(stderr, "Failed to readback service mode parameter (%d).\n", param->service_param);
      }
    } else {
      // Success, was in this format:
      // rdmb,0,109,1000
      value = atoi(read_buf);
    }
    pc_->exitServiceMode();
    return ret;
  }
  else if (param->global) {
    // TODO: global parameters get updated more often than others --
    // could tie the query to a single axis, but then it would never
    // be queried if that axis didn't exist... instead, should probably
    // query it with the controller. need additional polling thread.
    return pc_->writeRead(value, "%s", command, axisNo_);
  } else {
    return pc_->writeRead(value, "%s,%d", command, axisNo_);
  }
}

asynStatus ddriveAxis::writeParameter(int idx, int value) {
  if (idx < 0 || idx > pc_->dd_param_count_)
    return asynError;
  
  const DDParam *param = &pc_->ddparams_[idx];
  param_num_ = idx - 1; // query this parameter the next time
  return writeParameter(param, value);
}

asynStatus ddriveAxis::writeParameter(const DDParam *param, int value) {
  if (!param || param->global)
    return asynError;

  if (param->service_param) {
    pc_->enterServiceMode();

    // Write the parameter
    pc_->write("wrmb,%d,%d,%d", axisNo_, param->service_param, value);
    asynStatus ret = queryParameter(param, value);
    if (ret == asynSuccess) 
      setIntegerParam(param->idx, value);

    pc_->exitServiceMode(); // queryparameter() should already do this
    return ret;
  } else {
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
              "-> write param: %s,%d,%d\n", 
              param->command, axisNo_, value);
    return pc_->write("%s,%d,%d", param->command, axisNo_, value);
  }
}

asynStatus ddriveAxis::writeParameter(int idx, double value) {
  if (idx < 0 || idx > pc_->dd_param_count_)
    return asynError;
  
  param_num_ = idx - 1; // query this parameter the next time

  const DDParam *param = &pc_->ddparams_[idx];
  return writeParameter(param, value);
}

asynStatus ddriveAxis::writeParameter(const DDParam *param, double value) {
  if (!param || param->global)
    return asynError;

  // %g formatting good enough?
  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
            "write param: %s,%d,%g\n", 
            param->command, axisNo_, value);
  return pc_->write("%s,%d,%g", param->command, axisNo_, value);
}

asynStatus ddriveAxis::queryParameters() {
  int last_param = (param_num_ + pc_->queryRate_) % pc_->dd_param_count_;
  DDParam *param = NULL;
  do {
    param_num_ = (param_num_ + 1) % pc_->dd_param_count_;
    param = &pc_->ddparams_[param_num_];

    if (param->query || (initial_query_ && param->service_mode)) {
      if (param->type == asynParamInt32 || param->type == asynParamUInt32Digital) {
        int value;
        if (queryParameter(param, value) == asynSuccess) {
          if (param->type == asynParamInt32) {
            setIntegerParam(param->idx, value);
          } else {
            setUIntDigitalParam(param->idx, value);
            // NOTE: these will not work pre asyn-4-18 (? at least with the debian packages it does not work TODO check its version)
          }
        //printf("Queried parameter Axis: %d (#%d) %s %s = %d\n", axis_num_, param_num_, param->asyn_param, param->command, value);
        }
      } else if (param->type == asynParamFloat64) {
        double value;
        if (queryParameter(param, value) == asynSuccess)
          setDoubleParam(param->idx, value);
        //printf("Queried parameter Axis: %d (#%d) %s %s = %f\n", axis_num_, param_num_, param->asyn_param, param->command, value);
      }
    }
  } while (param_num_ != last_param);

  if ((param_num_ + pc_->queryRate_) >= pc_->dd_param_count_) {
    // Only query service-mode parameters initially or when one is changed
    initial_query_ = false;
    query_status_ = true;
  }

  return asynSuccess;
}

asynStatus ddriveAxis::queryStatus() {
  int status=0;
  asynStatus ret = pc_->writeRead(status, "stat,%d", axis_num_);

  if (ret == asynSuccess && status != status_) {
    // only evaluate when status has changed
    status_ = status;
    bool plugged = (status & DD_STATUS_PLUGGED) == DD_STATUS_PLUGGED;
    int measure = (status & DD_STATUS_MEAS_MASK) >> DD_STATUS_MEAS_MASK_OFFSET;
    bool closed_loop_sys = (status & DD_STATUS_CLOSED_LOOP_SYS) == DD_STATUS_CLOSED_LOOP_SYS;
    bool voltage_enabled = (status & DD_STATUS_VOLTAGE_EN) == DD_STATUS_VOLTAGE_EN;
    bool closed_loop = (status & DD_STATUS_CLOSED_LOOP) == DD_STATUS_CLOSED_LOOP;
    int generator = (status & DD_STATUS_GENERATOR_MASK) >> DD_STATUS_GEN_MASK_OFFSET;
    bool notch_filter = (status & DD_STATUS_NOTCH_FILTER) == DD_STATUS_NOTCH_FILTER;
    bool lp_filter = (status & DD_STATUS_LP_FILTER) == DD_STATUS_LP_FILTER;

    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "-- Axis %d: status changed --\n", axisNo_);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tPlugged: %d\n", plugged);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tMeasurement system: %s (%d)\n", DDRIVE_MEAS_STRINGS[measure], measure);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tClosed loop system: %d\n", closed_loop_sys);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tIn closed loop: %d\n", closed_loop);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tVoltage enabled: %d\n", voltage_enabled);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tGenerator: %s (%d)\n", DDRIVE_GEN_STRINGS[generator], generator);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tNotch filter enabled: %d\n", notch_filter);
    asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
              "\tLow-pass filter enabled: %d\n", lp_filter);
    
    setIntegerParam(pc_->param_notchon_, notch_filter);
    setIntegerParam(pc_->param_lpon_, lp_filter);
    setIntegerParam(pc_->param_cl_, closed_loop);
    setIntegerParam(pc_->param_closed_sys_, closed_loop_sys);
    setIntegerParam(pc_->param_voltage_on_, voltage_enabled);
    setUIntDigitalParam(pc_->param_meas_sys_, measure);
    setUIntDigitalParam(pc_->param_gfkt_, generator);

    if (generator == DD_STATUS_GEN_OFF) {
      mode_ = DD_MODE_POSITION;
    } else {
      mode_ = DD_MODE_SCAN;
    }
  }

  return ret;
}

bool ddriveAxis::checkMoving() {
  if (mode_ == DD_MODE_POSITION) {
    if (moving_) {
      // already locked in ::poll(), but in case this gets called elsewhere...
      pc_->lock();
    
      double enc_diff = fabs(move_target_ - encoder_pos_);
      asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
                "Moving. Target: %f: encoder: %f (difference %f)\n",
                move_target_, encoder_pos_, enc_diff);

      if (epicsTime::getCurrent() > t_move_timeout_ || enc_diff <= DDRIVE_MOVE_RESOLUTION) {
        double t = epicsTime::getCurrent() - t_move_start_;
        asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW, 
                  "Moved in %.3f sec. Encoder difference: %g\n",
                  t, enc_diff);
        motionFinished();
      }
      pc_->unlock();
    } else if (initial_query_) {
      motionFinished();
    }

  }
  return moving_;
}

void ddriveAxis::motionFinished() {
  setIntegerParam(pc_->motorStatusMoving_, 0);
  setIntegerParam(pc_->motorStatusDone_, 1);
  moving_ = false;
}

asynStatus ddriveAxis::stop(double acceleration)
{
  if (mode_ == DD_MODE_POSITION) {
    motionFinished();
    return asynSuccess;
  } else {
    return pc_->write("ss,%d,0", axis_num_);
  }
}

asynStatus ddriveAxis::moveVelocity(double min_velocity, double max_velocity, double acceleration) {
  asynStatus ret=asynError; //pc_->write("%dJOG50.0");
  return ret;
}

asynStatus ddriveAxis::home(double min_velocity, double max_velocity, double acceleration, int forwards) {
  return move(0.0, 0, 0.0, 0.0, 0.0);
}

asynStatus ddriveAxis::move(double position, int relative, double min_velocity, double max_velocity, double acceleration)
{
  asynPrint(pc_->pasynUser_, ASYN_TRACE_FLOW,
            "%s:%s: axis %d: move to %g (relative=%d)\n",
            driverName, __func__, axis_num_, 
            position, relative);

  if (mode_ == DD_MODE_POSITION) {
    position *= DDRIVE_COUNTS_TO_UM;
    max_velocity *= DDRIVE_COUNTS_TO_UM;

    pc_->lock();
    // if multiple moves are called, ensure that the move target is always the latest
    move_target_ = position;
    moving_ = true;
    t_move_start_ = epicsTime::getCurrent();
    t_move_timeout_ = t_move_start_ + pc_->position_move_timeout_;
    pc_->unlock();

    // No actual response from the controller even with invalid input...
    // What to do?
    return pc_->write("set,%d,%g", axis_num_, position);
  } else {
    // TODO
    return pc_->write("ss,%d,1", axis_num_);
  }
}

const char* get_ddrive_error_string(int error) {
  switch (error) {
   case DD_ERROR_NO_ERROR:                return "No Error";
  default:
    return "Unknown error";
  }
}

asynStatus ddriveAxis::setUIntDigitalParam(int index, epicsUInt32 value) {
  return pc_->setUIntDigitalParam(axisNo_, index, value, 0xffffffff);
}


/*
 * PiezoSystem Jena D-Drive controller driver
 *
 * TODO: further scan mode support (movement status, etc)
 * */
// vim: tabstop=2 shiftwidth=2
#ifndef _DDRIVE_H
#define _DDRIVE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>

#include <iocsh.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsExport.h>

#include <asynOctetSyncIO.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

// Set this to 0 to disable debug messages
#ifndef DEBUG
#define DEBUG                       1
#endif

// Enable to update the actuator status every poll
#define DDRIVE_FAST_STATUS          false
#define DDRIVE_TIMEOUT              0.5
#define DDRIVE_MOVE_TIMEOUT         0.6
#define DD_STRING_LEN               80
#define DDRIVE_COUNTS_TO_MM         1.0e-6
#define DDRIVE_COUNTS_TO_UM         1.0e-3
#define DDRIVE_COUNTS_TO_NM         1.0e-0
#define DDRIVE_MOVE_RESOLUTION      0.005 // um

#define DD_STATUS_PLUGGED           (1 << 0)
#define DD_STATUS_MEAS_MASK         ((1 << 1) | (1 << 2))
#define DD_STATUS_MEAS_MASK_OFFSET  1
#define DD_STATUS_CLOSED_LOOP_SYS   (1 << 4)
#define DD_STATUS_VOLTAGE_EN        (1 << 6)
#define DD_STATUS_CLOSED_LOOP       (1 << 7)
#define DD_STATUS_GENERATOR_MASK    ((1 << 9) | (1 << 10) | (1 << 11))
#define DD_STATUS_GEN_MASK_OFFSET   9
#define DD_STATUS_NOTCH_FILTER      (1 << 12)
#define DD_STATUS_LP_FILTER         (1 << 13)

#define DD_STATUS_MEAS_NONE         0
#define DD_STATUS_MEAS_STRAIN       1
#define DD_STATUS_MEAS_CAP          2
#define DD_STATUS_MEAS_INDUCTIVE    3

#define DD_STATUS_GEN_OFF           0
#define DD_STATUS_GEN_SINE          1
#define DD_STATUS_GEN_TRIANGLE      2
#define DD_STATUS_GEN_RECTANGLE     3
#define DD_STATUS_GEN_NOISE         4
#define DD_STATUS_GEN_SWEEP         5

#define DD_ERROR_NO_ERROR                    0

// R+W parameters
#define DD_PARAM_STR_KTEMP             "DD_KTEMP"    // amplifier temperature value [degree Celsius]
#define DD_PARAM_STR_ROHM              "DD_ROHM"     // operation time of actuator since shipping [minutes]
#define DD_PARAM_STR_RGVER             "DD_RGVER"    // displays the version number of loopcontroller number fan switches the fan on/off 0 = off 1 = on
#define DD_PARAM_STR_FENABLE           "DD_FENABLE"  // enables the actuator soft start 0= soft start disabled 1= soft start enabled
#define DD_PARAM_STR_SR                "DD_SR"       // slew rate 0.0000002 to 500.0 [V/ms], attributed to the modulation voltage (0 to 10V) modon modulation input
#define DD_PARAM_STR_MODON             "DD_MODON"    // plug 0 = off 1 = on
#define DD_PARAM_STR_MONSRC            "DD_MONSRC"   // monitor output (0 = default) 0 = position in closed loop 1 = command value 2 = controller output voltage 3 = closed loop deviation incl. sign 4 = absolute closed loop deviation 5 = actuator voltage 6 = position in open loop
#define DD_PARAM_STR_KP                "DD_KP"       // proportional term 0 to 999.0
#define DD_PARAM_STR_KI                "DD_KI"       // integral term 0 to 999.0
#define DD_PARAM_STR_KD                "DD_KD"       // differential term 0 to 999.0
#define DD_PARAM_STR_NOTCHON           "DD_NOTCHON"  // notch filter 0 = off 1 = on
#define DD_PARAM_STR_NOTCHF            "DD_NOTCHF"   // notch filter frequency 0 to 20000 [Hz]
#define DD_PARAM_STR_NOTCHB            "DD_NOTCHB"   // bandwidth (-3dB) 0 to 20000 (max. 2 * notch_fr) [Hz]
#define DD_PARAM_STR_LPON              "DD_LPON"     // low pass filter 0 = off 1 = on
#define DD_PARAM_STR_LPF               "DD_LPF"      // low pass cut frequency 1 to 20000 [Hz]
#define DD_PARAM_STR_GFKT              "DD_GFKT"     // internal function generator (see table 12) 0 = off 1 = sine 2 = triangle 3 = rectangle 4 = noise 5 = sweep
#define DD_PARAM_STR_GASIN             "DD_GASIN"    // generator amplitude sine 0 to 100 [%]
#define DD_PARAM_STR_GOSIN             "DD_GOSIN"    // amplitude offset sine 0 to 100 [%]
#define DD_PARAM_STR_GFSIN             "DD_GFSIN"    // generator frequency sine 0.1 to 9999.9 [Hz]
#define DD_PARAM_STR_GATRI             "DD_GATRI"    // generator amplitude triangle 0 to 100 [%]
#define DD_PARAM_STR_GOTRI             "DD_GOTRI"    // amplitude offset triangle 0 to 100 [%
#define DD_PARAM_STR_GFTRI             "DD_GFTRI"    // generator frequency triangle 0.1 to 9999.9 [Hz]
#define DD_PARAM_STR_GSTRI             "DD_GSTRI"    // symmetry of triangle 0.1 to 99.9 [%] default = 50 %
#define DD_PARAM_STR_GAREC             "DD_GAREC"    // generator amplitude rect. 0 to 100 [%]
#define DD_PARAM_STR_GOREC             "DD_GOREC"    // amplitude offset rectangle 0 to 100 [%]
#define DD_PARAM_STR_GFREC             "DD_GFREC"    // generator frequency rectangle 0.1 to 9999.9 [Hz]
#define DD_PARAM_STR_GSREC             "DD_GSREC"    // symmetry of rectangle 0.1 to 99.9 [%] default = 50 %
#define DD_PARAM_STR_GANOI             "DD_GANOI"    // generator amplitude noise 0 to 100 [%]
#define DD_PARAM_STR_GONOI             "DD_GONOI"    // amplitude offset noise 0 to 100 [%]
#define DD_PARAM_STR_GASWE             "DD_GASWE"    // generator amplitude sweep 0 to 100 [%]
#define DD_PARAM_STR_GOSWE             "DD_GOSWE"    // amplitude offset sweep 0 to 100 [%]
#define DD_PARAM_STR_GTSWE             "DD_GTSWE"    // generator sweep time 0.4 to 800 [sec/decade]
#define DD_PARAM_STR_SCT               "DD_SCT"      // scan type 0 = scan function off 1 = sine scan 2 = triangle scan
#define DD_PARAM_STR_TRGSS             "DD_TRGSS"    // trigger generation stroke position start minimum: >0.2% of total stroke to maximum: total stroke minus 0.2% of total stroke [µm] or [mrad]
#define DD_PARAM_STR_TRGSE             "DD_TRGSE"    // trigger generation stroke position end minimum: >0.2% of total stroke to maximum: total stroke minus 0.2% of total stroke [µm] or [mrad], always keep: trgse>trgss !
#define DD_PARAM_STR_TRGSI             "DD_TRGSI"    // trigger generation position intervals >0.05% of total stroke in closed loop [µm] or [mrad]
#define DD_PARAM_STR_TRGLEN            "DD_TRGLEN"   // duration of trigger impulses n*20µs  n=1...255
#define DD_PARAM_STR_TRGEDGE           "DD_TRGEDGE"  // trigger generation edge 0= trigger generation off 1= trigger generation at rising edge 2= trigger generation falling edge 3= trigger generation at both edges
#define DD_PARAM_STR_ENC_RATE          "DD_ENC_RATE" // encoder update rate (cycle time 1000...20) (*service mode*)

// Global parameters
#define DD_PARAM_STR_BRIGHT            "DD_BRIGHT"   // brightness of TFT-display 0 – display off ..  10 – maximum display brightness

// Can't read back
#define DD_PARAM_STR_SSTD              "DD_SSTD"     // set default values
#define DD_PARAM_STR_SS                "DD_SS"       // start scan without value: request scan state 1 = starts scan
#define DD_PARAM_STR_CL                "DD_CL"       // open loop / closed loop 0 = open loop 1 = closed loop
#define DD_PARAM_STR_FBREAK            "DD_FBREAK"   // aborts the actuator soft start - set command value: actuator voltage (ol) displacement (cl) -20 to 130.000 [V] 0 to xxx.xxx [μm] (maximum actuator displacement, see datasheet)

// Read-only parameters
#define DD_PARAM_STR_PLUGGED           "DD_PLUGGED"
#define DD_PARAM_STR_CLOSED_SYS        "DD_CLOSED_SYS"
#define DD_PARAM_STR_VOLTAGE_ON        "DD_VOLTAGE_ON"
#define DD_PARAM_STR_MEAS_SYS          "DD_MEAS_SYS"

// EPICS parameters
#define DD_PARAM_STR_MOVE_TIMEOUT      "DD_MOVE_TIMEOUT"

#define DD_PARAM_CMD_KTEMP             "ktemp"
#define DD_PARAM_CMD_ROHM              "rohm"
#define DD_PARAM_CMD_RGVER             "rgver"
#define DD_PARAM_CMD_FENABLE           "fenable"
#define DD_PARAM_CMD_SR                "sr"
#define DD_PARAM_CMD_MODON             "modon"
#define DD_PARAM_CMD_MONSRC            "monsrc"
#define DD_PARAM_CMD_KP                "kp"
#define DD_PARAM_CMD_KI                "ki"
#define DD_PARAM_CMD_KD                "kd"
#define DD_PARAM_CMD_NOTCHON           "notchon"
#define DD_PARAM_CMD_NOTCHF            "notchf"
#define DD_PARAM_CMD_NOTCHB            "notchb"
#define DD_PARAM_CMD_LPON              "lpon"
#define DD_PARAM_CMD_LPF               "lpf"
#define DD_PARAM_CMD_GFKT              "gfkt"
#define DD_PARAM_CMD_GASIN             "gasin"
#define DD_PARAM_CMD_GOSIN             "gosin"
#define DD_PARAM_CMD_GFSIN             "gfsin"
#define DD_PARAM_CMD_GATRI             "gatri"
#define DD_PARAM_CMD_GOTRI             "gotri"
#define DD_PARAM_CMD_GFTRI             "gftri"
#define DD_PARAM_CMD_GSTRI             "gstri"
#define DD_PARAM_CMD_GAREC             "garec"
#define DD_PARAM_CMD_GOREC             "gorec"
#define DD_PARAM_CMD_GFREC             "gfrec"
#define DD_PARAM_CMD_GSREC             "gsrec"
#define DD_PARAM_CMD_GANOI             "ganoi"
#define DD_PARAM_CMD_GONOI             "gonoi"
#define DD_PARAM_CMD_GASWE             "gaswe"
#define DD_PARAM_CMD_GOSWE             "goswe"
#define DD_PARAM_CMD_GTSWE             "gtswe"
#define DD_PARAM_CMD_SCT               "sct"
#define DD_PARAM_CMD_TRGSS             "trgss"
#define DD_PARAM_CMD_TRGSE             "trgse"
#define DD_PARAM_CMD_TRGSI             "trgsi"
#define DD_PARAM_CMD_TRGLEN            "trglen"
#define DD_PARAM_CMD_TRGEDGE           "trgedge"
#define DD_PARAM_CMD_WRMB              "wrmb"

// Global parameters
#define DD_PARAM_CMD_BRIGHT            "bright"

// Can't read back
#define DD_PARAM_CMD_SSTD              "sstd"
#define DD_PARAM_CMD_SS                "ss"
#define DD_PARAM_CMD_CL                "cl"
#define DD_PARAM_CMD_FBREAK            "fbreak"

// Service mode parameter addresses:
#define DD_PARAM_SVC_ENC_RATE          109

// Global commands (and/or unsupported):
//s   - show all available commands
//dprpon - periodically output position (on)
//dprpof -  (off)
//dprson - output status when updated
//dprsof -  (off)
//  TODO: if polling all the parameters is too intensive, switch these on
//        and fix up the writeRead() functions to intercept these
//DD_PARAM_SETF                  "SETF"                // set the output format of measurement value (mess) 0= three decimal places 1= scientific format
//DD_PARAM_SETG                  "SETG"                // set the output format of all floating point values except “mess” 0= three decimal places 1= scientific format

// already done while polling:
// stat
// mess

static const char* driverName = "ddrive";
class ddriveController;
char *strnchr(const char* str, size_t len, char c);
const char *get_ddrive_error_string(int error);

static const char *DDRIVE_MEAS_STRINGS[] = {
  "None",
  "Strain gauge",
  "Capacitive sensor",
  "Inductive sensor"
};

static const char *DDRIVE_GEN_STRINGS[] = {
  "Off",
  "Sine",
  "Triangle",
  "Rectangle",

  "Noise",
  "Sweep",
  "Unknown0",
  "Unknown1",
};

enum DDMoveMode {
  DD_MODE_POSITION=0, // go to commanded position
  DD_MODE_SCAN,
  DD_MODE_TRIGGER
};

typedef struct {
  const char *asyn_param;
  const char *command;
  asynParamType type;
  int idx;                 // index into the asyn parameter list
  bool global;             // global parameter
  bool query;              // query the status of this parameter?
  bool service_mode;       // service mode required to read/write param?
  int service_param;       // service mode required to read/write param?
} DDParam;

class ddriveAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  ddriveAxis(ddriveController *pC, int axis);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  //asynStatus setPosition(double position);
  asynStatus setUIntDigitalParam(int index, epicsUInt32 value);

  /* And these are specific to this class: */
  asynStatus setServo(bool enabled);
  asynStatus queryPosition();
  asynStatus queryStatus();
  bool checkMoving();

  template <typename T>
  asynStatus queryParameter(const DDParam *param, T& value);
  asynStatus queryParameters();
  asynStatus writeParameter(int idx, int value);
  asynStatus writeParameter(int idx, double value);
  asynStatus writeParameter(const DDParam *param, int value);
  asynStatus writeParameter(const DDParam *param, double value);

  inline bool isFlagSet(unsigned int flag) { return (flags_ & flag) == flag; }
  inline void setFlag(unsigned int flag)   { flags_ |= flag; }
  inline void clearFlag(unsigned int flag) { flags_ &= ~flag; }
  inline void setFlag(unsigned int flag, bool set) {
    if (set)
      flags_ |= flag;
    else
      flags_ &= ~flag;
  }

protected:

private:
  void motionFinished();

  friend class ddriveController;
  ddriveController *pc_;    /**< Pointer to the asynMotorController to which this axis belongs.
                            *   Abbreviated because it is used very frequently */
  double encoder_pos_;       /** < Cached copy of the encoder position */
  unsigned int flags_;      /** < Cached copy of the current flags */

  bool moving_;
  double move_target_;
  bool errored_;
  int axis_num_;             // according to asyn (0-based)
  int status_;
  int param_num_;            // last parameter number (according to ddrive_params) to be queried
  bool initial_query_;       // the first query
  bool query_status_;        // whether or not to query status the next update
  DDMoveMode mode_;
  epicsTime t_move_start_;
  epicsTime t_move_timeout_;
};

class ddriveController : public asynMotorController {
public:
  ddriveController(const char *portName, const char *ddrivePortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  ddriveAxis* getAxis(int axisNo) {
    return (ddriveAxis*)asynMotorController::getAxis(axisNo);
  }

  /** Returns a pointer to an ddriveMotorAxis object.
    * Returns NULL if the axis number encoded in pasynUser is invalid.
    * \param[in] pasynUser asynUser structure that encodes the axis index number. */
  ddriveAxis* getAxis(asynUser *pasynUser)
  {
    return static_cast<ddriveAxis*>(asynMotorController::getAxis(pasynUser));
  }

  asynStatus write(const char* fmt, ...);
  asynStatus write(const char* fmt, va_list);
  asynStatus writeRead(char* buf_input, char *buf_output, size_t* nread);
  asynStatus writeRead(char* input, size_t* nread, const char* fmt, ...);
  asynStatus writeRead(char* input, size_t* nread, const char* fmt, va_list);

  /** Format and write a string based on printf formatting,
   * expecting a response in the same format. That is,
   *  write: mess,0
   *  read:  mess,0,0.01
   * Returns asynSuccess on a successful match, ret_buf in the example would then be "0.01"
   *         asynError on a failed match, and ret_buf contains the full response
   * \param[out] ret_buf buffer of size DD_STRING_LEN
   * \param[in] fmt format for string to write
   * \param[in] argptr varargs
   * */
  asynStatus writeReadMatch(char *ret_buf, const char* fmt, ...);
  asynStatus writeReadMatch(char *ret_buf, const char* fmt, va_list argptr);
  asynStatus writeParameter(const DDParam *param, int value);
  asynStatus writeParameter(const DDParam *param, double value);
  asynStatus writeParameter(int idx, int value);
  asynStatus writeParameter(int idx, double value);

  /** Write and read back the controller's response, what should be a float. e.g.,
   *  write: mess,0
   *  read:  mess,0,0.01
   * Returns asynSuccess on a successful match, value in the example would then be 0.01
   *         asynError on a failed match, and value is unmodified
   * \param[out] value the value read
   * \param[in] fmt format for string to write
   * \param[in] ...
   * */
  asynStatus writeRead(float& value, const char* fmt, ...);
  asynStatus writeRead(double& value, const char* fmt, ...);
  asynStatus writeRead(double& value, const char* fmt, va_list);
  asynStatus writeRead(int &value, const char* fmt, va_list argptr);
  asynStatus writeRead(int& value, const char* fmt, ...);
  bool format(int &len, char *buf, const char *fmt, ...);
  bool format(int &len, char *buf, const char *fmt, va_list argptr);

  void enterServiceMode();
  void exitServiceMode();
  int getAxisCount() { return numAxes_; }
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeUInt32Digital(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);

protected:
  // R+W parameters
  int param_ktemp_;
#define FIRST_DDRIVE_PARAM param_ktemp_
  int param_rohm_;
  int param_rgver_;
  int param_fenable_;
  int param_sr_;
  int param_modon_;
  int param_monsrc_;
  int param_kp_;
  int param_ki_;
  int param_kd_;
  int param_notchon_;
  int param_notchf_;
  int param_notchb_;
  int param_lpon_;
  int param_lpf_;
  int param_gfkt_;
  int param_gasin_;
  int param_gosin_;
  int param_gfsin_;
  int param_gatri_;
  int param_gotri_;
  int param_gftri_;
  int param_gstri_;
  int param_garec_;
  int param_gorec_;
  int param_gfrec_;
  int param_gsrec_;
  int param_ganoi_;
  int param_gonoi_;
  int param_gaswe_;
  int param_goswe_;
  int param_gtswe_;
  int param_sct_;
  int param_trgss_;
  int param_trgse_;
  int param_trgsi_;
  int param_trglen_;
  int param_trgedge_;

  // Global parameters
  int param_bright_;

  // Write-only
  int param_sstd_;
  int param_ss_;
  int param_cl_;
  int param_fbreak_;
  int param_enc_rate_;
#define LAST_DDRIVE_CONTROLLER_PARAM param_enc_rate_
#define NUM_DDRIVE_CONTROLLER_PARAMS (&LAST_DDRIVE_CONTROLLER_PARAM - &FIRST_DDRIVE_PARAM + 1)
  // Read-only
  int param_plugged_;
  int param_closed_sys_;
  int param_voltage_on_;
  int param_meas_sys_;

  // Read-write
  int param_move_timeout_;

#define LAST_DDRIVE_PARAM param_move_timeout_
#define NUM_DDRIVE_PARAMS (&LAST_DDRIVE_PARAM - &FIRST_DDRIVE_PARAM + 1)
  DDParam *ddparams_;
  double timeout_;

private:
  friend class ddriveAxis;
  void createDDParam(const char *cmd, const char *param_str, asynParamType type, int *param_idx, 
                     bool query=true, bool global=false, bool service_mode=false, int service_param=0);
  const DDParam *DDParamFromIndex(int param_index);

  bool service_mode_;
  asynUser *pasynUser_;
  int queryRate_;           // Query N parameters every poll
  int dd_param_count_;
  double position_move_timeout_;

};

/* Use the following structure and functions to manage multiple instances
 * of the driver */
typedef struct ddriveNode {
    ELLNODE node;
    const char *portName;
    ddriveController *pController;
} ddriveNode;

bool addToList(const char *portName, ddriveController *drv);
ddriveController* findByPortName(const char *portName);


#endif

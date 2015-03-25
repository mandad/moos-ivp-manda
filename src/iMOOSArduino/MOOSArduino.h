/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: MOOSArduino.h                                   */
/*    DATE: January 2014                                    */
/************************************************************/

#ifndef MOOSArduino_H
#define MOOSArduino_H

#include <stdarg.h>

#include "MOOS/libMOOS/MOOSLib.h"
#include "../uMOOSArduinoLib/IMOOSComm.h"
#include "../uMOOSArduinoLib/CommType.h"

class MOOSArduino : public CMOOSApp
{
public:
  MOOSArduino();
  ~MOOSArduino();

protected:
  // CMOOSApp Overloaded Methods
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  void RegisterVariables();

  virtual bool getVar(const char *buf, const char *name, double &value);
  virtual bool getVar(const char *buf, const char *name, std::string &value);
  virtual bool getVars(const char *buf, const char *name, 
    std::list<std::string> &value);

private: // Configuration variables
  std::string serialPortName;
  int baudRate;
  const int defaultBaudRate;
  const CommType defaultCommType;
  bool debugOutput;

private: // State variables
  IMOOSComm *f_comms;
  CommType commType;

  unsigned int m_iterations;
  double       m_timewarp;

  const char *delim; // delimiter for key-value pairs
  const char *WRITE_FIFO_NAME;
  const char *READ_FIFO_NAME;
  
//  double dfTimeNow;
  double desiredSpeed;      // specified in meters/sec
  double desiredHeading;    // specified in radians
  double desiredThrust;     // specified as a signed percentage
  double lastDesiredThrust;
  double desiredRudder;     // specified as a signed degree
  double lastDesiredRudder;

  double nav_x;
  double nav_y;
  double nav_heading;
  double nav_speed;

  std::string getPolyString(const double x, const double y);
  void dprintf(const char *format, ...);
};

#endif /* MOOSArduino_H */

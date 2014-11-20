/************************************************************/
/*    NAME: Mike Bogochow                                   */
/*    ORGN: UNH                                             */
/*    FILE: MOOSArduinoTester.h                             */
/*    DATE: Feb, 2014                                       */
/************************************************************/

#ifndef MOOSArduinoTester_HEADER
#define MOOSArduinoTester_HEADER

#include "MOOS/libMOOS/MOOSLib.h"
#include "../uMOOSArduinoLib/IMOOSComm.h"

class MOOSArduinoTester : public CMOOSApp
{
public:
  MOOSArduinoTester();
  ~MOOSArduinoTester();

protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  void RegisterVariables();

  virtual bool postVar(const char *name, double value);
  virtual bool postVar(const char *buf, const char *name, double &value);

private: // Configuration variables

private: // State variables
  IMOOSComm *f_comms;
  unsigned int m_iterations;
  double       m_timewarp;

  const char *READ_FIFO_NAME;// = "MOOSArduinoFIFO";
  const char *WRITE_FIFO_NAME;// = "MOOSArduinoTesterFIFO";

  int wr_fifo;
  int rd_fifo;

  double nav_x;
  double nav_y;
  double nav_heading;
  double nav_speed;

  const char *delim;
};

#endif 

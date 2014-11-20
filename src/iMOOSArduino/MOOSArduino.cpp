/************************************************************/
/*    NAME: Mike Bogochow, Jeff Masucci, Cody Noel          */
/*    ORGN: UNH                                             */
/*    FILE: MOOSArduino.cpp                                 */
/*    DATE: January 2014                                    */
/************************************************************/
/* This application is designed to be an interface between  */
/* MOOS-IvP (the backseat driver) and an Arduino            */
/* microcontroller (acting as the frontseat).  It is part   */
/* the Autonomous Surface Vehicle (ASV) UNH cross           */
/* discipline senior project.                               */
/************************************************************/

#include "../uMOOSArduinoLib/MOOSVars.h"
#include "../uMOOSArduinoLib/CommType.h"
#include "../uMOOSArduinoLib/FIFOComm.h"
#include "../uMOOSArduinoLib/ArduinoComm.h"
//#include "../uMOOSArduinoLib/NetClientComm.h"
#include "MBUtils.h"
#include "MOOSArduino.h"
#include "UtilMOOSArduino.h"

#include <stdarg.h>
#include <string.h>
#include <sstream>
#include <poll.h>

using namespace std;

//---------------------------------------------------------
// Constructor
MOOSArduino::MOOSArduino()
  : defaultBaudRate(115200)
  , delim(",")
  , defaultCommType(ARDUINO)
  , WRITE_FIFO_NAME("MOOSArduinoFIFO")
  , READ_FIFO_NAME("MOOSArduinoTesterFIFO")
{
  m_iterations = 0;
  m_timewarp   = 1;

  desiredSpeed = 0;
  desiredHeading = 0;
  desiredThrust = 0;
  desiredRudder = 0;

  baudRate = defaultBaudRate;
} /* MOOSArduino */

//---------------------------------------------------------
// Destructor
MOOSArduino::~MOOSArduino()
{
  delete f_comms;
} /* !MOOSArduino */

//---------------------------------------------------------
// Procedure: OnNewMail
bool MOOSArduino::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSTrace("OnNewMail()\n");

  CMOOSMsg Msg;
  double dfNow = MOOSTime();

  if (m_Comms.PeekMail(NewMail, DESIRED_SPEED, Msg, false, true))
  {
    desiredSpeed = Msg.GetDouble();
    dprintf("Got DESIRED_SPEED mail: %f\n", desiredSpeed);    
  }

  if (m_Comms.PeekMail(NewMail, DESIRED_HEADING, Msg, false, true))
  {
    desiredHeading = Msg.GetDouble();
    dprintf("Got DESIRED_HEADING mail: %f\n", desiredHeading);
  }

  // PUBLISHED By: iRemote    <<<<< Look this up
  if (m_Comms.PeekMail(NewMail, DESIRED_RUDDER, Msg, false, true))
  {
    desiredRudder = (int)Msg.GetDouble();
    dprintf("Got DESIRED_RUDDER mail: %i\n", desiredRudder);
  }
  
  if (m_Comms.PeekMail(NewMail, DESIRED_THRUST, Msg, false, true))
  {
    desiredThrust = (int)Msg.GetDouble();
    dprintf("Got DESIRED_THRUST mail: %i\n", desiredThrust);
  }

  return true;
} /* OnNewMail */

//---------------------------------------------------------
// Procedure: OnConnectToServer
bool MOOSArduino::OnConnectToServer()
{
  dprintf("OnConnectToServer()\n");
  RegisterVariables();
  return true;
} /* OnConnectToServer */

/**
 * Extract the given variable from the buffer.
 *
 * @return true  if the value is found in the buffer,
 *         false if the variable is not found in the buffer
 */
bool MOOSArduino::getVar(const char *buf, const char *name, double &value)
{
  std::list<string> valBuf;

  if (ValsFromString(valBuf, buf, name, true, delim))
  {
    value = MOOS::StringToDouble(valBuf.front()); // use most recent value
    return true;
  }
  return false;
} /* getVar */

/**
 * Extract the given variable from the buffer.
 *
 * @return true  if the value is foudn in the buffer,
 *         false if the variable is not found in the buffer
 */
bool MOOSArduino::getVar(const char *buf, const char *name, std::string &value)
{
  std::list<string> valBuf;

  if (ValsFromString(valBuf, buf, name, true, delim))
  {
    value = valBuf.front(); // use most recent value
    return true;
  }
  return false;
} /* getVar */

/**
 * Extract the given variable from the buffer.
 *
 * @return true  if the value is foudn in the buffer,
 *         false if the variable is not found in the buffer
 */
bool MOOSArduino::getVars(const char *buf, const char *name, std::list<std::string> &value)
{
  std::list<string> valBuf;

  if (ValsFromString(valBuf, buf, name, true, delim))
  {
    value = valBuf; // use most recent value
    return true;
  }
  return false;
} /* getVars */

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second
bool MOOSArduino::Iterate()
{
  m_iterations++;
  dprintf("Iteration #%i\n", m_iterations);

  int nBytes;
  const int BUF_SIZE = 4096;
  char *buf = new char[BUF_SIZE];

  dprintf("Attempting to read %i bytes from frontseat\n", BUF_SIZE);
  nBytes = f_comms->readMsg(buf, BUF_SIZE);
  if (nBytes > 0)
  { // Something was read
    dprintf("Got %i bytes from frontseat\n", nBytes);
    buf[nBytes] = '\0';
    dprintf("%s\n", buf);

    // Process what was read
    if (getVar(buf, NAV_X, nav_x))
      Notify(NAV_X, nav_x);

    if (getVar(buf, NAV_Y, nav_y))
      Notify(NAV_Y, nav_y);

    if (getVar(buf, NAV_HEADING, nav_heading))
      Notify(NAV_HEADING, nav_heading);

    if (getVar(buf, NAV_SPEED, nav_speed))
      Notify(NAV_SPEED, nav_speed);

    std::list<std::string> dbgBuf;
    if (getVars(buf, DEBUG_MSG, dbgBuf))
    {
      ostringstream ss;
      ss << "DEBUG: ";
      for (std::list<std::string>::iterator it = dbgBuf.begin();
        it != dbgBuf.end(); it++)
      {
        std::string val = *it;
        ss << val << "  |  ";
      }
      ss << std::endl;
      MOOSTrace(ss.str());
    }
  }

  else if (nBytes < 0)
    MOOSTrace("Error reading from fifo: %s\n", strerror(errno));

  dprintf("Done reading from frontseat\n");

  // See if we got obstacle information
  std::list<string> valBuf;
  if (ValsFromString(valBuf, buf, OBSTACLE, true, delim))
  { // Received obstacle information
    for (std::list<std::string>::iterator it = valBuf.begin();
      it != valBuf.end(); it++)
    {
      std::string obstacle = *it;

      // Convert received message to MOOS polygon to represent obstacle
      // Messages should be in the form OBSTACLE_DATA=x:y,
      char *cstr = (char *)obstacle.c_str();
      int len = obstacle.length();
      ostringstream xstr;
      ostringstream ystr;
      const char coordDelim = ':';
      int i;
      for (i = 0; i < len; i++)
      {
        if (cstr[i] == coordDelim)
          break;
        xstr << cstr[i];
      }
      for (i++; i < len; i++)
      {
        ystr << cstr[i];
      }
      
      double x = MOOS::StringToDouble(xstr.str());
      double y = MOOS::StringToDouble(ystr.str());


      char *polyString = (char *)getPolyString(x, y).c_str();

      dprintf("%s\n", polyString);

      // Post the polygon to the MOOSDB
      Notify(OBSTACLE_UPDATE, polyString);
    }
  }

  delete buf;

  // Send heading and speed to PID controller
  // ? - need to know how this will be handled (i.e. do we have to write one?
  // is it built into something else? can we use pMarinePID?)

  // Send variables to frontseat
  // Maybe only send if they were updated in last call to OnNewMail() ?
  if (!f_comms->writeMsg(DESIRED_SPEED, desiredSpeed))
    MOOSTrace("Could not write %s to frontseat\n", DESIRED_SPEED);

  if (!f_comms->writeMsg(DESIRED_HEADING, desiredHeading))
    MOOSTrace("Could not write %s to frontseat\n", DESIRED_HEADING);

  return true;
} /* Iterate */

/**
 * Get a string representing a 1x1 square polygon from the given coordinates.  
 * The string is in the format expected by the Avoid_Obstacles behavior.
 * 
 * @param x
 *        the x value of the coordinates
 * @param y
 *        the y value of the coordinates
 * @return the string representation of the polygon created by the coordinates
 */
std::string MOOSArduino::getPolyString(const double x, const double y)
{
  ostringstream polygon;
  double nextX = x + 1;
  double nextY = y + 1;
  
  polygon << "polygon=";
  polygon << x     << ',' << y     << ':'
          << nextX << ',' << y     << ':'
          << nextX << ',' << nextY << ':'
          << x     << ',' << nextY;

  return polygon.str();
} /* getPolyString */

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open
// Read from configuration files and register for MOOS variables.
bool MOOSArduino::OnStartUp()
{
  dprintf("OnStartUp()\n");

  // Read the DebugOutput configuration field
  if (!m_MissionReader.GetConfigurationParam("DebugOutput", debugOutput))
    debugOutput = false;
  
  // Read the CommType configuration field
  std::string comms;
  if (!m_MissionReader.GetConfigurationParam("CommType", comms))
  {
    const char *cCommType = getStringFromCommType(defaultCommType).c_str();
    MOOSTrace("Warning: parameter 'CommType' not specified.  Using default comm"
      " type (%s)\n", cCommType);
    commType = defaultCommType;
  }
  
  else
  {
    commType = getCommType(comms);
    if (commType == UNKNOWN)
    {
      const char *cCommType = getStringFromCommType(defaultCommType).c_str();
      MOOSTrace("Warning: invalid value passed to parameter 'CommType' (%s).  "
        "Using default comm type (%s)\n", comms.c_str(), cCommType);
      commType = defaultCommType;
    }
  }
  
  // Initialize depending on the comms type and read configuration fields 
  // appropriate for the comms type
  switch(commType)
  {
  case FIFO:
    f_comms = new FIFOComm(READ_FIFO_NAME, WRITE_FIFO_NAME, delim);
    break;
  case ARDUINO:
    f_comms = new ArduinoComm(serialPortName.c_str(), baudRate, delim);
    MOOSTrace("Com port is: %s \n", serialPortName.c_str());

    if (!m_MissionReader.GetConfigurationParam("SerialPort", serialPortName))
    {
      MOOSTrace("Warning: parameter 'SerialPort' not specified.\n");
      MOOSTrace("Terminating\n");
      exit(-1);
    }

    if (!m_MissionReader.GetConfigurationParam("BaudRate", baudRate))
    {
      MOOSTrace("Warning: parameter 'BaudRate' not specified.  Using "
        "default baud rate (%i)\n", defaultBaudRate);
    }
    break;
  } /* no default since should have valid commType if default set correctly */

  if (!f_comms->openComm())
  {
    MOOSTrace("Could not open communications.  Terminating...");
    exit(-1);
  }
  
  else 
    MOOSTrace("Successfully opened communications\n");

  RegisterVariables();

  return true;
} /* OnStartUp */

//---------------------------------------------------------
// Procedure: RegisterVariables
void MOOSArduino::RegisterVariables()
{
  // m_Comms.Register("IVPHELM_ENGAGED", 0);
  // m_Comms.Register("VEHICLE_UNDERWAY", 0);

  m_Comms.Register(DESIRED_SPEED, 0);
  m_Comms.Register(DESIRED_HEADING, 0);

  m_Comms.Register(DESIRED_RUDDER, 0);
  m_Comms.Register(DESIRED_THRUST, 0);

  // m_Comms.Register("ZERO_RUDDER", 0);
  // m_Comms.Register("ZERO_ELEVATOR", 0);

  // m_Comms.Register("DESIRED_FRONTSEAT_WAYPOINT", 0);
} /* RegisterVariables */

void MOOSArduino::dprintf(const char *format, ...)
{
  if (debugOutput)
  {
    const unsigned int MAX_TRACE_STR = 2048;
    char buf[MAX_TRACE_STR * 2];

    va_list args;
    va_start(args, format);

    vsnprintf(buf, sizeof(buf), format, args);

    va_end(args);

    MOOSTrace(buf);
  }
} /* dprintf */

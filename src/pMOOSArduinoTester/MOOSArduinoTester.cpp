/************************************************************/
/*    NAME: Mike Bogochow                                   */
/*    ORGN: UNH                                             */
/*    FILE: MOOSArduinoTester.cpp                           */
/*    DATE: January 2014                                    */
/************************************************************/
/* This application is designed to simulate the Arduino     */
/* component of the ASV in order to test iMOOSArduino.      */
/* It will se expected output of the Arduino to a FIFO      */
/* which will take the place of the Arduino's serial        */
/* connection.  It will also read in the output of          */
/* iMOOSArduino and check its validity.                     */
/************************************************************/


#include "MBUtils.h"
#include "MOOSArduinoTester.h"
#include "../uMOOSArduinoLib/MOOSVars.h"
#include "../iMOOSArduino/UtilMOOSArduino.h"
#include "../uMOOSArduinoLib/FIFOComm.h"

#include <iterator>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

using namespace std;

//---------------------------------------------------------
// Constructor

MOOSArduinoTester::MOOSArduinoTester()
  : READ_FIFO_NAME("MOOSArduinoFIFO")
  , WRITE_FIFO_NAME("MOOSArduinoTesterFIFO")
  , delim(",")
{
  m_iterations = 0;
  m_timewarp   = 1;

  nav_x = 0;
  nav_y = 0;
  nav_heading = 90;
  nav_speed = 2.0;

  f_comms = new FIFOComm(READ_FIFO_NAME, WRITE_FIFO_NAME, delim);
}

//---------------------------------------------------------
// Destructor

MOOSArduinoTester::~MOOSArduinoTester()
{
  delete f_comms;
}

//---------------------------------------------------------
// Procedure: OnNewMail
bool MOOSArduinoTester::OnNewMail(MOOSMSG_LIST &NewMail)
{
  MOOSTrace("OnNewMail()\n");

  MOOSMSG_LIST::iterator p;
  for(p = NewMail.begin(); p != NewMail.end(); p++)
    CMOOSMsg &msg = *p;

  return true;
}

//---------------------------------------------------------
// Procedure: OnConnectToServer
bool MOOSArduinoTester::OnConnectToServer()
{
  MOOSTrace("OnConnectToServer()\n");
  return true;
}

/**
 * Post the given variable to the MOOSDB.
 *
 * @return true  if the value is successfully posted to the MOOSDB,
 *         false if the call to Notify fails
 */
bool MOOSArduinoTester::postVar(const char *name, double value)
{
  MOOSTrace("%s=%f\n", name, value);
  return true;
} /* postVar */

/**
 * Extract the given variable from the buffer and post it to the MOOSDB.
 *
 * @return true  if the value is successfully posted to the MOOSDB,
 *         false if the post fails or the variable is not found in the buffer
 */
bool MOOSArduinoTester::postVar(const char *buf, const char *name, double &value)
{
  std::list<string> valBuf;

  if (ValsFromString(valBuf, buf, name, true, delim))
  {
    value = MOOS::StringToDouble(valBuf.front());
    return postVar(name, value);
  }

  return false;
} /* postVar */

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second
bool MOOSArduinoTester::Iterate()
{
  m_iterations++;
  MOOSTrace("Iteration #%i\n", m_iterations);

  const int BUF_SIZE = 4096;
  char *buf = new char[BUF_SIZE];

  MOOSTrace("Attempting to read %i bytes from read fifo\n", BUF_SIZE);

  int nBytes = f_comms->readMsg(buf, BUF_SIZE);
  MOOSTrace("Attempted to read %i bytes, got %i bytes\n", BUF_SIZE, nBytes);
  if (nBytes > 0)
  {
    // We read something from the FIFO
    MOOSTrace("Got %i bytes from fifo\n", nBytes);

    buf[nBytes] = '\0';

    // Print what we read
    double temp;
    postVar(buf, DESIRED_SPEED, temp);
    postVar(buf, DESIRED_HEADING, temp);
  }

  else if (nBytes < 0)
    MOOSTrace("Error reading from fifo: %s\n", strerror(errno));

  MOOSTrace("Done reading from buffer\n");
  delete buf;

  // Write to the fifo
  // Alter the variables to show change for testing
  if (!f_comms->writeMsg(NAV_X, nav_x++))
    MOOSTrace("Could not write %s to iMOOSArduino\n", NAV_X);

  if (!f_comms->writeMsg(NAV_Y, nav_y++))
    MOOSTrace("Could not write %s to iMOOSArduino\n", NAV_Y);

  if (!f_comms->writeMsg(NAV_HEADING, (nav_heading + 1) > 360
      ? nav_heading = 0 : ++nav_heading))
  {
    MOOSTrace("Could not write %s to iMOOSArduino\n", NAV_HEADING);
  }
  
  if (!f_comms->writeMsg(OBSTACLE, "10:-22"))
  {
    MOOSTrace("Could not write obstacle data to iMOOSArduino\n");
  }

  if (!f_comms->writeMsg(NAV_SPEED, nav_speed + 1 > 4 ? nav_speed = 2 : 
    ++nav_speed))
  {
    MOOSTrace("Could not write %s to iMOOSArduino\n", NAV_SPEED);
  }

  return true;
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool MOOSArduinoTester::OnStartUp()
{
  MOOSTrace("OnStartUp()\n");

  if (!f_comms->openComm())
  {
    MOOSTrace("Error opening FIFO(s): %s\n", strerror(errno));
    exit(-1);
  }

  MOOSTrace("OnStartUp: fifos opened\n");

  return true;
}


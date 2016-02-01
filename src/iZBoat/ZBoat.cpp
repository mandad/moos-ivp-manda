/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: ZBoat.cpp                                       */
/*    DATE: 19 Aug 2015                                     */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "ZBoat.h"

using namespace std;

//---------------------------------------------------------
// Constructor and Destructor

ZBoat::ZBoat()
{
   m_dfMaxRudder = 45;
   m_dfMaxThrottle = 100;
   m_cPwmMessage[0] = '\0';
}

ZBoat::~ZBoat()
{
  //Set the thrusters to stop
  const char * sStopThrust = "!pwm, *, 0.000, 0.000, 0.000, *, *\r\n";
  m_Port.Write(sStopThrust, strlen(sStopThrust));

  //Return menual control.
  const char * sInit = "!SetManualControl\r\n";
  MOOSTrace("ZBoat: Sending %s\n", sInit);
  m_Port.Write(sInit, strlen(sInit));
}


//---------------------------------------------------------
// Procedure: OnNewMail

bool ZBoat::OnNewMail(MOOSMSG_LIST &NewMail)
{
  // AppCastingMOOSApp::OnNewMail(NewMail);

//   MOOSMSG_LIST::iterator p;
//   for(p=NewMail.begin(); p!=NewMail.end(); p++) {
//     CMOOSMsg &msg = *p;
//     string key    = msg.GetKey();

// #if 0 // Keep these around just for template
//     string comm  = msg.GetCommunity();
//     double dval  = msg.GetDouble();
//     string sval  = msg.GetString();
//     string msrc  = msg.GetSource();
//     double mtime = msg.GetTime();
//     bool   mdbl  = msg.IsDouble();
//     bool   mstr  = msg.IsString();
// #endif

//      if(key == "FOO")
//        cout << "great!";

//      else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
//        reportRunWarning("Unhandled Mail: " + key);
//    }

   UpdateMOOSVariables(NewMail);

   GeneratePWMMessage();

   PublishData();

   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool ZBoat::OnConnectToServer()
{
   if (IsSimulateMode()) {
    //not much to do...
    RegisterMOOSVariables();
  } else {

  }
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool ZBoat::Iterate()
{
  // AppCastingMOOSApp::Iterate();

  // if (GetData()) {
  //   PublishData();
  // }

  // AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool ZBoat::OnStartUp()
{
  // AppCastingMOOSApp::OnStartUp();
  CMOOSInstrument::OnStartUp();

  MOOSTrace("ZBoat: StartUp");

  // 20 Hz max input rate
  double dfInputPeriod = 0.05;

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    MOOSTrace("No config block found for " + GetAppName());
    // reportConfigWarning("No config block found for " + GetAppName());
  }

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = toupper(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "MAXINPUTRATE") {
      dfInputPeriod = 1 / atof(value.c_str());
      handled = true;
    }
    else if (param == "MAXRUDDER") {
      m_dfMaxRudder = atof(value.c_str());
    }
    else if (param == "MAXTHROTTLE") {
      m_dfMaxThrottle = atof(value.c_str());
    }
    else if(param == "PORT" || param == "BAUDRATE" || param == "HANDSHAKING" ||
      param == "STREAMING") {
      // These are all handled by CMOOSInstrument
      handled = true;
    }

    if(!handled) {
      MOOSTrace("Unhandled config line: " + orig);
      //reportUnhandledConfigWarning(orig);
    }

  }

  //here we make the variables that we are managing

  //GPS update @ 2Hz
  AddMOOSVariable("Throttle", "DESIRED_THRUST", "", dfInputPeriod);
  AddMOOSVariable("Rudder", "DESIRED_RUDDER", "", dfInputPeriod);
  AddMOOSVariable("PWM", "", "ZBOAT_PWM", dfInputPeriod);
  AddMOOSVariable("SetAutonomy", "SET_AUTONOMY_MODE", "", dfInputPeriod);

  registerVariables();
  //try to open

  if (!SetupPort()) {
    return false;
  }

  //try 10 times to initialise sensor
  if (!InitialiseSensorN(10, "MOOS to ZBoat")) {
    return false;
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void ZBoat::registerVariables()
{
  //AppCastingMOOSApp::RegisterVariables();
  RegisterMOOSVariables();
  // Register("FOOBAR", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

// bool ZBoat::buildReport()
// {
//   m_msgs << "============================================ \n";
//   m_msgs << "File:                                        \n";
//   m_msgs << "============================================ \n";

//   ACTable actab(4);
//   actab << "Alpha | Bravo | Charlie | Delta";
//   actab.addHeaderLines();
//   actab << "one" << "two" << "three" << "four";
//   m_msgs << actab.getFormattedString();

//   return(true);
// }

//-----------------------------------------------------------
// MOOSInstrument Functions

bool ZBoat::InitialiseSensor()
{
  const char * sInit = "!SetAutonomousControl\r\n";
  MOOSTrace("ZBoat: Sending %s\n", sInit);
  m_Port.Write(sInit, strlen(sInit));

  return true;

}

//===========================================================
// Custom functions for this instrument

bool ZBoat::GetData()
{
  //here we actually access serial ports etc
  //For this app we are not expecting to receive anything

  /*
  string sWhat;

  double dfWhen;

  if (m_Port.IsStreaming()) {
    if (!m_Port.GetLatest(sWhat, dfWhen)) {
      return false;
    }
  } else {
    if (!m_Port.GetTelegram(sWhat, 0.5)) {
      return false;
    }
  }

  //MOOSTrace("Rx:  %s",sWhat.c_str());
  if (PublishRaw()) {
    SetMOOSVar("Raw", sWhat, MOOSTime());
  }
  */
  //Process the data here (sWhat)

  return true;

}

bool ZBoat::PublishData()
{
  // char cPwmMessage[40];
  //  GetMOOSVar("PWM")->();
  if (strlen(m_cPwmMessage) > 0) {
    MOOSTrace("ZBoat Tx: %s", m_cPwmMessage);
    m_Port.Write(m_cPwmMessage, strlen(m_cPwmMessage));
  }

  //Check if we need to set autonomy mode again
  CMOOSVariable * pSetAutonomy = GetMOOSVar("SetAutonomy");
  double dfSetAutonomy = -1;
  if (pSetAutonomy->IsFresh()){
    dfSetAutonomy = pSetAutonomy->GetDoubleVal();
  }
  if (dfSetAutonomy == 1) {
    InitialiseSensor();
  }
  return PublishFreshMOOSVariables();
}

void ZBoat::GeneratePWMMessage()
{
  CMOOSVariable * pThrottleSet = GetMOOSVar("Throttle");
  CMOOSVariable * pRudderSet = GetMOOSVar("Rudder");
  double dfThrottleSet = 0;
  double dfRudderSet = 0;

  if (pThrottleSet->IsFresh()) {
    dfThrottleSet = pThrottleSet->GetDoubleVal();

  }
  if (pRudderSet->IsFresh()) {
    dfRudderSet = pRudderSet->GetDoubleVal();
  }

  MOOSAbsLimit(dfRudderSet, m_dfMaxRudder);
  MOOSAbsLimit(dfThrottleSet, m_dfMaxThrottle);

  double dfScaledThrottle = 1.5 - (dfThrottleSet / m_dfMaxThrottle) * 0.5;
  double dfScaledRudder = 1.5 + (dfRudderSet / m_dfMaxRudder) * 0.3;

  char cPwmMessage[40];
  sprintf(cPwmMessage, "!pwm, *, %4.3f, %4.3f, %4.3f, *, *\r\n", dfScaledThrottle,
    dfScaledThrottle, dfScaledRudder);
  strncpy(m_cPwmMessage, cPwmMessage, sizeof(cPwmMessage));

  string sPwmMessage(cPwmMessage);
  SetMOOSVar("PWM", sPwmMessage, MOOSTime());

}

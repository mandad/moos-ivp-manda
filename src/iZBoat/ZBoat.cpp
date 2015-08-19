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
}

ZBoat::~ZBoat()
{
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

  if (GetData()) {
    PublishData();
  }

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
  const char * sInit = "!SetAutonomousControl\r";
  MOOSTrace("Sending %s\n", sInit);
  m_Port.Write(sInit, strlen(sInit));

  // if (MOOSStrCmp(m_sType, "ASHTECH")) {
  //   const char * sInit = "$PASHS,NME,GGA,A,ON\r\n";
  //   MOOSTrace("Sending %s\n", sInit);
  //   m_Port.Write(sInit, strlen(sInit));

  //   MOOSPause(2000);
  //   string sReply;
  //   double dfTime;

  //   if (m_Port.GetLatest(sReply, dfTime)) {
  //     MOOSTrace("Rx %s", sReply.c_str());
  //   } else {
  //     MOOSTrace("No reply\n");
  //   }

  // } 

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
  return PublishFreshMOOSVariables();
}

void GeneratePWMMessage()
{

}



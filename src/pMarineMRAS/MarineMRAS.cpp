/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: UNH                                              */
/*    FILE: MarineMRAS.cpp                                        */
/*    DATE: 2015-12-06                                                */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "MarineMRAS.h"
#include "AngleUtils.h"

using namespace std;

//---------------------------------------------------------
// Constructor

MarineMRAS::MarineMRAS()
{
    m_k_star = 1;
    m_tau_star = 1;
    m_z = 1;
    m_beta = 1;
    m_alpha = 1;
    m_gamma = 1;
    m_xi = 1;
    m_rudder_limit = 45;
    m_max_ROT = 60; // deg/s
    m_cruising_speed = 2;
    m_length = 2;

    m_first_heading = true;
    m_current_ROT = 0;
    m_has_control = false;
    //m_last_heading_time = MOOSTime();
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool MarineMRAS::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

    if(key == "NAV_HEADING") {
      double curr_time = MOOSTime();
      double cur_head = msg.GetDouble();
      m_current_heading = angle180(cur_head);
      if (m_first_heading) {
        m_first_heading = false;
      } else {
        double diff = m_previous_heading - m_current_heading;
        diff = angle180(diff);
        m_current_ROT = diff / (curr_time - m_last_heading_time);
      }

      m_previous_heading = m_current_heading;
      m_last_heading_time = curr_time;
    }
    else if (key == "NAV_SPEED")
      m_current_speed = msg.GetDouble();
    else if (key == "DESIRED_HEADING")
      m_desired_heading = msg.GetDouble();
    else if((key == "MOOS_MANUAL_OVERIDE") || (key == "MOOS_MANUAL_OVERRIDE")) {
      if(MOOSStrCmp(msg.GetString(), "FALSE")) {
        m_has_control = true;
      } else if(MOOSStrCmp(msg.GetString(), "TRUE")) {
          m_has_control = false;
      }
    }

     else if(key != "APPCAST_REQ") // handle by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool MarineMRAS::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool MarineMRAS::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if (m_has_control) {
    double desired_rudder = 0;
    if (!m_first_heading) {
      desired_rudder = m_CourseControl.Run(m_desired_heading, m_current_heading, 
        m_current_ROT, m_current_speed, m_last_heading_time);
    }
    Notify("DESIRED_RUDDER", desired_rudder);
    Notify("DESIRED_THRUST", 50.0);
  } else {
    Notify("DESIRED_RUDDER", 0.0);
    Notify("DESIRED_THRUST", 0.0);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool MarineMRAS::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = toupper(biteStringX(line, '='));
    string value = line;
    double dval  = atof(value.c_str());

    bool handled = false;
    if(param == "K_STAR") {
      m_k_star = dval;
      handled = true;
    }
    else if(param == "TAU_STAR") {
      m_tau_star = dval;
      handled = true;
    }
    else if(param == "DAMPINGRATIO") {
      m_z = dval;
      handled = true;
    }
    else if(param == "BETA") {
      m_beta = dval;
      handled = true;
    }
    else if(param == "ALPHA") {
      m_alpha = dval;
      handled = true;
    }
    else if(param == "GAMMA") {
      m_gamma = dval;
      handled = true;
    }
    else if(param == "XI") {
      m_xi = dval;
      handled = true;
    }
    else if(param == "RUDDERLIMIT") {
      m_rudder_limit = dval;
      handled = true;
    }
    else if(param == "CRUISINGSPEED") {
      m_cruising_speed = dval;
      handled = true;
    }
    else if(param == "LENGTH") {
      m_length = dval;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  //Initialize the Control system
  m_CourseControl.SetParameters(m_k_star, m_tau_star, m_z, m_beta, 
        m_alpha, m_gamma, m_xi, m_rudder_limit, m_cruising_speed, m_length, 
        m_max_ROT);

  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void MarineMRAS::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("DESIRED_HEADING", 0);
  Register("MOOS_MANUAL_OVERIDE", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool MarineMRAS::buildReport() 
{
  m_msgs << "============================================ \n";
  m_msgs << "File: pMarineMRAS                            \n";
  m_msgs << "============================================ \n";

  if (m_has_control) {
    ACTable actab(6);
    actab << "Kp | Kd | Ki | Model Heading | Measured | Desired ";
    actab.addHeaderLines();
    actab << m_CourseControl.GetStatusInfo();
    m_msgs << actab.getFormattedString();
  } else {
    m_msgs << "Control not running.";
  }

  return(true);
}





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
    m_ROT_filter_len = 4;
    m_cruising_speed = 2;
    m_length = 2;
    m_decrease_adapt = true;

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
      double curr_time = msg.GetTime();
      double cur_head = msg.GetDouble();
      m_current_heading = angle180(cur_head);
      
      UpdateROT(curr_time);

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

    //Debug variables for logging
    double vars[11];
    m_CourseControl.GetDebugVariables(vars);
    Notify("MRAS_KP", vars[0]);
    Notify("MRAS_KD", vars[1]);
    Notify("MRAS_KI", vars[2]);
    Notify("MRAS_RUDDER_OUT", vars[3]);
    Notify("MRAS_MODEL_HEADING", vars[4]);
    Notify("MRAS_MODEL_ROT", vars[5]);
    Notify("MRAS_SERIES_MODEL_HEADING", vars[6]);
    Notify("MRAS_SERIES_MODEL_ROT", vars[7]);
    Notify("MRAS_PSI_REF_P", vars[8]);
    Notify("MRAS_PSI_REF_PP", vars[9]);
    Notify("MRAS_SERIES_F", vars[10]);
    Notify("NAV_ROT", m_current_ROT);
    Notify("NAV_HEADING_180", m_current_heading);
    Notify("DESIRED_HEADING_180", angle180(m_desired_heading));
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
    else if(param == "MAXROT") {
      m_max_ROT = dval;
      handled = true;
    }
    else if (param == "ROTFILTER") {
      m_ROT_filter_len = (int) dval;
      handled= true;
    }
    else if (param == "DECREASEADAPTATION") {
      if (toupper(value) == "FALSE")
        m_decrease_adapt = false;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  //Initialize the Control system
  m_CourseControl.SetParameters(m_k_star, m_tau_star, m_z, m_beta, 
        m_alpha, m_gamma, m_xi, m_rudder_limit, m_cruising_speed, m_length, 
        m_max_ROT, m_decrease_adapt);

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
    actab << " | | | | | ";
    actab << "Psi_r'' | Psi_r' | Rudder | Series Heading | Model ROT | Series ROT";
    actab.addHeaderLines();
    actab << m_CourseControl.GetDebugInfo();
    m_msgs << actab.getFormattedString();
  } else {
    m_msgs << "Control not running.";
  }

  return(true);
}

void MarineMRAS::UpdateROT(double curr_time) {
  if (m_first_heading) {
    m_first_heading = false;
  } else {
    // double diff = angle180(m_current_heading - m_previous_heading);
    // double curr_ROT = diff / (curr_time - m_last_heading_time);
    // //this is an arbitary threshold to eliminate noise from sim
    // if (fabs(curr_ROT - m_current_ROT) < 5) {
    //   m_current_ROT = curr_ROT;
    // }

    // //Try limiting it for sim
    // m_current_ROT = CourseChangeMRAS::TwoSidedLimit(m_current_ROT, 40);

    //figure out differential (adapted from IvP PID)
    double diff = angle180(m_current_heading - m_previous_heading);
    double curr_ROT = diff / (curr_time - m_last_heading_time);
    //MOOSTrace("Curr ROT: %f\n", curr_ROT);
    
    //Find the mean and stdev
    if (m_DiffHistory.size() >= 2) {
      double sum = 0;
      double sq_sum = 0;
      list<double>::iterator p;
      for(p = m_DiffHistory.begin();p!=m_DiffHistory.end();p++) {
        sum   += *p;
        sq_sum += *p * (*p);
        //MOOSTrace("List element: %f\n", *p);
      }
      double mean = sum / m_DiffHistory.size();
      double ROT_stdev = sqrt(sq_sum / m_DiffHistory.size() - mean * mean);

      //MOOSTrace("Calculated Mean %f\n", mean);
      //Added the +5 to account
      if (fabs(curr_ROT - mean) <= (2 * ROT_stdev + 5))  {
        m_DiffHistory.push_front(curr_ROT);
        while(m_DiffHistory.size() > m_ROT_filter_len) {
          m_DiffHistory.pop_back();
        }
        //Add the new value
        m_current_ROT = (sum + curr_ROT) / m_DiffHistory.size();
      } else {
        m_current_ROT = mean;
      }
    } else {  // too small diff history
      //MOOSTrace("Diff Hist Too Small\n");
      if (fabs(curr_ROT) < m_max_ROT) {
        m_DiffHistory.push_front(curr_ROT);
        m_current_ROT = curr_ROT;
      } else {
        m_current_ROT = CourseChangeMRAS::TwoSidedLimit(curr_ROT, m_max_ROT);
      }
    }
  }
}





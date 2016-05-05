/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: MarineMRAS.cpp                                  */
/*    DATE: 2015-12-06                                      */
/************************************************************/

#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "MarineMRAS.h"
#include "AngleUtils.h"

using namespace std;

// Found to be ~20 when does not align with desired
// Could be ~10 with proper KI
#define TURN_THRESHOLD 20 //degrees
#define HEADING_HIST_SEC 10

#define DEBUG false

//---------------------------------------------------------
// Constructor

MarineMRAS::MarineMRAS()
{
  std::cout << "Initializing MRAS\n";

  m_k_star = 1;
  m_tau_star = 1;
  m_z = 1;
  m_wn = 2;
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
  m_speed_factor   = 0;
  m_max_thrust = 100;
  m_rudder_speed = 15;
  m_discard_large_ROT = false;
  m_rudder_deadband = 0;
  m_output = true;
  m_record_mode = false;
  m_course_keep_only = false;
  m_adapt_turns = false;
  m_speed_var = "NAV_SPEED_OVER_GROUND";
  m_cog_var = "NAV_HEADING_OVER_GROUND";

  m_first_heading = true;
  m_current_ROT = 0;
  m_has_control = false;
  m_desired_thrust = 50;
  m_desired_speed = 0;
  m_allstop_posted = false;
  m_last_controller = ControllerType::CourseChange;
  m_end_last_turn = MOOSTime();
  m_last_iterate_time = MOOSTime();
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
    } else if (key == m_speed_var) {
      m_current_speed = msg.GetDouble();
      m_current_speed_time = msg.GetTime();
    } else if (key == "DESIRED_HEADING") {
      m_desired_heading = msg.GetDouble();
      AddHeadingHistory(m_desired_heading, msg.GetTime());
    } else if (key == m_cog_var) {
      m_current_cog = angle180(msg.GetDouble());
    } else if (key == "DESIRED_SPEED") {
      m_desired_speed = msg.GetDouble();
    } else if((key == "MOOS_MANUAL_OVERIDE") || (key == "MOOS_MANUAL_OVERRIDE")) {
      MOOSTrace("pMarineMRAS: Received Manual Override Change\n");
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
  #if DEBUG
  MOOSTrace("pMarineMRAS: Starting iterate\n");
  #endif
  AppCastingMOOSApp::Iterate();

  if (m_has_control) {
    try {
      // ------ Determine the thrust --------
      #if DEBUG
      MOOSTrace("pMarineMRAS: Determining Thrust, speed factor: %0.2f\n", m_speed_factor);
      #endif
      if(m_speed_factor != 0) {
        m_desired_thrust = m_desired_speed * m_speed_factor;
      } else {
        m_desired_thrust = m_speed_control.Run(m_desired_speed, m_current_speed,
          m_desired_heading, m_current_heading, m_current_speed_time, IsTurning(),
          m_current_cog);
      }

      // ------- Determine the rudder -------
      #if DEBUG
      MOOSTrace("pMarineMRAS: Determining Rudder\n");
      #endif
      double desired_rudder = 0;
      //prevent controller runup when speed is 0
      ControllerType controller_to_use = DetermineController();
      if (!m_first_heading && m_desired_thrust > 0 && m_desired_speed > 0) {
        if (controller_to_use ==  ControllerType::CourseChange) {
          //MOOSTrace("Using Course Change Controller\n");
          if (m_last_controller == ControllerType::CourseKeep) {
            m_CourseControl.ResetModel(m_current_heading, m_current_ROT,
              m_CourseKeepControl.GetModelRudder());
            m_CourseControl.SwitchController(m_CourseKeepControl.GetTauStar(),
              m_CourseKeepControl.GetKStar());
            m_last_controller = ControllerType::CourseChange;
          }
          // Still run the course keep control, since it only updates during turns
          bool turning = IsTurning();
          m_CourseKeepControl.Run(m_desired_heading, m_current_heading,
            m_current_ROT, m_current_speed, m_last_heading_time,
            turning && m_current_speed > 0.5, turning);
          desired_rudder = m_CourseControl.Run(m_desired_heading, m_current_heading,
            m_current_ROT, m_desired_speed, m_last_heading_time);
        } else {
          // Use the course keep controller
          if (m_last_controller == ControllerType::CourseChange) {
            m_CourseKeepControl.ResetModel(m_current_heading, m_current_ROT,
              m_CourseControl.GetModelRudder());
            m_CourseKeepControl.SwitchController();
            m_last_controller = ControllerType::CourseKeep;
          }
          bool do_adapt = true;
          if (controller_to_use == ControllerType::CourseKeepNoAdapt)
            do_adapt = false;
          //using desired speed instead of current to prevent minor fluctuations
          desired_rudder = m_CourseKeepControl.Run(m_desired_heading, m_current_heading,
            m_current_ROT, m_current_speed, m_last_heading_time, do_adapt, IsTurning());
        }
      }

      if (m_output) {
        Notify("DESIRED_THRUST", m_desired_thrust);
        Notify("DESIRED_RUDDER", desired_rudder);
      }

      #if DEBUG
      MOOSTrace("pMarineMRAS: Logging Variables\n");
      #endif
      //Debug variables for logging
      double vars[12];
      if (controller_to_use ==  ControllerType::CourseChange) {
        m_CourseControl.GetDebugVariables(vars);
      } else {
        m_CourseKeepControl.GetDebugVariables(vars);
      }
      Notify("MRAS_KP", vars[0]);
      Notify("MRAS_KD", vars[1]);
      Notify("MRAS_KI", vars[2]);
      Notify("MRAS_RUDDER_OUT", vars[3]);
      Notify("MRAS_MODEL_HEADING", vars[4]);
      Notify("MRAS_MODEL_ROT", vars[5]);
      if (controller_to_use == ControllerType::CourseChange) {
        Notify("MRAS_SERIES_MODEL_HEADING", vars[6]);
        Notify("MRAS_SERIES_MODEL_ROT", vars[7]);
        Notify("MRAS_PSI_REF_P", vars[8]);
        Notify("MRAS_PSI_REF_PP", vars[9]);
      } else {
        Notify("MRAS_TAU_M_STAR", vars[6]);
        Notify("MRAS_PSI_PP", vars[7]);
        Notify("MRAS_TAU_M", vars[8]);
        Notify("MRAS_K_M", vars[9]);
        Notify("MRAS_K_M_STAR", vars[11]);
      }
      Notify("MRAS_MODEL_RUDDER", vars[10]);
      Notify("MRAS_IS_TURNING", IsTurning());

      double speed_vars[1];
      m_speed_control.GetVarInfo(speed_vars);
      Notify("MRAS_SPEED_STATE", speed_vars[0]);

      Notify("NAV_ROT", m_current_ROT);
      Notify("NAV_HEADING_180", m_current_heading);
      Notify("DESIRED_HEADING_180", angle180(m_desired_heading));
      m_allstop_posted = false;
    } catch (std::exception& err) {
      MOOSTrace("pMarineMRAS: Error in Iterate %c\n",err.what());
    } catch (...) {
      MOOSTrace("Other exception\n");
    }
  } else if (!m_record_mode) {
    #if DEBUG
    MOOSTrace("pMarineMRAS: Posting All Stop\n");
    #endif
    PostAllStop();
  }

  if (m_record_mode) {
    #if DEBUG
    MOOSTrace("Recording Mode Running, time %.1f\n", MOOSTime());
    #endif
    Notify("NAV_ROT", m_current_ROT);
    Notify("NAV_HEADING_180", m_current_heading);
    Notify("DESIRED_HEADING_180", angle180(m_desired_heading));
  }
  double this_time = MOOSTime();
  m_iterate_len = this_time - m_last_iterate_time ;
  m_last_iterate_time = this_time;

  #if DEBUG
  MOOSTrace("pMarineMRAS: Posting Appcast Report (end iterate)\n");
  #endif

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool MarineMRAS::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  // Variables to be set from the parameters
  std::string thrust_map = "";
  bool use_thrust_map_only = false;

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
    else if (param == "NATURALFREQUENCY") {
      m_wn = dval;
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
    } else if (param == "THRUSTPERCENT") {
      m_desired_thrust = dval;
      handled = true;
    } else if(param == "SPEED_FACTOR") {
      m_speed_factor = vclip(dval, 0, 100);
      handled = true;
    } else if(param == "MAXTHRUST") {
      m_max_thrust = dval;
      handled = true;
    } else if(param == "RUDDERSPEED") {
      m_rudder_speed = dval;
      handled = true;
    } else if (param == "DISCARDLARGEROT") {
      if (toupper(value) == "TRUE")
        m_discard_large_ROT = true;
      handled = true;
    } else if (param == "RUDDERDEADBAND") {
      m_rudder_deadband = dval;
      handled = true;
    } else if (param == "NOOUTPUT") {
      if (toupper(value) == "TRUE")
        m_output = false;
      handled = true;
    } else if (param == "RECORDMODE") {
      if (toupper(value) == "TRUE")
        m_record_mode = true;
      handled = true;
    } else if (param == "COURSEKEEPONLY") {
      if (toupper(value) == "TRUE")
        m_course_keep_only = true;
      handled = true;
    } else if (param == "ADAPTDURINGTURNS") {
      if (toupper(value) == "TRUE")
        m_adapt_turns = true;
      handled = true;
    } else if (param == "THRUST_MAP") {
      thrust_map = value;
      handled = true;
    } else if (param == "USETHRUSTMAPONLY") {
      if (toupper(value) == "TRUE")
        use_thrust_map_only = true;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }

  //Initialize the Control system
  m_CourseControl.SetParameters(m_k_star, m_tau_star, m_z, m_beta,
        m_alpha, m_gamma, m_xi, m_rudder_limit, m_cruising_speed, m_length,
        m_max_ROT, m_decrease_adapt, m_rudder_speed);
  m_CourseKeepControl.SetParameters(m_k_star, m_tau_star, m_z, m_wn, m_beta,
        m_alpha, m_gamma, m_xi, m_rudder_limit, m_cruising_speed, m_length,
        m_max_ROT, m_decrease_adapt, m_rudder_speed, m_rudder_deadband);
  m_speed_control.SetParameters(thrust_map, m_max_thrust, use_thrust_map_only);

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void MarineMRAS::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_HEADING", 0);
  Register(m_speed_var, 0);
  Register(m_cog_var, 0);
  Register("DESIRED_HEADING", 0);
  Register("DESIRED_SPEED", 0);
  Register("MOOS_MANUAL_OVERIDE", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool MarineMRAS::buildReport()
{
  m_msgs << "============================================ \n";
  m_msgs << "pMarineMRAS                                  \n";
  m_msgs << "============================================ \n";

  if (m_has_control) {
    ACTable actab(6);
    actab << "Kp | Kd | Ki | Model Heading | Measured | Desired ";
    actab.addHeaderLines();
    ControllerType controller_in_use = DetermineController();
    if (controller_in_use == ControllerType::CourseChange)
      actab << m_CourseControl.GetStatusInfo();
    else
      actab << m_CourseKeepControl.GetStatusInfo();
    actab << " | | | | | ";
    actab.addHeaderLines();
    if (controller_in_use == ControllerType::CourseChange) {
      actab << "Psi_r'' | Psi_r' | Rudder | Series Heading | Model ROT | Series ROT";
      actab << m_CourseControl.GetDebugInfo();
    } else {
      actab << "Km | Km* | Tm | Tm* | Model ROT | Measured ROT";
      actab << m_CourseKeepControl.GetDebugInfo();
    }
    m_msgs << actab.getFormattedString();

    if (controller_in_use == ControllerType::CourseChange)
      m_msgs << "\nCourse Change Controller in use.";
    else if (controller_in_use == ControllerType::CourseKeep)
      m_msgs << "\nCourse Keep Controller in use w/ adaptation.";
    else if (controller_in_use == ControllerType::CourseKeepNoAdapt) {
      m_msgs << "\nCourse Keep Controller in use, no adaptation.";
    }
    if (IsTurning()) {
      m_msgs << "\n\nIs Turning? yes";
    } else {
      m_msgs << "\n\nIs Turning? no";
    }
    m_msgs << "\n\n" << m_speed_control.AppCastMessage();
  } else if (m_record_mode) {
    m_msgs << "Recording Mode Running.";
  } else {
    m_msgs << "Control not running.";
  }

  m_msgs << "\n\nIterate timing: " << m_iterate_len;

  return(true);
}

void MarineMRAS::UpdateROT(double curr_time) {
  if (m_first_heading) {
    m_first_heading = false;
  } else if (m_ROT_filter_len > 1) {
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
      //Added the +5 to account for small stdev
      if (fabs(curr_ROT - m_DiffHistory.front()) <= (2 * ROT_stdev + 10) ||
        !m_discard_large_ROT)  {
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
      if (fabs(curr_ROT) < m_max_ROT || m_record_mode) {
        m_DiffHistory.push_front(curr_ROT);
        m_current_ROT = curr_ROT;
      } else {
        m_current_ROT = CourseChangeMRAS::TwoSidedLimit(curr_ROT, m_max_ROT);
      }
    }
  } else {
    double diff = angle180(m_current_heading - m_previous_heading);
    double curr_ROT = diff / (curr_time - m_last_heading_time);
    //this is an arbitary threshold to eliminate noise from sim
    if (fabs(curr_ROT - m_current_ROT) < 10 || !m_discard_large_ROT) {
      m_current_ROT = curr_ROT;
    }

    //Try limiting it for sim
    m_current_ROT = CourseChangeMRAS::TwoSidedLimit(m_current_ROT, m_max_ROT);
  }
}

void MarineMRAS::PostAllStop()
{
  if(m_allstop_posted)
    return;

  if (m_output) {
    Notify("DESIRED_RUDDER", 0.0);
    Notify("DESIRED_THRUST", 0.0);
  }

  m_allstop_posted = true;
}

void MarineMRAS::AddHeadingHistory(double heading, double heading_time) {
  m_desired_heading_history.push_front(angle180(heading));
  m_desired_hist_time.push_front(heading_time);

  //Keep last 10 sec of data - this needs to adapt to turn rate of vessel
  double heading_hist_sec = GetSettleTime() * 1.5;
  while (heading_time - m_desired_hist_time.back() > heading_hist_sec) {
    m_desired_hist_time.pop_back();
    m_desired_heading_history.pop_back();
  }
}

ControllerType MarineMRAS::DetermineController() {
  if (MOOSTime() - m_desired_hist_time.back() > GetSettleTime()) {
    //10 degree heading change threshold for CourseChange
    if (!IsTurning()) {
      return ControllerType::CourseKeep;
    } else {
      //Course change is the default if we have less than 10 sec same course
      if (m_course_keep_only) {
        if (m_adapt_turns) {
          return ControllerType::CourseKeep;
        } else {
          return ControllerType::CourseKeepNoAdapt;
        }
      } else {
        return ControllerType::CourseChange;
      }
    }
  } else {
    //What to do when we don't have much history
    if (m_course_keep_only) {
      // Don't adapt if we aren't settled
      return ControllerType::CourseKeepNoAdapt;
    } else {
      return ControllerType::CourseChange;
    }
  }
}

bool MarineMRAS::IsTurning() {
  if (m_desired_heading_history.size() > 2) {
    // bool recent_change = fabs(angle180(m_desired_heading_history.back()
    //                       - angle180(m_desired_heading))) > TURN_THRESHOLD;
      bool not_near_desired = fabs(angle180(angle180(m_current_heading) -
                              angle180(m_desired_heading))) > TURN_THRESHOLD;
      if (not_near_desired) {
        m_end_last_turn = m_desired_hist_time.front();
      }
      bool recent_change = true;
      // Even in the case of CourseChange, TauM is based on settling time
      double settle_time = GetSettleTime();
      if ((MOOSTime() - m_end_last_turn) > settle_time) {
        recent_change = false;
      }
      return not_near_desired || recent_change;
  } else {
    //assume we are not turning if we don't know
    return false;
  }
}

double MarineMRAS::GetSettleTime() {
  return 7 * m_CourseKeepControl.GetTauM();
}

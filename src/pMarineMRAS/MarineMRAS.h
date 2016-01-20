/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: UNH                                              */
/*    FILE: MarineMRAS.h                                          */
/*    DATE: 2015-12-06                            */
/************************************************************/

#ifndef MarineMRAS_HEADER
#define MarineMRAS_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "CourseChangeMRAS.h"
#include "CourseKeepMRAS.h"

enum class ControllerType {
  CourseChange,
  CourseKeep
};

class MarineMRAS : public AppCastingMOOSApp
{
 public:
   MarineMRAS();
   ~MarineMRAS() {};


 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void UpdateROT(double curr_time);
   void PostAllStop();
   void AddHeadingHistory(double heading, double heading_time);
   ControllerType DetermineController();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // Configuration variables
    double m_k_star;
    double m_tau_star;
    double m_z;
    double m_beta;
    double m_alpha;
    double m_gamma;
    double m_xi;
    double m_rudder_limit;
    double m_rudder_deadband;
    double m_max_ROT;
    double m_cruising_speed;
    double m_length;
    bool   m_decrease_adapt;
    double m_desired_thrust;
    bool   m_allstop_posted;
    double m_speed_factor;
    double m_max_thrust;
    double m_rudder_speed;
    bool   m_discard_large_ROT;
    bool   m_output;

 private: // State variables
    double m_desired_heading;
    double m_current_heading;
    double m_desired_speed;
    double m_current_speed;
    double m_last_heading_time;
    bool   m_first_heading;
    double m_previous_heading;
    double m_current_ROT;
    std::list<double> m_DiffHistory;
    int m_ROT_filter_len;
    bool   m_has_control;
    std::list<double> m_desired_heading_history;
    std::list<double> m_desired_hist_time;

    CourseChangeMRAS m_CourseControl;
    CourseKeepMRAS m_CourseKeepControl;
    ControllerType m_last_controller;

};

#endif 

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
    double m_max_ROT;
    double m_cruising_speed;
    double m_length;

 private: // State variables
    double m_desired_heading;
    double m_current_heading;
    double m_current_speed;
    double m_last_heading_time;
    bool   m_first_heading;
    double m_previous_heading;
    double m_current_ROT;
    bool   m_has_control;

    CourseChangeMRAS m_CourseControl;

};

#endif 

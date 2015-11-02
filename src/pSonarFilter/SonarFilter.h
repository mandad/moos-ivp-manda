/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: MIT                                             */
/*    FILE: SonarFilter.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef SonarFilter_HEADER
#define SonarFilter_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class SonarFilter : public AppCastingMOOSApp
{
 public:
   SonarFilter();
   ~SonarFilter() {};

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void InjestDepthVal(double depth);
   double GetStDev(std::list<double> * v);
   double GetMean(std::list<double> * v);
   std::string GenerateSwathMessage();

 private: // Configuration variables
    enum sonar_type
    {
      SBES,
      MBES 
    };
    int                 m_filter_len;
    double              m_std_limit;
    double              m_sim_swath_angle;
    sonar_type          m_sonar_type;


 private: // State variables
    std::list<double>   m_all_depths;
    double              m_last_valid_depth;
    bool                m_fresh_depth;
    char                m_last_msg[200];
    int                 m_cycles_since_last;
};

#endif 

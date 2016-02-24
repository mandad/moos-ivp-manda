/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#ifndef SurveyPath_HEADER
#define SurveyPath_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class SurveyPath : public AppCastingMOOSApp
{
 public:
   SurveyPath();
   ~SurveyPath() {};

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

 private: // State variables
};

#endif

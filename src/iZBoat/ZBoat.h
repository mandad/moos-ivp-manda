/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: MIT                                             */
/*    FILE: ZBoat.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ZBoat_HEADER
#define ZBoat_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class ZBoat : public AppCastingMOOSApp
{
 public:
   ZBoat();
   ~ZBoat() {};

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

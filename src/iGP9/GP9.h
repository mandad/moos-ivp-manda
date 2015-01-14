/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: UNH, Durham NH                                          */
/*    FILE: GP9.h                                          */
/*    DATE: Jan 12th 2015                             */
/************************************************************/

#ifndef GP9_HEADER
#define GP9_HEADER

#include <iostream>
#include <math.h>
#include <cstring>

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "SerialComms.h"

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"


class GP9 : public AppCastingMOOSApp
{
 public:
   GP9();
   ~GP9() {};

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

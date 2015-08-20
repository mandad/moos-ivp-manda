/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: MIT                                             */
/*    FILE: ZBoat.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef ZBoat_HEADER
#define ZBoat_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "MOOS/libMOOS/MOOSLib.h"

class ZBoat : public CMOOSInstrument
{
 public:
   ZBoat();
   virtual ~ZBoat();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

   //CMOOSInstrument Functions
   bool InitialiseSensor();

 // protected: // Standard AppCastingMOOSApp function to overload 
   // bool buildReport();

 protected:
   void registerVariables();
   bool GetData();
   bool PublishData();
   void GeneratePWMMessage();

 private: // Configuration variables
   double m_dfMaxRudder;
   double m_dfMaxThrottle;

 private: // State variables
};

#endif 

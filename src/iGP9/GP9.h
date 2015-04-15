/************************************************************/
/*    NAME: Damian Manda                                              */
/*    ORGN: UNH, Durham NH                                          */
/*    FILE: GP9.h                                          */
/*    DATE: Apr 15th 2015                             */
/************************************************************/

#ifndef GP9_HEADER
#define GP9_HEADER

#include <iostream>
#include <math.h>
#include <cstring>

#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
//#include "SerialComms.h"
#include  "comms.h"
#include  "registers.h"
#include  "serial/serial.h"

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
   // Fuctions from ROS library
   // bool handleResetService(gp9::Comms* sensor, const gp9::Reset::Request& req, 
   //    const gp9::Reset::Response& resp);
   void publishMsgs(gp9::Registers& r);
   void configureSensor(gp9::Comms* sensor);
   template<typename RegT>
   void sendCommand(gp9::Comms* sensor, const gp9::Accessor<RegT>& reg, std::string human_name);
   // template<typename RegT>
   // void configureVector3(gp9::Comms* sensor, const gp9::Accessor<RegT>& reg,
   //    std::string param, std::string human_name);


 private: // Configuration variables
    std::string serialPort;
    int32_t baudRate;
    const int defaultBaudRate;

 private: // State variables
    serial::Serial ser;
    float covar[9];     // orientation covariance values
    bool first_failure;
    gp9::Comms sensor;
    gp9::Registers registers;

};

#endif 

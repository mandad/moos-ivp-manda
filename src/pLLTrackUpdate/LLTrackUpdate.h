/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH/NOAA                                        */
/*    FILE: LLTrackUpdate.h                                 */
/*    DATE: 25 Aug 2015                                     */
/************************************************************/

#ifndef LLTrackUpdate_HEADER
#define LLTrackUpdate_HEADER

#include <string>
#include "MOOS/libMOOS/MOOSLib.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "XYSegList.h"

using namespace std;

// struct GeoPt {
// 	double x,
// 	double y
// };

class LLTrackUpdate : public CMOOSApp
{
 public:
   LLTrackUpdate();
   ~LLTrackUpdate();

 protected:
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();
   void RegisterVariables();
   CMOOSGeodesy m_Geodesy;
   XYSegList ProcessUTMString(string utm_points_msg);
   XYSegList PointsString2SegList (string points_msg);

 private: // Configuration variables

 private: // State variables
   unsigned int m_iterations;
   double       m_timewarp;
   bool			m_survey_update;
   bool			m_turn_update;
   bool			m_alignment_update;
   XYSegList    m_survey_pts;
   XYSegList	m_turn_pts;
   XYSegList	m_alignment_pts;
};

#endif 

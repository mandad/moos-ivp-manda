/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SurveyPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#ifndef SurveyPath_HEADER
#define SurveyPath_HEADER

#include <thread>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "RecordSwath.h"
#include "PathPlan.h"

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
   BoatSide AdvanceSide(BoatSide side);
   bool InjestSwathMessage(std::string msg);
   void PostSurveyRegion();
   bool DetermineStartAndTurn(XYSegList& next_pts, bool post_turn = false);
   void CreateNewPath();
   void PostSwathSide();

 private: // Configuration variables
  BoatSide m_first_swath_side;
  double m_swath_interval;
  double m_alignment_line_len;
  double m_turn_pt_offset;
  bool m_remove_in_coverage;
  double m_swath_overlap;

 private: // State variables
  //BoatSide m_next_swath_side;
  BoatSide m_swath_side;
  // double m_swath_width;
  // double m_nav_x;
  // double m_nav_y;
  // double m_nav_heading;
  bool m_line_end;
  bool m_line_begin;
  bool m_turn_reached;
  // bool m_post_ready;
  // bool m_path_ready;
  bool m_recording;
  BPolygon m_op_region;
  //XYSeglist m_first_line;
  RecordSwath m_swath_record;
  std::map<std::string, double> m_swath_info;
  std::string m_posted_path_str;
  XYSegList m_survey_path;
  XYPoint m_turn_pt;
  bool m_turn_pt_set;
  bool m_post_turn_when_ready;
  bool m_path_plan_done;
  XYSegList m_alignment_line;

  std::thread m_path_plan_thread;
};

#endif

/**
 * @file PathPlan.h
 * @brief Plans a path for surveying based on a recorded path and swath.
 * @details Paths are offset and processed to give a valid vehicle path
 * @author Damian Manda
 * @date 25 Feb 2016
 * @copyright MIT License
 */

#ifndef SurveyPath_PathPlan_HEADER
#define SurveyPath_PathPlan_HEADER

#include "RecordSwath.h"
#include "XYSegList.h"
#include "XYPolygon.h"
#include <Eigen/Core>

typedef Eigen::Matrix<double, Eigen::Dynamic, 2> PointList;

class PathPlan
{
  public:
    PathPlan(RecordSwath last_swath, BoatSide side, XYPolygon op_region,
      double margin=0.2, bool restrict_to_region = true);
    ~PathPlan() {};
    XYSegList GenerateNextPath();


  private:
    // Configuration variables
    bool m_restrict_asv_to_region;
    double m_max_bend_angle;
    double m_margin;


    // State variables
    RecordSwath m_last_line;
    BoatSide m_planning_side;
    PointList m_next_path_pts;
};

#endif

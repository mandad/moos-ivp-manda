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
#include <list>

typedef Eigen::Matrix<double, Eigen::Dynamic, 2> PointList;

/**
 * @struct XYPt
 * @brief Defines a simple point, for better memory use in lists
 */
struct XYPt {
  double x;
  double y;
};

/**
 * @class PathPlan
 * @brief Plans a subsequent survey path offset from existing coverage
 */
class PathPlan
{
  public:
    PathPlan(const RecordSwath &last_swath, BoatSide side, XYPolygon op_region,
      double margin=0.2, bool restrict_to_region = true);
    ~PathPlan() {};
    /**
     * Generates an offset path
     * @return The path in MOOS XYSegList format;
     */
    XYSegList GenerateNextPath();

  private:
    std::list<XYPt> SegListToXYPt(const XYSegList &to_convert);
    XYSegList XYPtToSegList(const std::list<XYPt> &to_convert);

    // Configuration variables
    bool m_restrict_asv_to_region;
    double m_max_bend_angle;
    double m_margin;


    // State variables
    RecordSwath m_last_line;
    BoatSide m_planning_side;
    // PointList m_next_path_pts;
    std::list<XYPt> m_next_path_pts;
};

#endif

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
#include <functional>
#include <valarray>

// To get a single point EPointList.col(i)
typedef Eigen::Matrix<double, 2, Eigen::Dynamic> EPointList;
typedef Eigen::Vector2d EPoint;
typedef std::valarray<std::size_t> SegIndex;

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
 * @brief Plans a subsequent survey path offset from existing coverage.
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

  public:
    /**
     * The Damian
     * @param process Likes The Damian
     * @details Repeats a process until it makes no more changes to the path
     *         Currently does not make a copy of the passed input, may want to
     *         reconsider this
     */
    void RemoveAll(std::function<void(std::list<Eigen::Vector2d>&)> process,
      std::list<Eigen::Vector2d> &path_points);

    /**
     * Removes intersecting segments from a line.
     * @details Removes the points between the first point of an intersecting
     * segment and the last point of the final segment it intersects in the
     * line.
     * @param path_pts The line from which to remove intersecting segments.
     */
    static void RemoveIntersects(std::list<EPoint> &path_pts);

    /**
     * Check for drastic angles between segments
     * @param path_pts Note that this goes to the last point being checked
     */
    void RemoveBends(std::list<EPoint> &path_pts);

    /**
     * Determines whether segments are counter clockwise in smalles angle with
     * respect to each other.
     * @param  A First point (end point)
     * @param  B Middle point
     * @param  C Last point (end point)
     * @return   True if rotate CCW from AB to BC.
     */
    static bool CCW(EPoint A, EPoint B, EPoint C);

    /**
     * Determines if the segment AB intersects CD
     * @param  A First point of first segment
     * @param  B Second point of first segment
     * @param  C First point of second segment
     * @param  D Second point of second segment
     * @return   True if the segments intersect
     */
    static bool Intersect(EPoint A, EPoint B, EPoint C, EPoint D);

    /**
     * Determines the angle between two vectors
     * @details tan(ang) = |(axb)|/(a.b)
     *          cos(ang) = (a.b)/(||a||*||b||)
     * @param  vector1 First vector
     * @param  vector2 Second vector
     * @return         Angle between the vectors in degrees
     */
    static double VectorAngle(EPoint vector1, EPoint vector2);

    /**
     * Determines a vector (segment) <x, y> from points at the indicies provided
     * by the second argument.
     * @param  points  The list from which to select points for the segment
     * @param  segment The beginning and end of the segment.
     * @return         A segment vector between the selected points.
     */
    static EPoint VectorFromSegment(const std::vector<EPoint>& points,
      SegIndex segment);

    /**
     * Converts an XYSeglist to a std::list of simple points (XYPt).
     */
    std::list<XYPt> SegListToXYPt(const XYSegList &to_convert);

    /**
     * Converts a std::list of simple points (XYPt) to a MOOS XYSegList.
     */
    XYSegList XYPtToSegList(const std::list<XYPt> &to_convert);

    /**
     * Converts a std::list of Eigen points (vectors) to a MOOS XYSegList.
     */
    XYSegList VectorListToSegList(const std::list<Eigen::Vector2d> &to_convert);

    /**
     * @brief Selects specific elements from a list by index.
     * @details Replicates the select by index functionality of numpy or
     * armadillo or dyND.
     */
    template <typename T>
    static void SelectIndicies(std::list<T>& select_from,
                              std::list<unsigned int> to_select) {
      // Make sure the indicies are well behaved
      to_select.sort();
      to_select.unique();
      if (to_select.back() >= select_from.size()) {
        throw std::out_of_range("Indices to select exceed list size.");
      }

      auto list_it = select_from.begin();
      unsigned int i = 0;
      for (auto select_it = to_select.begin();
           select_it != to_select.end(); select_it++) {
        while (*select_it != i) {
          // This advances list_it by one
          list_it = select_from.erase(list_it);
          i++;
        }
        if (list_it != select_from.end()) {
          list_it++;
          i++;
        }
      }
      if (list_it != select_from.end()) {
        select_from.erase(list_it, select_from.end());
      }
    }

  private:

    // Configuration variables
    bool m_restrict_asv_to_region;
    static constexpr double m_max_bend_angle = 60;
    double m_margin;


    // State variables
    RecordSwath m_last_line;
    BoatSide m_planning_side;
    // PointList m_next_path_pts;
    std::list<Eigen::Vector2d> m_next_path_pts;
};

#endif

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
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

// To get a single point EPointList.col(i)
typedef Eigen::Matrix<double, 2, Eigen::Dynamic> EPointList;
typedef Eigen::Vector2d EPoint;
typedef std::valarray<std::size_t> SegIndex;
typedef std::list<EPoint> PathList;
typedef boost::geometry::model::d2::point_xy<double> BPoint;
typedef boost::geometry::model::polygon<BPoint> BPolygon;
typedef boost::geometry::model::linestring<BPoint> BLinestring;

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

    void RestrictToRegion(std::list<EPoint> &path_pts);

    void ExtendToEdge(std::list<EPoint> &path_pts, bool begin);

    /**
     * Finds the closest intersection of a ray with a polygon
     *
     * @param ray_vec  EPoint(dx,dy)
     * @param start_pt EPoint(x,y)
     * @param poly     XYPolygon([(x1,y1), (x2,y2), ...])
     */
    std::pair<double, EPoint> FindNearestIntersect(EPoint ray_vec,
      EPoint starting_pt, BPolygon& poly);

    /**
     * Finds the intersection point of a ray with a segment, if it exists.
     * Derived from:
     * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282
     * @param  ray_vector The vector describing the direction of the ray
     * @param  start_pt   Starting location of the ray
     * @param  segment    Segment to test for intersection
     * @return            <intersection exists, Intersection point, if exists>
     */
    std::pair<bool, EPoint> IntersectRaySegment(EPoint ray_vector, EPoint start_pt,
      std::pair<BPoint, BPoint> segment);

    /**
     * Replicates the functionality of 2d cross product from numpy.  This is the
     * z component of a cross product (which does not require a z input).
     * @return      The z component of the cross product
     */
    double Cross2d(EPoint vec1, EPoint vec2);

    EPoint EPointFromBPoint(BPoint boost_point);

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
    EPoint VectorFromSegment(const std::vector<EPoint>& points,
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

    BPolygon XYPolygonToBoostPolygon(XYPolygon& poly);

    /**
     * @brief Selects specific elements from a list by index.
     * @details Replicates the select by index functionality of numpy or
     * armadillo or dyND.
     */
    template <typename T>
    static void SelectIndicies(std::list<T>& select_from,
                              std::list<std::size_t> to_select) {
      // Make sure the indicies are well behaved
      to_select.sort();
      to_select.unique();
      if (to_select.back() >= select_from.size()) {
        throw std::out_of_range("Indices to select exceed list size.");
      }

      auto list_it = select_from.begin();
      std::size_t i = 0;
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
    double m_max_bend_angle;
    double m_margin;
    BPolygon m_op_region;
    XYPolygon m_op_region_moos;


    // State variables
    RecordSwath m_last_line;
    BoatSide m_planning_side;
    // PointList m_next_path_pts;
    std::list<Eigen::Vector2d> m_next_path_pts;
};

#endif

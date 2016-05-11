/**
 * @file RecordSwath.h
 * @brief Records a swath history from a sonar on a moving vehicle.
 * @details Also gives the outer points at the edge of the swath along a track.
 * @author Damian Manda
 * @date 23 Feb 2016
 * @copyright MIT License
 */

#ifndef SurveyPath_RecordSwath_HEADER
#define SurveyPath_RecordSwath_HEADER

//GEOS Headers
//#include <geos.h>
#include <geos/geom/PrecisionModel.h>
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Geometry.h>
#include <geos/geom/Point.h>
#include <geos/geom/Polygon.h>
#include <geos/util/GEOSException.h>
#include <geos/util/IllegalArgumentException.h>

//MOOS Headers
#include "XYPoint.h"
#include "XYSegList.h"
#include "XYVector.h"

#include <list>
#include <vector>
#include <map>

//#include "PathPlan.h"

/**
 * @enum BoatSide
 * @brief Indicates the side of a boat for the swath
 */
enum class BoatSide {
  Stbd,
  Port,
  Unknown
};

/**
 * @class RecordSwath
 * @brief Records points of a sonar swath for analysis of the coverage and
 * subsequent tracks.
 */
class RecordSwath
{
private:
  /**
   * @struct SwathRecord
   * @brief Stores the location and width of one measured sonar swath.
   */
  struct SwathRecord {
    double loc_x;
    double loc_y;
    double heading;
    // Could make this a map w/ BoatSide index
    double swath_stbd;
    double swath_port;
    // This will either be the nadir depth (if this is all we have) or outer
    // swath depth
    double depth;
  };

public:
  RecordSwath(double interval = 10);
  ~RecordSwath() {};
  /**
  * Adds a recorded swath to the path.
  * @param  swath_stbd Swath width to starboard
  * @param  swath_port Swath width to port
  * @param  loc_x      X coordinate of position where record takes place
  * @param  loc_y      Y coordinate of position
  * @param  heading    Heading of the vessel at the time of recording
  * @return            True if the record coverage was successfully added
  */
  bool AddRecord(double swath_stbd, double swath_port, double loc_x, double loc_y,
        double heading, double depth);
  /**
  * Resets the storage for a new line.
  */
  void ResetLine();

  /**
  * Saves the last point to a record.
  * This makes sure that the last swath (after crossing the boundary) is
  * recorded so that it is included in planning.
  * @return If the min_record is valid
  */
  bool SaveLast();

  /**
  * Get all of the points on one side of the swath limits
  * @param side   The side of the boat on which to return the swath
  * @return       An ordered list of the points on the outside of the swath
  */
  XYSegList SwathOuterPts(BoatSide side);

  std::pair<XYPoint, XYPoint> LastOuterPoints();

  bool SwathCoverage(BoatSide side, geos::geom::Polygon &coverage);

  /**
  * Gets a specific width along a recorded decimated swath
  * @param  side  Side of the boat on which the swath was recorded
  * @param  index Position of the desired swath
  * @return       The swath width in meters
  */
  double SwathWidth(BoatSide side, unsigned int index);

  /**
   * Gets all the minimum swath widths on a side (recorded at the set side)
   * @param side The side from which to get swaths
   * @return     A vector of swath widths.
   */
  std::vector<double> AllSwathWidths(BoatSide side);

  /**
   * Gets the x,y location of where a specific min swath was recorded
   * @param  index The index along the swath record
   * @return       XYPoint of recording location
   */
  XYPoint SwathLocation(unsigned int index);

  /**
  * Sets the side that will be used for outer point determination
  * @param side Side of the boat on which to generate outer swath points
  */
  void SetOutputSide(BoatSide side) { m_output_side = side; }

  /**
   * Gets the side on which minimum interval points are being processed
   */
  BoatSide GetOutputSide() {return m_output_side; }

  /**
   * The distance between subsequent analysis intervals for swath minimums.
   */
  double IntervalDist() { return m_interval; }

  /**
   * Determines if the record has valid points for building a path.
   */
  bool ValidRecord();

 protected:
  /**
  * Determines the minimum swath over the recorded interval and places it into
  * a list of minimums.
  */
  void MinInterval();

  /**
  * Gets the x,y position of the edge of a swath from a record
  * @param  record The swath record to use for location and width
  * @param  side   The side of the boat on which to project the swath
  * @return        Location of the swath outer points
  */
  XYPoint OuterPoint(const SwathRecord &record, BoatSide side);

  /**
  * Adds a record to the coverage model.
  * @param  record The record to add
  * @return        Whether the record was able to be added sucessfully (no
  * geometry errors).
  */
  bool AddToCoverage(SwathRecord record);

 private:
  // Configuration Variables
  double m_min_allowable_swath;
  double m_interval;

  // State varables
  std::vector<SwathRecord> m_interval_record;
  std::list<SwathRecord> m_min_record;
  std::map<BoatSide, std::vector<double>> m_interval_swath;
  double m_last_x;
  double m_last_y;
  bool m_has_records;
  double m_acc_dist;
  SwathRecord m_previous_record;
  BoatSide m_output_side;

  // Geometry Stuff
  std::map<BoatSide, std::vector<geos::geom::Point*>> m_outer_points;
  geos::geom::Polygon* m_coverage;
  geos::geom::GeometryFactory* m_geom_factory;

};

#endif

/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: RecordSwath.cpp                                 */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#include "RecordSwath.h"
#include <cmath>
#include <algorithm>
#include "AngleUtils.h"
#include "GeomUtils.h"

#define DEBUG false
#define TURN_THRESHOLD 20
//---------------------------------------------------------
// Constructor

RecordSwath::RecordSwath(double interval) : m_min_allowable_swath(0),
                         m_has_records(false), m_acc_dist(0),
                         m_interval(interval), m_output_side(BoatSide::Unknown),
                         m_previous_record{0, 0, 0, 0, 0, 0}
{
  /*
  // Define a precision model using 0,0 as the reference origin
	// and 2.0 as coordinates scale.
	geos::geom::PrecisionModel *pm = new geos::geom::PrecisionModel(
    geos::geom::PrecisionModel::FLOATING);

	// Initialize global factory with defined PrecisionModel
	// and a SRID of -1 (undefined).
	m_geom_factory = geos::geom::GeometryFactory::create(pm, -1);
	// We do not need PrecisionMode object anymore, it has
	// been copied to global_factory private storage
	delete pm;
  */

  //m_coverage = m_geom_factory->createPolygon();

  // Initialize the point records
  m_interval_swath[BoatSide::Port] = std::vector<double>();
  m_interval_swath[BoatSide::Stbd] = std::vector<double>();
  m_outer_points[BoatSide::Port] = std::vector<geos::geom::Point*>();
  m_outer_points[BoatSide::Stbd] = std::vector<geos::geom::Point*>();
}

bool RecordSwath::AddRecord(double swath_stbd, double swath_port, double loc_x,
                            double loc_y, double heading, double depth) {
  // Dont add records at duplicate location
  if (loc_x == m_previous_record.loc_x && loc_y == m_previous_record.loc_y
        && heading == m_previous_record.heading)
    return false;

  SwathRecord record = {loc_x, loc_y, heading, swath_stbd, swath_port, depth};
  m_interval_record.push_back(record);
  m_interval_swath[BoatSide::Stbd].push_back(swath_stbd);
  m_interval_swath[BoatSide::Port].push_back(swath_port);

  if (m_has_records) {
    m_acc_dist += distPointToPoint(m_last_x, m_last_y, loc_x, loc_y);
    #if DEBUG
    std::cout << "Accumulated distance: " + std::to_string(m_acc_dist) + "\n";
    #endif

    if (m_acc_dist >= m_interval) {
      #if DEBUG
      std::cout << "Running MinInterval()\n";
      #endif
      m_acc_dist = 0;
      MinInterval();
    } else {
      //Override the min interval on turns to the outside
      double turn = angle180(angle180(heading) - angle180(m_min_record.back().heading));
      if ((turn > TURN_THRESHOLD && m_output_side == BoatSide::Port)
          || (turn < -TURN_THRESHOLD && m_output_side == BoatSide::Stbd)) {
        #if DEBUG
        std::cout << "Adding Turn Based Point\n";
        #endif
        m_min_record.push_back(record);
        m_interval_record.clear();
        m_interval_swath.clear();
      }
    }
  }

  m_last_x = loc_x;
  m_last_y = loc_y;
  m_has_records = true;
  m_previous_record = record;

  // Add progressively to the coverage model
  return AddToCoverage(record);
}

bool RecordSwath::AddToCoverage(SwathRecord record) {
  // Tackle this later
  return true;
}

void RecordSwath::MinInterval() {
  // Get the record from the side we are offsetting
  if (m_output_side == BoatSide::Unknown) {
    throw std::runtime_error("Cannot find swath minimum without output side.");
    return;
  }
  std::vector<double>* side_record = &m_interval_swath[m_output_side];

  std::size_t min_index = 0;
  if (side_record->size() > 0) {
    min_index = std::min_element(side_record->begin(), side_record->end())
      - side_record->begin();
  }

  if (m_interval_record.size() > min_index) {
    // Add the first point if this is the first interval in the record
    if (m_min_record.size() == 0 && min_index != 0) {
      #if DEBUG
      std::cout << "Saving First record of line\n";
      #endif
      m_min_record.push_back(m_interval_record[0]);
    }
    m_min_record.push_back(m_interval_record[min_index]);
    // These are always cleared in the python version
    m_interval_record.clear();
    m_interval_swath.clear();
  }
}

bool RecordSwath::SaveLast() {
  if (m_min_record.size() > 0 && m_interval_record.size() > 0) {
    SwathRecord last_min = m_min_record.back();
    SwathRecord last_rec = m_interval_record.back();
    if (last_min.loc_x != last_rec.loc_x || last_min.loc_y != last_rec.loc_y) {
      #if DEBUG
      std::cout << "Saving last record of line, (" << last_rec.loc_x << ", "
        << last_rec.loc_y << ")\n";
      #endif
      m_min_record.push_back(last_rec);
    }
    return true;
  }
  return false;
}

void RecordSwath::ResetLine() {
  m_interval_record.clear();
  m_min_record.clear();
  m_interval_swath[BoatSide::Stbd].clear();
  m_interval_swath[BoatSide::Port].clear();
  //m_coverage = m_geom_factory->createPolygon();
  m_outer_points[BoatSide::Stbd].clear();
  m_outer_points[BoatSide::Port].clear();
  m_acc_dist = 0;
  m_has_records = false;
}

XYSegList RecordSwath::SwathOuterPts(BoatSide side) {
  XYSegList points;
  std::list<SwathRecord>::iterator record;
  for (record = m_min_record.begin(); record != m_min_record.end(); record++) {
    // #if DEBUG
    // std::cout << "Getting swath outer point for " << record->loc_x
    //   << ", "  << record->loc_y << "\n";
    // #endif
    XYPoint outer_pt = OuterPoint(*record, side);
    points.add_vertex(outer_pt);
  }
  return points;
}

// list<XYPt> RecordSwath::SwathOuterPts(BoatSide side) {
//   list<XYPt> points;
//   std::list<SwathRecord>::iterator record;
//   for (record = m_min_record.begin(); record != m_min_record.end(); record++) {
//     XYPoint outer_pt = OuterPoint(*record, m_output_side);
//     XYPt outer_pt_simple = {outer_point.x(), outer_point.y()}
//     points.add_vertex(outer_pt);
//   }
//   return points;
// }

XYPoint RecordSwath::OuterPoint(const SwathRecord &record, BoatSide side) {
  // Could have SwathRecord be a class with functions to return representation
  // as a vector or point.
  double swath_width = 0;
  double rotate_degs = 0;
  if (side == BoatSide::Stbd) {
    swath_width = record.swath_stbd;
    rotate_degs = 90;
  } else if (side == BoatSide::Port) {
    swath_width = record.swath_port;
    rotate_degs = -90;
  }
  XYVector swath_vector(record.loc_x, record.loc_y, swath_width,
    record.heading);
  swath_vector.augAngle(rotate_degs);
  return XYPoint(swath_vector.xpos() + swath_vector.xdot(),
    swath_vector.ypos() + swath_vector.ydot());
}

std::pair<XYPoint, XYPoint> RecordSwath::LastOuterPoints() {
  if (m_has_records) {
    XYPoint port_point = OuterPoint(m_previous_record, BoatSide::Port);
    XYPoint stbd_point = OuterPoint(m_previous_record, BoatSide::Stbd);
    return std::make_pair(port_point, stbd_point);
  }
  return std::make_pair(XYPoint(), XYPoint());
}

double RecordSwath::SwathWidth(BoatSide side, unsigned int index) {
  if (m_min_record.size() > index) {
    std::list<SwathRecord>::iterator list_record = std::next(m_min_record.begin(),
     index);
    if (side == BoatSide::Stbd) {
      return list_record->swath_stbd;
    } else if (side == BoatSide::Port) {
      return list_record->swath_port;
    }
  }
  return 0;
}

std::vector<double> RecordSwath::AllSwathWidths(BoatSide side) {
  std::vector<double> widths;
  widths.reserve(m_min_record.size());
  std::list<SwathRecord>::iterator list_record;
  for (list_record = m_min_record.begin(); list_record != m_min_record.end();
    list_record++) {
    if (side == BoatSide::Stbd) {
      widths.push_back(list_record->swath_stbd);
    } else if (side == BoatSide::Port) {
      widths.push_back(list_record->swath_port);
    }
  }
  return widths;
}

XYPoint RecordSwath::SwathLocation(unsigned int index) {
  if (m_min_record.size() > index) {
    std::list<SwathRecord>::iterator list_record = std::next(m_min_record.begin(),
     index);
    return XYPoint(list_record->loc_x, list_record->loc_y);
  }
  throw std::out_of_range("Swath index out of range.");
  return XYPoint();
}

bool RecordSwath::ValidRecord() {
  return (m_min_record.size() > 1);
}

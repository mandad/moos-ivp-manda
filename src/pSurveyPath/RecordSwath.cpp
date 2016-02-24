/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: RecordSwath.cpp                                 */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#include "RecordSwath.h"
#include <cmath>
#include "AngleUtils.h"

//---------------------------------------------------------
// Constructor

RecordSwath::RecordSwath(double interval) : m_min_allowable_swath(0),
                         m_has_records(false), m_acc_dist(0),
                         m_interval(interval)
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

  m_coverage = m_geom_factory->createPolygon();

  // Initialize the point records
  m_interval_swath[BoatSide::Port] = std::vector<double>();
  m_interval_swath[BoatSide::Stbd] = std::vector<double>();
  m_outer_points[BoatSide::Port] = std::vector<geos::geom::Point*>();
  m_outer_points[BoatSide::Stbd] = std::vector<geos::geom::Point*>();
}

/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: RecordPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

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

#include <list>
#include <vector>
#include <map>

enum class BoatSide {
  Stbd,
  Port
};

class RecordSwath
{
 public:
   RecordSwath(double interval = 10);
   ~RecordSwath() {};

 private:
   struct SwathRecord {
     double loc_x;
     double loc_y;
     double heading;
     double swath_stbd;
     double swath_port;
   };

 private:
   // Configuration Variables
   double m_min_allowable_swath;

   // State varables
   std::vector<SwathRecord> m_interval_record;
   std::map<BoatSide, std::vector<double>> m_interval_swath;
   double m_last_x;
   double m_last_y;
   bool m_has_records;
   double m_acc_dist;
   double m_interval;

   // Geometry Stuff
   std::map<BoatSide, std::vector<geos::geom::Point*>> m_outer_points;
   geos::geom::Polygon* m_coverage;
   geos::geom::GeometryFactory* m_geom_factory;

};

#endif

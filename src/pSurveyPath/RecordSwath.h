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

//MOOS Headers
#include "XYPoint.h"
#include "XYSegList.h"

#include <list>
#include <vector>
#include <map>

enum class BoatSide {
  Stbd,
  Port
};

class RecordSwath
{
private:
  struct SwathRecord {
    double loc_x;
    double loc_y;
    double heading;
    double swath_stbd;
    double swath_port;
  };

 public:
   RecordSwath(double interval = 10);
   ~RecordSwath() {};
   bool AddRecord(double swath_stbd, double swath_port, double loc_x, double loc_y,
          double heading);
   void ResetLine();
   void SaveLast();
   void GetSwathOuterPts(BoatSide side, XYSegList& points);
   bool GetSwathCoverage(BoatSide side, geos::geom::Polygon& coverage);
   double GetSwathWidth(BoatSide side, unsigned int index);
   std::vector<double> GetAllSwathWidths(BoatSide side);
   XYPoint GetSwathLocation(unsigned int index);

 protected:
   void MinInterval();
   XYPoint GetOuterPoint(SwathRecord record, BoatSide side);

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

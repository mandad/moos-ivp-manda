/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: RecordPath.h                                    */
/*    DATE: 23 Feb 2016                                     */
/************************************************************/

#ifndef SurveyPath_RecordSwath_HEADER
#define SurveyPath_RecordSwath_HEADER

#include <geos.h>
#include <list>

class RecordSwath
{
 public:
   RecordSwath();
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

   // State varables
   std::list<SwathRecord> interval_record;

};

#endif

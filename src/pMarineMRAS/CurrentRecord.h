/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CurrentEstimate.h                               */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#ifndef MarineMRAS_CurrentRecord_HEADER
#define MarineMRAS_CurrentRecord_HEADER

#define SIMPLE_MODE true

#include <list>
#include <utility>
#include <complex>
#include "AngleUtils.h"

struct SpeedInfoRecord {
    SpeedInfoRecord(double time, double sog, double speed_est,
              double hdg, double cog = 1000, double stw = 1000) : time(time),
                         speed_over_ground{sog}, speed_through_water{stw},
                         speed_estimate{speed_est} {
        // 0.5 m/s limit from experimental observation
        if (cog == 1000 || sog < 0.5) {
            valid_cog = false;
        } else {
            valid_cog = true;
            course_over_ground = angle360(cog);
        }
        if (stw == 1000) {
            valid_stw = false;
        } else {
            valid_stw = true;
        }
        heading = angle360(hdg);
    }

    double time;
    double speed_over_ground;
    double speed_through_water;
    double speed_estimate;
    double heading;
    double course_over_ground;
    bool valid_stw;
    bool valid_cog;
};

// SpeedInfoRecord::SpeedInfoRecord(double time, double sog, double speed_est,
//                          double hdg, double cog, double stw) : time(time),
//                          speed_over_ground{sog}, speed_through_water{stw},
//                          speed_estimate{speed_est} {
//     // 0.5 m/s limit from experimental observation
//     if (cog == 1000 || sog < 0.5) {
//         valid_cog = false;
//     } else {
//         valid_cog = true;
//         course_over_ground = angle360(cog);
//     }
//     if (stw == 1000) {
//         valid_stw = false;
//     } else {
//         valid_stw = true;
//     }
//     heading = angle360(hdg);
// }

class CurrentRecord
{

public:
    CurrentRecord(double save_time=3600, int max_records=100);
    ~CurrentRecord() {}
    bool GetAverageCurrent(double &mag, double &heading);
    double GetAverageSpeedDiff();
    bool SaveRecord(SpeedInfoRecord record);
    int NumRecords();

private:
    //Functions
    std::complex<double> VectorDiff(SpeedInfoRecord record);

    //State variables
    std::list<SpeedInfoRecord> m_record_hist;
    std::complex<double> m_average_diff;
    std::list<std::complex<double>> m_vector_hist;

    double m_avg_cog;

    //Configuration variables
    int m_max_records;
    double m_save_time;

};

#endif

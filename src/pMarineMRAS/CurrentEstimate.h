/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CurrentEstimate.h                               */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#ifndef MarineMRAS_CurrentEstimate_HEADER
#define MarineMRAS_CurrentEstimate_HEADER

#include <utility>
#include "AngleUtils.h"

class CurrentEstimate
{

public:
    struct SpeedDiff {
        SpeedDiff(double time, double sog, double speed_est,
                  double hdg, double cog = 1000, double stw = 1000) : time(time), 
                  speed_over_ground{sog}, speed_through_water{stw}, 
                  speed_estimate{speed_est} {
            if (cog == 1000) {
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

    CurrentEstimate(double bin_width=20);
    ~CurrentEstimate() {}  
    double GetSpeedDiff(double heading);
    double SaveHistory(SpeedDiff record);

private:
    //Functions
    int BinnedHeading(double heading);
    
    //State variables
    
    std::map<int, std::pair<double, int>> m_direction_average;
    double m_time_at_speed;

    //Configuration variables
    double m_bin_width;


    std::list<SpeedDiff> m_history;

};

#endif
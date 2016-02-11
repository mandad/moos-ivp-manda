/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: CurrentEstimate.h                               */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#ifndef MarineMRAS_CurrentEstimate_HEADER
#define MarineMRAS_CurrentEstimate_HEADER

#include <utility>

class CurrentEstimate
{

public:
    CurrentEstimate();
    ~CurrentEstimate() {}  

private:
    //Functions
    
    //State variables
    
    std::map<int, std::pair<double, int>> m_direction_average;
    double m_time_at_speed;

    //Configuration variables

    struct SpeedRecord {
        SpeedRecord(double desired_speed, double speed, double heading, 
                    double time) : m_desired_speed{desired_speed}, m_speed{speed},
                    m_heading{heading}, m_time{time} {}

        double m_desired_speed;
        double m_speed;
        double m_heading;
        double m_time;
    };
    std::list<SpeedRecord> m_speed_hist;

};

#endif
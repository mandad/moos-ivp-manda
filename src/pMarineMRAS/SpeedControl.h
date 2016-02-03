/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                             */
/*    FILE: SpeedControl.h                                  */
/*    DATE: 2016-02-01                                      */
/************************************************************/

#ifndef SpeedControl_HEADER
#define SpeedControl_HEADER

#include "ThrustMap.h"
#include <utility>

class SpeedControl
{

public:
    SpeedControl();
    ~SpeedControl() {}

    double Run(double desired_speed, double speed, double heading, double time);
    void SetParameters(std::string thrust_map);

private:
    //Functions
    void InitControls(double speed, double heading);

    //State variables
    bool m_first_run;
    bool m_thrust_map_set;
    double m_thrust_output;
    double m_desired_speed;

    // std::list<double> m_speed_hist;
    // std::list<double> m_time_hist;
    // std::list<double> m_des_speed_hist;
    std::map<int, std::pair<double, int>> m_direction_average;
    double m_time_at_speed;

    //Configuration variables
    ThrustMap m_thrust_map;
    double m_max_thrust;


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
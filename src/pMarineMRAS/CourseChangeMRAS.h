/************************************************************/
/*    NAME: Damian Manda                                    */
/*    ORGN: UNH                                              */
/*    FILE: CourseChangeMRAS.h                                     */
/*    DATE: 2015-12-07                            */
/************************************************************/

#ifndef CourseChangeMRAS_HEADER
#define CourseChangeMRAS_HEADER

class CourseChangeMRAS
{
public:
    CourseChangeMRAS();
    CourseChangeMRAS(double dfKStar, double dfTauStar, double dfZ, double dfBeta, 
        double dfAlpha, double dfGamma, double dfXi, double dfRudderLimit, 
        double dfCruisingSpeed, double dfShipLength, double dfMaxROT);
    ~CourseChangeMRAS() {}

    double Run(double dfDesiredHeading, double dfMeasuredHeading, double dfMeasuredROT,
     double dfSpeed, double dfTime);
    void SetParameters(double dfKStar, double dfTauStar, double dfZ, 
        double dfBeta, double dfAlpha, double dfGamma, double dfXi, 
        double dfRudderLimit, double dfCruisingSpeed, double dfShipLength, 
        double dfMaxROT);
    bool NewHeading(double dfSpeed);
    void ResetModel(double dfHeading, double dfROT);
    void UpdateModel(double dfDesiredHeading, double dfDeltaT);
    double TwoSidedLimit(double dfNumToLimit, double dfLimit);
    std::string GetStatusInfo();


 private: // Configuration variables
    double m_dfKStar;
    double m_dfTauStar;
    double m_dfZ;
    double m_dfBeta;
    double m_dfAlpha;
    double m_dfGamma;
    double m_dfXi;
    double m_dfRudderLimit;
    double m_dfMaxROT;
    double m_dfCruisingSpeed;
    double m_dfShipLength;

 private: //Derived variables
    double m_dfTauM;
    double m_dfKpm;
    double m_dfP12;
    double m_dfP22;

 private: // State variables
    double m_dfKp;
    double m_dfKd;
    double m_dfKi;

    double m_dfPreviousTime;
    double m_dfPreviousHeading;
    double m_dfMeasuredHeading;
    double m_dfCourseChangeTime;
    long   m_lIterations;
    bool   m_bFirstRun;
    bool   m_bParametersSet;
    double m_dfRudderOut;

    double m_dfKp0;
    double m_dfKd0;
    double m_dfKi0;

    double m_dfModelHeading;
    double m_dfModelROT;
    double m_dfSeriesHeading;
    double m_dfSeriesROT;
    double m_dfPsiRefP;
    double m_dfPsiRefPP;
    
};

#endif